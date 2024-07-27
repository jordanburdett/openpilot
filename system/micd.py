#!/usr/bin/env python3
import numpy as np
from functools import cache

from cereal import messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.retry import retry
from openpilot.common.swaglog import cloudlog

RATE = 10
FFT_SAMPLES = 4096
REFERENCE_SPL = 2e-5  # newtons/m^2
SAMPLE_RATE = 44100
SAMPLE_BUFFER = 4096  # approx 100ms


@cache
def get_a_weighting_filter():
  # Calculate the A-weighting filter
  # https://en.wikipedia.org/wiki/A-weighting
  freqs = np.fft.rfftfreq(FFT_SAMPLES, d=1 / SAMPLE_RATE)
  A = 12194 ** 2 * freqs ** 4 / ((freqs ** 2 + 20.6 ** 2) * (freqs ** 2 + 12194 ** 2) * np.sqrt((freqs ** 2 + 107.7 ** 2) * (freqs ** 2 + 737.9 ** 2)))
  return A / np.max(A)


@cache
def get_hanning_window(size):
  return np.hanning(size)


def calculate_spl(measurements):
  # https://www.engineeringtoolbox.com/sound-pressure-d_711.html
  sound_pressure = np.sqrt(np.mean(measurements ** 2))  # RMS of amplitudes
  if sound_pressure > 0:
    sound_pressure_level = 20 * np.log10(sound_pressure / REFERENCE_SPL)  # dB
  else:
    sound_pressure_level = 0
  return sound_pressure, sound_pressure_level


def apply_a_weighting(measurements: np.ndarray) -> np.ndarray:
  # Generate a Hanning window of the same length as the audio measurements
  hanning_windowed_measurements = measurements * get_hanning_window(len(measurements))
  # Apply the A-weighting filter to the signal
  weighted_fft = np.fft.rfft(hanning_windowed_measurements) * get_a_weighting_filter()
  weighted_measurements = np.fft.irfft(weighted_fft)

  return np.abs(weighted_measurements)


class Mic:
  def __init__(self):
    self.rk = Ratekeeper(RATE)
    self.pm = messaging.PubMaster(['microphone'])

    self.measurements = np.empty(0)

    self.sound_pressure = 0
    self.sound_pressure_weighted = 0
    self.sound_pressure_level_weighted = 0

  def update(self):
    msg = messaging.new_message('microphone', valid=True)
    msg.microphone.soundPressure = float(self.sound_pressure)
    msg.microphone.soundPressureWeighted = float(self.sound_pressure_weighted)

    msg.microphone.soundPressureWeightedDb = float(self.sound_pressure_level_weighted)

    self.pm.send('microphone', msg)
    self.rk.keep_time()

  def callback(self, indata, frames, time, status):
    """
    Using amplitude measurements, calculate an uncalibrated sound pressure and sound pressure level.
    Then apply A-weighting to the raw amplitudes and run the same calculations again.

    Logged A-weighted equivalents are rough approximations of the human-perceived loudness.
    """

    self.measurements = np.concatenate((self.measurements, indata[:, 0]))

    while self.measurements.size >= FFT_SAMPLES:
      measurements = self.measurements[:FFT_SAMPLES]

      self.sound_pressure, _ = calculate_spl(measurements)
      measurements_weighted = apply_a_weighting(measurements)
      self.sound_pressure_weighted, self.sound_pressure_level_weighted = calculate_spl(measurements_weighted)

      self.measurements = self.measurements[FFT_SAMPLES:]

  @retry(attempts=7, delay=3)
  def get_stream(self, sd):
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    return sd.InputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER)

  def micd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd

    with self.get_stream(sd) as stream:
      cloudlog.info(f"micd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        self.update()


def main():
  mic = Mic()
  mic.micd_thread()


if __name__ == "__main__":
  main()
