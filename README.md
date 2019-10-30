# WeatherStation

**Status:** Under construction. Arduino C++ code is near completion. Python code is working for the most part, but incomplete.

## Goals

![Arduino](/images/nano33.jpg)

Use an [Arduino Nano 33 BLE Sense](https://store.arduino.cc/usa/nano-33-ble-sense) to read environmental parameters:
- temperature
- relative humidity
- air pressure
- acceleration in 3D (just gravity if not moving - might work as an earthquake detector)
- gyroscope data (rotation, in degrees / second) - should be near zero for a stationary device
- magnetic field in 3D (Earth's own field most of the time)
- light: red, green, blue components, plus overall intensity
- noise

![RPi0](/images/rpi0.jpg)

Use a [Raspberry Pi Zero W](https://www.raspberrypi.org/products/raspberry-pi-zero-w/) to read data from the Arduino and write it into a [Graphite](https://graphiteapp.org/) server as timeseries data. The RPi should cache data temporarily if Graphite is too busy or unavailable.

Graphite could do several things:
- store data for a long time
- expose it to a Web GUI such as [Grafana](https://grafana.com/) for visualization
- make data accessible via a REST API for analysis (trends, correlations, etc)

## FFT (Fast Fourier Transform) for noise

Most ambiental parameters are straightforward.

Noise loudness requires some math. The Arduino sensor could capture a snapshot of the noise waveform. From there, we need to estimate loudness.

One way to do this is to do FFT from the waveform and generate the spectral components. Then sum all components on a logarithmic scale. This ought to be close enough to perceived loudness.

Surprisingly, the Arduino is fast enough for real time FFT.

## Example of Grafana dashboard with sensor data

![Grafana](/images/grafana-test.png)
