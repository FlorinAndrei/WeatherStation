# WeatherStation

Read environmental metrics:
- temperature
- relative humidity
- air pressure
- acceleration in 3D (or just gravity when not moving - might work as an earthquake detector)
- gyroscope data (rotation, in degrees / second) - should be near zero for a stationary device
- magnetic field in 3D (Earth's own field most of the time)
- light: RGB components plus overall intensity
- noise

Send the metrics to a data store such as Graphite. Try hard to not lose data: read the sensors often, cache data for a while when Graphite is not available.

## Software

The code is lavishly commented. Read the comments in addition to this text.

### Arduino

The Arduino dialect of C++. Simply read data and present it to serial output. Other than that, do as little as possible - move fast and save CPU cycles. To capture noise, the Fast Fourier Transform is unavoidable (and quite compute-heavy). [This is the code](/nano33/nano33.ino).

The main loop runs several times each second, gathering metrics from sensors each time.

Rarely, when a sensor does not respond the right way, the main loop could freeze. For this reason, this app initializes a hardware watchdog in the Arduino CPU - if the watchdog is not pinged regularly by the main loop, it sends a hard reset to the whole device.

Thread on the Arduino forum regarding the loop() freeze bug: https://forum.arduino.cc/index.php?topic=643883.0

Thread on Nordic Semi Devzone regarding the watchdog: https://devzone.nordicsemi.com/f/nordic-q-a/53904/nrf52840-watchdog-for-arduino-nano-33-ble-sense

### Python

Python for [the parser / cache / logger](weather_station.py). Multithreaded to reduce interruptions. Cache data in memory when storage is offline.

All sensor readings within the same second are averaged by the parser. This reduces noise; the data store cannot deal with a time resolution better than 1 second anyway.

## Hardware

### Arduino

![Arduino](/images/nano33.jpg)

The [Arduino Nano 33 BLE Sense](https://store.arduino.cc/usa/nano-33-ble-sense) is used to read environmental parameters.

Surprisingly, the Nano 33 is fast enough to do FFT on the noise signal in real time. This would have been difficult with older platforms.

### Raspberry Pi

![RPi0](/images/rpi0.jpg)

The [Raspberry Pi Zero W](https://www.raspberrypi.org/products/raspberry-pi-zero-w/) is used to read data from the Arduino, parse it, cache it when necessary, and write it into a [Graphite](https://graphiteapp.org/) server as timeseries data. It runs the Python parser / cache / logger.

### The whole system

The Arduino is plugged into one of the USB ports on the Pi Zero. This provides power to the Arduino, and also establishes the serial/USB connection between devices. Power to the whole system is provided via the second USB port on the Pi Zero.

The Pi Zero is on the local WiFi network. That's how it connects to the Graphite server.

## Sensor corrections - linear regression

The temperature sensor on the Nano 33 seems quite inaccurate. At mid-range outdoors temperatures, measured in Celsius, its readings differ from reality by half a dozen degrees or so. The error appears to be linear.

Fortunately, we have a public weather station nearby, whose readings we could use to calibrate the Arduino:
- read the temperature on the Arduino
- get the real temperature from the public weather station
- write the two values in a CSV file - this makes one data point

If enough data points are collected, linear regression could be performed on the data, which would provide correction parameters for the Arduino. If **y** is the real temperature, and **x** is the temperature indicated by the Arduino, and **a** and **b** are the correction parameters, then:

```
y = ax + b
```

The data points are collected in [this CSV file](weather-temp.csv).

The linear regression code is in [the Jupyter notebook](linear_regression_temp_sensor.ipynb). At the end of the notebook you can see the most recent correction parameters; the value of **b** (offset) is quite substantial, but **a** (the slope) is close to 1. Bad offset, decent slope, good enough linearity so far.

This code does not apply corrections to the temperature readings - since correcting the sensor is an ongoing project, changing the corrections would alter the data in Graphite all the time. We chose to write into Graphite the "wrong" temperature. Corrections are applied in Grafana, when the data is visualized.

## FFT (Fast Fourier Transform) for noise

Most ambiental parameters are straightforward.

Noise requires some math. The Arduino sensor captures a snapshot of the noise waveform. From there, we need to estimate loudness.

One way to do this is to do FFT on the waveform and generate the spectral components. Then sum all components on a logarithmic scale. This ought to be close enough to perceived loudness - the human ear is a logarithmic sensor.

## Data store and user interface

Graphite performs several functions:
- long term data storage
- expose data to a Web GUI such as [Grafana](https://grafana.com/) for visualization
- make data accessible via a REST API for analysis (trends, correlations, etc)

Example of Grafana dashboard showing sensor data:

![Grafana](/images/grafana.png)
