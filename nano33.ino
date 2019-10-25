#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_APDS9960.h>
#include <PDM.h>
#include <arduinoFFT.h>

arduinoFFT FFT = arduinoFFT();

float temperature, humidity, pressure;
float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float magnet_x, magnet_y, magnet_z;

char linebuf_atm[80];
char linebuf_acc[80];
char linebuf_gyro[80];
char linebuf_magnet[80];
char linebuf_color[80];
char linebuf_noise[80];

int r, g, b, w;

#define SAMPLES 256
#define SAMPLING_FREQUENCY 16000
// buffer to read samples into, each sample is 16-bits
short wform[SAMPLES];
// FFT real and imaginary vectors
double vReal[SAMPLES];
double vImag[SAMPLES];

// number of samples read
volatile int samplesRead;

double ftsum = 0.0;

int ledState = LOW;

void setup() {
  Serial.begin(9600);
  delay(100);

  HTS.begin();
  delay(100);

  BARO.begin();
  delay(100);
  // The baro sensor reads wrong first time after init
  // so let's do a throw-away read here.
  pressure = BARO.readPressure(MILLIBAR);
  delay(100);

  IMU.begin();
  delay(100);

  APDS.begin();
  delay(100);

  PDM.onReceive(onPDMdata);
  delay(100);
  PDM.begin(1, SAMPLING_FREQUENCY);
  delay(100);

  pinMode(LED_BUILTIN, OUTPUT);

  // Let's allow things to settle down.
  delay(100);
}

void loop() {

  while (! APDS.colorAvailable()) {
    // always wait a bit after APDS.colorAvailable()
    delay(5);
  }
  APDS.readColor(r, g, b, w);
  delay(10);

  temperature = HTS.readTemperature();
  delay(10);
  humidity = HTS.readHumidity();
  delay(10);
  pressure = BARO.readPressure(MILLIBAR);
  delay(10);

  IMU.readAcceleration(acc_x, acc_y, acc_z);
  delay(10);
  IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  delay(10);
  IMU.readMagneticField(magnet_x, magnet_y, magnet_z);
  delay(10);

  // wait for sound samples to be read
  if (samplesRead) {
    delay(10);
    for (int i = 0; i < SAMPLES; i++) {
      // load the waveform into the FFT real vector
      vReal[i] = double(wform[i]);
      // FFT imaginary vector is zero
      vImag[i] = 0.0;
    }

    // compute the spectrum
    // at the end of the sequence, vReal will contain the spectrum
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    // calculate the sum of all spectral components
    // with log10() to adjust for perceptual scale
    ftsum = 0.0;
    // don't start i at 0, low frequencies are too noisy
    // stop at sR / 2 since the spectrum is repeated symmetrically after that
    for (int i = 8; i < samplesRead / 2; i++) {
      ftsum += log10(vReal[i]);
    }

    // clear the samples read count
    samplesRead = 0;
  }

  // format the output in CSV
  sprintf(linebuf_atm,
    "a,%f,%f,%f",
    temperature, humidity, pressure);
  sprintf(linebuf_acc,
    "g,%f,%f,%f",
    acc_x, acc_y, acc_z);
  sprintf(linebuf_gyro,
    "r,%f,%f,%f",
    gyro_x, gyro_y, gyro_z);
  sprintf(linebuf_magnet,
    "m,%f,%f,%f",
    magnet_x, magnet_y, magnet_z);
  sprintf(linebuf_color,
    "l,%u,%u,%u,%u",
    r, g, b, w);
  sprintf(linebuf_noise,
    "n,%u",
    int(ftsum));

  // print the measured values
  Serial.println(linebuf_atm);
  Serial.println(linebuf_acc);
  Serial.println(linebuf_gyro);
  Serial.println(linebuf_magnet);
  Serial.println(linebuf_color);
  Serial.println(linebuf_noise);
  Serial.println("END");

  // blink the LED every cycle
  ledState = ledState ? LOW: HIGH;
  digitalWrite(LED_BUILTIN,  ledState);
  
  delay(10);
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(wform, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
