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

char linebuf_all[200];

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

short apds_loop;
#define APDS_MAX 50

double ftsum = 0.0;

short srelax = 20;

int ledState = LOW;

void setup() {
  Serial.begin(115200);
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

  apds_loop = 0;
  while (! APDS.colorAvailable()) {
    // always wait a bit after APDS.colorAvailable()
    delay(srelax);
    // don't get stuck
    if (++apds_loop > APDS_MAX) {
      break;
    }
  }
  if (apds_loop <= APDS_MAX) {
    APDS.readColor(r, g, b, w);
    delay(srelax);
  } else {
    r = 0;
    g = 0;
    b = 0;
    w = 0;
  }

  temperature = HTS.readTemperature();
  delay(srelax);
  humidity = HTS.readHumidity();
  delay(srelax);
  pressure = BARO.readPressure(MILLIBAR);
  delay(srelax);

  IMU.readAcceleration(acc_x, acc_y, acc_z);
  delay(srelax);
  IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  delay(srelax);
  IMU.readMagneticField(magnet_x, magnet_y, magnet_z);
  delay(srelax);

  // wait for sound samples to be read
  if (samplesRead) {
    delay(srelax);
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

  sprintf(linebuf_all,
    "a,%.2f,%.1f,%.2f,g,%.2f,%.2f,%.2f,r,%.2f,%.2f,%.2f,m,%.1f,%.1f,%.1f,l,%u,%u,%u,%u,n,%u",
    temperature, humidity, pressure,
    acc_x, acc_y, acc_z,
    gyro_x, gyro_y, gyro_z,
    magnet_x, magnet_y, magnet_z,
    r, g, b, w,
    int(ftsum));

  Serial.println(linebuf_all);

  // blink the LED every cycle
  ledState = ledState ? LOW: HIGH;
  digitalWrite(LED_BUILTIN,  ledState);
  
  delay(srelax);
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(wform, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
