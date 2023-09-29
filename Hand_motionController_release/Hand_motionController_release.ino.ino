#include "I2Cdev.h"
#include "MPU6050.h"

//#define DEBUG

#define MAX_ANGLE_ROLL 45       // максимальны угол наклона рукоятки в градусах по roll (наклоны влево-вправо)
#define MAX_ANGLE_PITCH 30      // максимальны угол наклона рукоятки в градусах по pitch (наклоны вперед - назад)
#define TRIGGER_MIN 0           // минимальное значение с АЦП курка
#define TRIGGER_MAX 1023        // максимальное значение с АЦП курка

/* РЕВЕРСЫ */
#define ROLL_REV 0             // реверс по roll (наклоны влево-вправо) 0 - норм, 1 - реверс
#define PITCH_REV 0            // реверс по pitch (наклоны вперед - назад)
#define THR_REV 0              // реверс курка (значения с потенциометра)

#define PPM_PIN 9              // пин ppm выхода
#define TRIGGER_PIN A0         // пин для подключения потенциометра
#define LED_PIN 6              // пин подключения светодиода
#define BTN_PIN 7              // пин подключения кнопки


#define BUFFER_SIZE 100
#define TIME_GYRO 2000
#define COMPL_K 0.07

#define CH_MAX  7
#define PPM_FrLen 22500
#define PPM_PulseLen 300
uint16_t ppm[CH_MAX];
uint16_t ppm_signal;
uint16_t thr;

int16_t offset[2];

MPU6050 mpu;

int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
uint16_t raw_pot;
float accXangle; // Angle calculate using the accelerometer
float accYangle;
float temp;
float gyroXangle = 0; // Angle calculate using the gyro
float gyroYangle = 0;
float compAngleX = 0; // Calculate the angle using a Kalman filter
float compAngleY = 0;
float angleX = 0;
float angleY = 0;

uint32_t time_timer;
uint32_t timer;
uint32_t last_time = 0;




float gyro_x_zero, gyro_y_zero, gyro_z_zero;


void setup() {

  pinMode(PPM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }


  for (int i = 0; i < CH_MAX; i ++) {
    ppm[i] = 1000;
  }

  Wire.begin();

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif

  mpu.initialize();
  calibrateMPU();
  delay(2);

  /*Настройка прерывания по таймеру для ppm сигнала*/
  byte scale = 2;
  OCR1A = 50 * scale;
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TCCR1A |= _BV(COM1A0);
  TIMSK1 |= (1 << OCIE1A);
  sei();

  if (!digitalRead(BTN_PIN)) {
    offset[0] = 0;
    offset[1] = 0;
  }
}

void loop() {

  if ((millis() - last_time) >= 20) {
    last_time = millis();

    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
    calculateAngles();

    if (ROLL_REV) angleY = - compAngleY;
    else angleY = compAngleY;
    if (PITCH_REV) angleX = - compAngleX;
    else angleX = compAngleX;

    ppm[0] = ((angleY + offset[1]) * (512 / MAX_ANGLE_ROLL)) + 1500;
    if (ppm[0] > 2000) ppm[0] =  2000;
    else if (ppm[0] < 1000) ppm[0] = 1000;

    ppm[1] = ((angleX + offset[0]) * (512 / MAX_ANGLE_PITCH)) + 1500;
    if (ppm[1] > 2000) ppm[1] =  2000;
    else if (ppm[1] < 1000) ppm[1] = 1000;


    if (THR_REV) raw_pot = ~analogRead(TRIGGER_PIN) & 0x3FF;
    else raw_pot = analogRead(TRIGGER_PIN);

    thr = map(raw_pot, TRIGGER_MIN, TRIGGER_MAX, 1000, 2000);
    if (thr <= 1030)thr = 1000;
    if (thr > 2000)thr = 2000;
    ppm[2] = thr;

#ifdef DEBUG
    Serial.print("angle ROLL - "); Serial.print(compAngleY); Serial.print(", \t");
    Serial.print("angle PITCH - "); Serial.print(compAngleX); Serial.print(", \t");
    Serial.print("ANALOG RAW - "); Serial.print(raw_pot); Serial.print(", \t");
    Serial.print(ppm[0]); Serial.print(", ");
    Serial.print(ppm[1]); Serial.print(", ");
    Serial.print(ppm[2]); Serial.println();
#endif

  }


  if (!digitalRead(BTN_PIN)) {
    offset[0] = compAngleX;
    offset[1] = compAngleY;
  }

}




void calibrateMPU() {
  int i = 0;
  int NUM_CALIBRATE = 100;
  while (i < NUM_CALIBRATE) {
    if ( time_timer < micros() ) {
      time_timer = micros() + TIME_GYRO;
      mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
      gyro_x_zero += gyroX;
      gyro_y_zero += gyroY;
      gyro_z_zero += gyroZ;
      i++;
    }
  }
  gyro_x_zero /= NUM_CALIBRATE;
  gyro_y_zero /= NUM_CALIBRATE;
  gyro_z_zero /= NUM_CALIBRATE;
}

void calculateAngles() {

  accYangle = atan2(accX, accZ) * RAD_TO_DEG;
  accXangle = atan2(accY, accZ) * RAD_TO_DEG;
  float gyroXrate = (float)gyroX / 131.0;
  float gyroYrate = -((float)gyroY / 131.0);
  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * ((float)(micros() - timer) / 1000000);
  gyroYangle += gyroYrate * ((float)(micros() - timer) / 1000000);


  compAngleX = ((float)(1 - COMPL_K) * (compAngleX + (gyroXrate * (float)(micros() - timer) / 1000000))) + (COMPL_K * accXangle);
  compAngleY = ((float)(1 - COMPL_K) * (compAngleY + (gyroYrate * (float)(micros() - timer) / 1000000))) + (COMPL_K * accYangle);


  timer = micros();
}



ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;
  static boolean state = true;
  byte scale = 2;
  digitalWrite(PPM_PIN, LOW);  //set the PPM signal pin to the default state (off)

  if (state) { //start pulse
    digitalWrite(PPM_PIN, LOW);
    OCR1A = PPM_PulseLen * scale;
    state = false;
  }
  else { //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
    digitalWrite(PPM_PIN, LOW);
    state = true;

    if (cur_chan_numb >= CH_MAX) {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;//
      OCR1A = (PPM_FrLen - calc_rest) * scale;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * scale;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}
