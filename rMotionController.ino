/*
    DIY Arduino - FPV Motion Controller

    Created by Red Fraction (Daniil Sidorenko) https://x-red.dev/
    Inspired by Novoselov's project https://github.com/NovoselovMilk/Hand_Controller

    Red Fraction © 2023
*/

//#define DEBUG
#define USB_JOYSTICK    // Supported only on leonardo / pro micro

#include "MPU6050_6Axis_MotionApps20.h"
#include "PPMEncoder.h"

#ifdef USB_JOYSTICK
#include "Joystick.h"
#endif

// REVERSES
//#define YAW_REV	// YAW reverse
#define ROLL_REV	// ROLL reverse
#define PITCH_REV	// PITCH reverse
//#define THR_REV	// реверс курка

// GYRO/ACC DEGRESS RANGE
#define MAX_ANGLE_ROLL    30	// ROLL
#define MAX_ANGLE_PITCH   30	// PITCH
#define MAX_ANGLE_YAW     40	// YAW

// TRIGGER RANGE
#define TRIGGER_MIN   415	// TRIGGER MIN VALUE (PUT YOUR VALUE HERE)
#define TRIGGER_MAX   495 // TRIGGER MAX VALUE (PUT YOUR VALUE HERE)

// PINOUT SETUP
#define PPM_PIN     A3	// PPM signal out pin
#define TRIGGER_PIN A0	// Trigger pin
#define LED_PIN     6	// LED_PIN
#define BTN_PIN     7	// Pin for calibration button
#define SWITCH_1    14	// Switch up pin
#define SWITCH_2    16	// Switch down pin

// PPM SIGNAL VALUE RANGE - DO NOT EDIT IF YOU DONT KNOW WHAT YOU ARE DOING
#define MIN 988
#define MID 1500
#define MAX 2012

// PPM CH COUNT - Max 8 channels
#define PPM_CH 4 

// CHANNELS ORDER - by default AETR (Starts from 0)
#define ROLL_CH     0  
#define PITCH_CH    1
#define THR_CH      2
#define YAW_CH      3

bool thrHold = false;

uint16_t ppm[PPM_CH];

uint16_t raw_pot;
uint16_t thr;

bool calibrated = false;

/* --------------------------------------------------------*/
/* ------------------------  MPU  ------------------------ */
/* --------------------------------------------------------*/
#define INTERRUPT_PIN 2

#define YAW_AXIS    yrp[0]
#define ROLL_AXIS   yrp[1]
#define PITCH_AXIS  yrp[2]

MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float yrp[3];           // [yaw, roll, pitch]   yaw/roll/pitch container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void calibrateMPU() {
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);
    setNeutral();
}

void _mpu_init() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    #ifdef DEBUG
    while (!Serial);
    #endif

    #ifdef DEBUG
    Serial.println(F("Initializing I2C devices..."));
    #endif

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    devStatus = mpu.dmpInitialize();

    // MAGIC NUMBERS
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        #ifdef DEBUG
        Serial.println(F("Enabling DMP..."));
        #endif
        mpu.setDMPEnabled(true);

        #ifdef DEBUG
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        #endif

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        #ifdef DEBUG
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        #endif
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        #ifdef DEBUG
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        #endif
    }
}

void _mpu_loop() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
        #ifdef YAW_REV
          yrp[0] = -yrp[0];
        #endif
        #ifdef ROLL_REV
          yrp[1] = -yrp[1];
        #endif
        #ifdef PITCH_REV
          yrp[2] = -yrp[2];
        #endif

    }
}
/* --------------------------------------------------------*/
/* ----------------------- END MPU  ---------------------- */
/* --------------------------------------------------------*/

/* --------------------------------------------------------*/
/* ---------------------- rMotion  ----------------------- */
/* --------------------------------------------------------*/
void ledOut() {
	digitalWrite(LED_PIN, LOW);
	delay(100);
	digitalWrite(LED_PIN, HIGH);
	delay(100);
	digitalWrite(LED_PIN, LOW);
}

void setNeutral() {
    for (uint8_t i = 0; i < PPM_CH + 1; i++) {
        ppm[i] = MID;
    }
}

#ifndef USB_JOYSTICK
void send_ppm() {
    for (uint8_t i = 0; i < PPM_CH + 1; i++) {
        ppmEncoder.setChannel(i, ppm[i]);
    }
}
#endif

void setAER(float val, uint8_t ch, uint8_t max_angle) {
        
    ppm[ch] = map((val * 180/M_PI), -max_angle, max_angle, MIN, MAX);

    if (ppm[ch] > MAX) {
        ppm[ch] = MAX;
    } else if (ppm[ch] < MIN) {
        ppm[ch] = MIN;
    }
}

// void setOffset() {
//     // TODO: Solve Z offset problem
// }

void setThr(uint8_t ch) {
    
    #ifdef THR_REV
    raw_pot = ~analogRead(TRIGGER_PIN) & 0x3FF;
    #elif !THR_REV
    raw_pot = analogRead(TRIGGER_PIN);
    #endif

    if (thrHold) return;
    
    thr = map(raw_pot, TRIGGER_MIN, TRIGGER_MAX, MIN, MAX);
    
    if (thr <= MIN + 30){
        thr = MIN;
      }
        
    if (thr > MAX) 
        thr = MAX;

    ppm[ch] = thr;
}

void setBool(bool val, uint8_t ch) {
  ppm[ch] = val ? MIN : MAX;
}

#ifdef DEBUG
void chDebugLog() {
    Serial.print("\t THR(RAW)-"); Serial.print(raw_pot); 
    Serial.print("\t");

    Serial.print("\tPPM - ");
    for (int i = 0; i <= PPM_CH; i++) {
        if (i == 0) {
            Serial.print(ppm[i]);
        } else {
            Serial.print(", "); Serial.print(ppm[i]);
        }
    }
    Serial.print("\t SW1 - "); Serial.print(digitalRead(SWITCH_1)); 
    Serial.print("\t SW2 - "); Serial.print(digitalRead(SWITCH_2));

    if (thrHold) {
        Serial.print("\t THR HOLDED -");
    }
    Serial.println();
} 
#endif

#ifdef USB_JOYSTICK
    Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
        0,      0,              // Button Count, Hat Switch Count
        true,   true,   true,   // X , Y, Z Axis
        false,  false,  false,  // No Rx, Ry, or Rz
        false,  true,           // No rudder or throttle
        false,  false,  false   // No accelerator, brake, or steering
    ); 
#endif

   
void setup() {

    #ifdef USB_JOYSTICK
    Joystick.begin();
    Joystick.setXAxisRange(MIN, MAX);
    Joystick.setYAxisRange(MIN, MAX);
    Joystick.setZAxisRange(MIN, MAX);
    Joystick.setThrottleRange(MIN, MAX);
    #endif

    pinMode(SWITCH_1, INPUT_PULLUP);
    pinMode(SWITCH_2, INPUT_PULLUP);

    #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("Initializing I2C devices...");
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    #endif

    ppmEncoder.begin(PPM_PIN);

    _mpu_init();
    calibrateMPU();
}

void loop() {
    delay(20);
    _mpu_loop();

    // A E T R
    setAER(ROLL_AXIS, ROLL_CH, MAX_ANGLE_ROLL);
    setAER(PITCH_AXIS, PITCH_CH, MAX_ANGLE_PITCH);
    setThr(THR_CH);
    setAER(YAW_AXIS, YAW_CH, MAX_ANGLE_YAW);

    // Throttle hold
    if (digitalRead(SWITCH_1) == 0) {
        if (!thrHold) {
            thrHold = true;
        }
    } else {
        if (thrHold) {
            thrHold = false;
        }
    }

    // MPU Calibrate / Neutral
    if (digitalRead(SWITCH_2) == 0 || digitalRead(BTN_PIN) == 1) {
        if (!calibrated) {
            calibrated = true;
            calibrateMPU();
            #ifdef DEBUG
            Serial.println("CALIBRATED");
            #endif
        }
        setNeutral();
    } else {
        if (calibrated) {
            #ifdef DEBUG
            Serial.println("Already calibrated");
            #endif
            calibrated = false;
        }
    }

    #ifdef DEBUG
    chDebugLog();
    #endif

    #ifndef USB_JOYSTICK
    send_ppm();
    #endif
    #ifdef USB_JOYSTICK
    Joystick.setXAxis(ppm[ROLL_CH]);
    Joystick.setYAxis(ppm[PITCH_CH]);
    Joystick.setZAxis(ppm[THR_CH]);
    Joystick.setThrottle(ppm[YAW_CH]);
    #endif
}
/* --------------------------------------------------------*/
/* --------------------- END rMotion  -------------------- */
/* --------------------------------------------------------*/
