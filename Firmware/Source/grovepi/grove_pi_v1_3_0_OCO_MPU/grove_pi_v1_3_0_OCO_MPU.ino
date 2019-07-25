#include "ChainableLED.h"
#include "DHT.h"
#include "Encoder.h"
#include "Grove_LED_Bar.h"
#include "TM1637.h"
#include "TimerOne.h"
#include "IRremote.h"
// For IMU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Wire.h>

// Start IMU6050
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

MPU6050 mpu;
   // MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize=0;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

   // orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// End IMU6050

DHT dht;
Grove_LED_Bar ledbar[6]; // 7 instances for D2-D8, however, max 4 bars, you
                         // can't use adjacent sockets, 4 pin display
TM1637 fourdigit[6];     // 7 instances for D2-D8, however, max 4 displays, you
                         // can't use adjacent sockets, 4 pin display
ChainableLED rgbled[6];  // 7 instances for D2-D8

IRrecv irrecv;          // object to interface with the IR receiver
decode_results results; // results for the IR receiver

#define SLAVE_ADDRESS 0x04

#define dust_sensor_read_cmd 10
#define dust_sensor_en_cmd 14
#define dust_sensor_dis_cmd 15
#define dust_sensor_int_cmd 9
#define dust_sensor_read_int_cmd 6

#define encoder_read_cmd 11
#define encoder_en_cmd 16
#define encoder_dis_cmd 17

#define flow_read_cmd 12
#define flow_en_cmd 18
#define flow_dis_cmd 13

#define ir_read_cmd 21
#define ir_recv_pin_cmd 22
#define ir_read_isdata 24

#define data_not_available 23

volatile uint8_t cmd[5];
volatile int index = 0;
volatile int flag = 0;
volatile byte b[21], float_array[4], dht_b[21];
volatile bool need_extra_loop = false;

volatile int run_once;
unsigned char dta[21];
int length, i;
int aRead = 0;
byte accFlag = 0, clkFlag = 0;
int8_t accv[3];
byte rgb[] = {0, 0, 0};

// Dust sensor variables:
volatile uint16_t lowpulseoccupancy = 0;
unsigned long latest_dust_val = 0;
volatile unsigned long t, pulse_end, pulse_start, duration;
volatile int dust_latest = 0;
unsigned long starttime;
unsigned long sampletime_ms = 30000; // sample 30s ;
int dust_run_bk = 0;
int l_status;

// Pins used for interrupt routines
volatile uint8_t dust_sensor_pin = 2;
uint8_t flow_sensor_pin = 2;

// Encoder variable
int index_LED;
volatile byte enc_val[3];
// Given it's own I2C buffer so that it does not corrupt the
// data from other sensors when running in background
int enc_run_bk = 0; // Flag for first time setup

// Flow sensor variables
volatile int NbTopsFan; // measuring the rising edges of the signal
volatile byte
    flow_val[3]; // Given it's own I2C buffer so that it does not corrupt the
                 // data from other sensors when running in background
int Calc;
int hallsensor = 2; // The pin location of the sensor
int flow_run_bk = 0;
long flow_read_start;
int pin;
int j;

// all user-defined functions
void processIO();
void flushI2C();
void receiveData();
void sendData();
void rpm();
void readPulseDust();

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void mpu6050ReadValue(){
  float afloat=10.1; // sumulate 1 value = 10.1
  float *buffer;

  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  afloat=q.x;
  
  buffer=&afloat;
  byte *b1 = (byte *)buffer;
  byte status=1;
  
  b[0] = cmd[0];
  b[1] = status;
  for (j = 2; j < 6; j++)
     b[j] = b1[j - 1];
}

void setup() {
  Serial.begin(38400); // start serial for output
  Wire.begin(SLAVE_ADDRESS);
  Wire.setClock(400000); // 400kHz I2C clock for MPU. Comment this line if having compilation difficulties

  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();

// supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  DDRD |= 0x10;
  PORTD &= ~0x10;
}

void processIO() {
  long dur, RangeCm;
  // Dust sensor can run in background so has a dedicated if condition
  if (dust_run_bk) {
    float this_time = millis();
    float current_time = this_time - starttime;
    if (current_time > sampletime_ms) {
      latest_dust_val = lowpulseoccupancy * sampletime_ms / current_time;
      lowpulseoccupancy = 0;

      starttime = this_time;
      dust_latest = 1;

      // Serial.println(latest_dust_val);
    }
  }
  if (index == 4 && flag == 0) {
    flag = 1;
    // Digital Read
    if (cmd[0] == 1)
    {
      b[0] = cmd[0];
      b[1] = digitalRead(cmd[1]);
    }

    // Digital Write
    else if (cmd[0] == 2)
      digitalWrite(cmd[1], cmd[2]);

    // Analog Read
    else if (cmd[0] == 3) {
      aRead = analogRead(cmd[1]);
      b[0] = cmd[0];
      b[1] = aRead / 256;
      b[2] = aRead % 256;
    }

    // Set up Analog Write
    else if (cmd[0] == 4)
      analogWrite(cmd[1], cmd[2]);

    // Set up pinMode
    else if (cmd[0] == 5)
      pinMode(cmd[1], cmd[2]);

    // Ultrasonic Read
    else if (cmd[0] == 7) {
       int trigPin  = cmd[1];
      pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
      int echoPin  = cmd[2];
      pinMode(echoPin , INPUT); // Sets the echoPin as an Input
      // Clears the trigPin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      RangeCm= duration*0.034/2;
      b[0] = cmd[0];
      b[1] = RangeCm / 256;
      b[2] = RangeCm % 256;
    }
    // Firmware version
    else if (cmd[0] == 8) {
      b[0] = cmd[0];
      b[1] = 2;
      b[2] = 0;
      b[3] = 0;
    }
    // Grove temp and humidity sensor pro
    // 40- Temperature
    else if (cmd[0] == 40) {
    }

    // Grove LED Bar
    else if (cmd[0] == 50) {
    }

    // Grove LED Bar, Change the orientation
    else if (cmd[0] == 51) {
    }

    // Grove LED Bar, Set level (0-10)
    else if (cmd[0] == 52) {
    }

    // Grove LED Bar, Set a single led
    else if (cmd[0] == 53) {
    }

    // Grove LED Bar, Toggle a single led
    else if (cmd[0] == 54) {
    }

    // Grove LED Bar, Set the current state, one bit for each led
    else if (cmd[0] == 55) {
    }

    // Grove LED Bar, Return the current state
    else if (cmd[0] == 56 ) {
    }
    
    // Grove 4 Digit Display (7 segment)
    else if (cmd[0] == 70) {
    }

    // Grove 4 Digit Display (7 segment), set brightness
    else if (cmd[0] == 71) {
    }

    // Grove 4 Digit Display (7 segment), show right aligned decimal value without leading zeros
    else if (cmd[0] == 72) {
    }

    // Grove 4 Digit Display (7 segment), show right aligned decimal value with leading zeros
    else if (cmd[0] == 73 ) {
    }

    // Grove 4 Digit Display (7 segment), set individual digit
    else if (cmd[0] == 74) {
    }

    // Grove 4 Digit Display (7 segment), set individual segment
    else if (cmd[0] == 75) {
    }

    // Grove 4 Digit Display (7 segment), set left and right with colon separator
    else if (cmd[0] == 76) {
    }

    // Grove 4 Digit Display (7 segment), analog read
    else if (cmd[0] == 77) {
    }

    // Grove 4 Digit Display (7 segment), display on
    // [78, pin, unused, unused]
    else if (cmd[0] == 78) {
    }

    // Grove 4 Digit Display (7 segment), display off
    else if (cmd[0] == 79) {
    }

    // Grove Chainable RGB LED
    else if (cmd[0] == 90) {
    }

    // Grove Chainable RGB LED, Initialise a RGB LED chain
    // [91, pin, num leds, unused]
    else if (cmd[0] == 91) {
    }

    // Grove Chainable RGB LED, Test colors, repeating red green blue
    else if (cmd[0] == 92) {
    }

    // Grove Chainable RGB LED, Set one or more leds to the stored color using pattern
    else if (cmd[0] == 93) {
    }

    // Grove Chainable RGB LED, Set one or more leds to the stored color using modulo
    else if (cmd[0] == 94) {
    }

    // Grove Chainable RGB LED, Set level (0 to num leds), counting outwards from the GrovePi, 0 = all
    else if (cmd[0] == 95) {

    // Grove Chainable RGB LED, Set level (0 to num leds), counting outwards from the GrovePi, 0 = all
    }else if (cmd[0] == 96) {
      mpu6050ReadValue();      
    } else if (cmd[0] == dust_sensor_en_cmd) {
    } else if (cmd[0] == dust_sensor_dis_cmd) {
    } else if (cmd[0] == dust_sensor_int_cmd) {
    } else if (cmd[0] == dust_sensor_read_int_cmd) {
    } else if (cmd[0] == dust_sensor_read_cmd) {
    } else if (cmd[0] == encoder_en_cmd) {
      encoder.Timer_init();
      enc_run_bk = 1;
      cmd[0] = 0;
    } else if (cmd[0] == encoder_dis_cmd) {
      encoder.Timer_disable();
      enc_run_bk = 0;
    } else if (cmd[0] == flow_en_cmd) {
      flow_sensor_pin = cmd[1];
      pinMode(digitalPinToInterrupt(flow_sensor_pin), INPUT);
      attachInterrupt(digitalPinToInterrupt(flow_sensor_pin), rpm, RISING);
      NbTopsFan = 0;
      flow_read_start = millis();
      flow_run_bk = 1;
      cmd[0] = 0;
    } else if (cmd[0] == flow_dis_cmd) {
      flow_run_bk = 0;
      detachInterrupt(digitalPinToInterrupt(flow_sensor_pin));
      cmd[0] = 0;
    } else if (cmd[0] == ir_recv_pin_cmd) {
    } else if (cmd[0] == ir_read_cmd) {
    } else if (cmd[0] == ir_read_isdata) {
    }
  }
  if (enc_run_bk) {
    if (encoder.rotate_flag == 1) {
      if (encoder.direct == 1) {
        index_LED++;
        if (index_LED > 24)
          index_LED = 0;
        enc_val[0] = cmd[0];
        enc_val[1] = 1;
        enc_val[2] = index_LED;
      } else {
        index_LED--;
        if (index_LED < 0)
          index_LED = 24;
        enc_val[0] = cmd[0];
        enc_val[1] = 1;
        enc_val[2] = index_LED;
      }
      encoder.rotate_flag = 0;
    }
  }

  if (flow_run_bk) {
    if (millis() - flow_read_start > 2000) {
      Calc = (NbTopsFan * 30 / 73);
      flow_val[0] = 1;
      flow_val[1] = Calc % 256;
      flow_val[2] = Calc / 256;
      NbTopsFan = 0;
      flow_read_start = millis();
    }
  }
}

void loop() {
  if (need_extra_loop == true) {
    processIO();
    need_extra_loop = false;
  } else {
    processIO();
  }
}

void receiveData(int byteCount) {
  if (!need_extra_loop) {
    while (Wire.available()) {
      if (Wire.available() == 4) {
        flag = 0;
        index = 0;
        run_once = 1;
      }
      cmd[index++] = Wire.read();
    }
    need_extra_loop = true;
  } else {
    flushI2C();
  }
}

void flushI2C() {
  while (Wire.available())
    Wire.read();
}

// callback for sending data
void sendData() {
    if (need_extra_loop == false) {
    if (cmd[0] == 1)
      Wire.write((byte *)b, 2);
    if (cmd[0] == 3 || cmd[0] == 7 || cmd[0] == 56)
      Wire.write((byte *)b, 3);
    if (cmd[0] == 8 || cmd[0] == 20)
      Wire.write((byte *)b, 4);
    if (cmd[0] == 30)
      Wire.write((byte *)b, 9);
    if (cmd[0] == 40)
      Wire.write((byte *)dht_b, 9);

    if (cmd[0] == 96) { // send MPU data: 93 foat value
      Wire.write((byte *)b, 5);
      b[0] = 0;
    }
    if (cmd[0] == ir_read_cmd) {
      Wire.write((byte *)b, 8);
      b[0] = 0;
    }
    if (cmd[0] == ir_read_isdata) {
      Wire.write((byte *)b, 2);
    }
    if (cmd[0] == dust_sensor_read_cmd) {
      Wire.write((byte *)b, 5);
      dust_latest = 0;
      cmd[0] = 0;
    }
    if (cmd[0] == dust_sensor_read_int_cmd) {
      Wire.write((byte *)b, 3);
      cmd[0] = 0;
    }
    if (cmd[0] == encoder_read_cmd) {
      Wire.write((byte *)enc_val, 3);
      enc_val[0] = enc_val[1] = 0;
      cmd[0] = 0;
    }
    if (cmd[0] == flow_read_cmd) {
      Wire.write((byte *)flow_val, 3);
      flow_val[0] = 0;
      cmd[0] = 0;
    }
  }
  // otherwise just reply the Pi telling
  // there's no data available yet
  else {
    Wire.write(data_not_available);
  }
}

// ISR for the flow sensor
void rpm() // This is the function that the interupt calls
{
  NbTopsFan++; // This function measures the rising and falling edge of the

  // hall effect sensors signal
}

void readPulseDust() {
  // PORTD |= 0x10;

  t = micros();
  l_status = digitalRead(dust_sensor_pin); // Represents if the line is low or high.
  if (l_status)
    // If the line is high (1), the pulse just ended
    pulse_end = t;
  else
    // If the line is low (0), the pulse just started
    pulse_start = t;

  if (pulse_end > pulse_start) {
    duration = (pulse_end - pulse_start) / 1000;
    lowpulseoccupancy =
        lowpulseoccupancy + duration; // Add to the pulse length.
    pulse_end = 0; // If you don't reset this, you'll keep adding the pulse
                   // length over and over.
  }

  // PORTD &= ~0x10;
}
