#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include <Process.h>

Process telnet;  // make a new instance of the Process class

const int sendInterval = 200; // minimum time between messages to the server
const int sendIntervalFast = 100;
const int debounceInterval = 15;  // used to smooth out pushbutton readings
long lastTimeSent = 0;       // timestamp of the last server message

const int upPin = 13;
const int leftPin = 8;
const int rightPin = 12;
const int downPin = 11;
const int centerPin = 10;
const int backPin = 9;

int leftState = LOW;
int rightState = LOW;
int upState = LOW;
int downState = LOW;
int centerState = LOW;
int backState = LOW;

boolean entered = 0;
/* Assign a unique base ID for this sensor */
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000


#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;

  lsm.getSensor(&accel, &mag, &gyro, &temp);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(upPin, OUTPUT);
  pinMode(downPin, OUTPUT);
  pinMode(centerPin, OUTPUT);
  pinMode(backPin, OUTPUT);


  leftState = HIGH;
  rightState = HIGH;
  upState = HIGH;
  downState = HIGH;
  centerState = HIGH;
  backState = HIGH;

  digitalWrite(leftPin, leftState);
  digitalWrite(rightPin, rightState);
  digitalWrite(upPin, upState);
  digitalWrite(downPin, downState);
  digitalWrite(centerPin, centerState);
  digitalWrite(backPin, backState);

  leftState = LOW;
  rightState = LOW;
  upState = LOW;
  downState = LOW;
  centerState = LOW;
  backState = LOW;
  digitalWrite(leftPin, leftState);
  digitalWrite(rightPin, rightState);
  digitalWrite(upPin, upState);
  digitalWrite(downPin, downState);
  digitalWrite(centerPin, centerState);
  digitalWrite(backPin, backState);


  Bridge.begin();
  Serial.begin(9600);
  Serial.println("Bridge Begin");

  telnet.print("x");
  //while (!Serial);  // wait for flora/leonardo

  //Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  /* Initialise the sensor */
  if (!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();

  /* We're ready to go! */
  Serial.println("");
  centerState = HIGH;
  digitalWrite(centerPin, centerState);
  backState = HIGH;
  digitalWrite(backPin, backState);


}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);

  // print out accelleration data
  //    Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  //    Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  //    Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");


  // If the process is running, listen for serial input:
  if (telnet.running()) {
    if (Serial.available() > 0) {// if there's any data in the serial in buffer,
      char c = Serial.read();    // read serial in
      telnet.write(c);           // send it to the telnet process
      Serial.write("telnet said: " + c);           // echo it back locally
    }
  }

  // listen for bytes from the telnet process
  if (telnet.available()) {
    // print the characters to the serial monitor
    Serial.print((char)telnet.read());

  }

  long now = millis();


  if (accel.acceleration.x >= -4 && accel.acceleration.x <= 2) {
    if (accel.acceleration.y >= -3 && accel.acceleration.y <= 3) {
      if (entered == 0) {
        if (accel.acceleration.z > 0) {
          if (telnet.running()) {
            Serial.println("running");
          } else {
            Serial.println("connecting");
            telnet.runShellCommandAsynchronously("telnet 128.122.151.74 8080");
            entered = 1;
            centerState = HIGH;
            digitalWrite(centerPin, centerState);
            backState = LOW;
            digitalWrite(backPin, backState);
            leftState = LOW;
            rightState = LOW;
            upState = LOW;
            downState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
          }
        }
      } else if (entered == 1) {
        if (telnet.running()) {
          if (accel.acceleration.z < 0) {
            Serial.println("disconnecting");
            telnet.print("x");
            entered = 0;
            backState = HIGH;
            digitalWrite(backPin, backState);
            centerState = LOW;
            digitalWrite(centerPin, centerState);
            leftState = LOW;
            rightState = LOW;
            upState = LOW;
            downState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
          } else {
            centerState = HIGH;
            digitalWrite(centerPin, centerState);
            backState = LOW;
            digitalWrite(backPin, backState);
            leftState = LOW;
            rightState = LOW;
            upState = LOW;
            downState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
          }
        }
      }
    }
  }

  if (telnet.running() && entered == 1) {
    if (now - lastTimeSent > sendInterval) {
      if (accel.acceleration.x >= -4 && accel.acceleration.x <= 2) {
        if (accel.acceleration.y > 3) {
          if (accel.acceleration.y > 6) {
            //sharp left
            telnet.print("lll");
            Serial.println("lll");
            lastTimeSent = now;
            leftState = HIGH;
            rightState = LOW;
            upState = LOW;
            downState = LOW;
            centerState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
            digitalWrite(centerPin, centerState);
          } else {
            //left
            telnet.print("l");
            Serial.println("l");
            lastTimeSent = now;
            leftState = HIGH;
            rightState = LOW;
            upState = LOW;
            downState = LOW;
            centerState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
            digitalWrite(centerPin, centerState);
          }
        } else if (accel.acceleration.y < -3) {
          if (accel.acceleration.y < -6) {
            //sharp right
            telnet.print("rrr");
            Serial.println("rrr");
            lastTimeSent = now;
            leftState = LOW;
            rightState = HIGH;
            upState = LOW;
            downState = LOW;
            centerState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
            digitalWrite(centerPin, centerState);
          } else {
            //right
            telnet.print("r");
            Serial.println("r");
            lastTimeSent = now;
            leftState = LOW;
            rightState = HIGH;
            upState = LOW;
            downState = LOW;
            centerState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
            digitalWrite(centerPin, centerState);
          }

        }
      }  else if (accel.acceleration.x > 2) {
        if (accel.acceleration.y >= -3 && accel.acceleration.y <= 3) {
          if (accel.acceleration.x > 5) {
            //sharp up
            telnet.print("uuu");
            Serial.println("uuu");
            lastTimeSent = now;
            leftState = LOW;
            rightState = LOW;
            upState = HIGH;
            downState = LOW;
            centerState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
            digitalWrite(centerPin, centerState);
          } else {
            //up
            telnet.print("u");
            Serial.println("u");
            lastTimeSent = now;
            leftState = LOW;
            rightState = LOW;
            upState = HIGH;
            downState = LOW;
            centerState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
            digitalWrite(centerPin, centerState);
          }

        } else if (accel.acceleration.y > 3) {
          telnet.print("ul");
          Serial.println("ul");
          lastTimeSent = now;
          leftState = HIGH;
          rightState = LOW;
          upState = HIGH;
          downState = LOW;
          centerState = LOW;
          digitalWrite(leftPin, leftState);
          digitalWrite(rightPin, rightState);
          digitalWrite(upPin, upState);
          digitalWrite(downPin, downState);
          digitalWrite(centerPin, centerState);
        } else if (accel.acceleration.y < -3) {
          telnet.print("ur");
          Serial.println("ur");
          lastTimeSent = now;
          leftState = LOW;
          rightState = HIGH;
          upState = HIGH;
          downState = LOW;
          centerState = LOW;
          digitalWrite(leftPin, leftState);
          digitalWrite(rightPin, rightState);
          digitalWrite(upPin, upState);
          digitalWrite(downPin, downState);
          digitalWrite(centerPin, centerState);
        }
      } else if (accel.acceleration.x < -4) {
        if (accel.acceleration.y >= -3 && accel.acceleration.y <= 3) {
          if (accel.acceleration.x < -7) {
            //sharp down
            telnet.print("ddd");
            Serial.println("ddd");
            lastTimeSent = now;
            leftState = LOW;
            rightState = LOW;
            upState = LOW;
            downState = HIGH;
            centerState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
            digitalWrite(centerPin, centerState);
          } else {
            //down
            telnet.print("d");
            Serial.println("d");
            lastTimeSent = now;
            leftState = LOW;
            rightState = LOW;
            upState = LOW;
            downState = HIGH;
            centerState = LOW;
            digitalWrite(leftPin, leftState);
            digitalWrite(rightPin, rightState);
            digitalWrite(upPin, upState);
            digitalWrite(downPin, downState);
            digitalWrite(centerPin, centerState);
          }
        } else if (accel.acceleration.y > 3) {
          telnet.print("dl");
          Serial.println("dl");
          lastTimeSent = now;
          leftState = HIGH;
          rightState = LOW;
          upState = LOW;
          downState = HIGH;
          centerState = LOW;
          digitalWrite(leftPin, leftState);
          digitalWrite(rightPin, rightState);
          digitalWrite(upPin, upState);
          digitalWrite(downPin, downState);
          digitalWrite(centerPin, centerState);
        } else if (accel.acceleration.y < -3) {
          telnet.print("dr");
          Serial.println("dr");
          lastTimeSent = now;
          leftState = LOW;
          rightState = HIGH;
          upState = LOW;
          downState = HIGH;
          centerState = LOW;
          digitalWrite(leftPin, leftState);
          digitalWrite(rightPin, rightState);
          digitalWrite(upPin, upState);
          digitalWrite(downPin, downState);
          digitalWrite(centerPin, centerState);
        }
      }
    }
  }
}
