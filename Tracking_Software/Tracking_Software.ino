#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>

/* This driver reads raw data from the BNO055

   History
   =======
   2020/MAR/06  - First release (Stefan Helm)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define DEG_TO_RAD 0.017453292519943295769236907684886
double position_x = 0;
double position_y = 0;
double position_z = 0;
double hyp;
double winkel;
bool check = true;
Adafruit_BNO055 bno = Adafruit_BNO055();
bool sdInitSuccess = false; //card init status
File myFile;
const int pinReed = 2;
int counter = 0;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  attachInterrupt(digitalPinToInterrupt(pinReed),ReedSwitch,RISING);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{



      if (!sdInitSuccess) { //if not already initialized
        Serial.println("Initializing SD Card..");
        if (!SD.begin(10)) { //using pin 10 (SS)
          Serial.println("Initialization failed!\n");
          sdInitSuccess = false; //failure
          return;
        }
        else {
          Serial.println("Intitialization success.");
          Serial.println();
          sdInitSuccess = true;
        }
      }
  
    
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  /* Display the floating point data */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
if(mag != 3){
  Serial.println(mag);
 }
 

  if(mag == 3){
     if(counter == 3){
      position_z = position_z + sin((euler.y()*DEG_TO_RAD));
      hyp =  cos((euler.y()*DEG_TO_RAD));    
      
      if(euler.x()<=90){
      position_x = position_x + cos((euler.x()*DEG_TO_RAD)) * (-1) * hyp;
      position_y = position_y + sin((euler.x()*DEG_TO_RAD)) * hyp;
      }

      else if(euler.x() > 90 && euler.x() <=180){
      winkel = 180 - euler.x();
      position_x = position_x + cos((winkel*DEG_TO_RAD)) * hyp;
      position_y = position_y + sin((winkel*DEG_TO_RAD)) * hyp;
      }

      else if(euler.x() > 180 && euler.x() <=270){
      winkel= 270 - euler.x();
      position_x = position_x + sin((winkel*DEG_TO_RAD)) * hyp;
      position_y = position_y + cos((winkel*DEG_TO_RAD)) * (-1) * hyp;
      }

      else if(euler.x() > 270){
      winkel = 360 - euler.x();
      position_x = position_x + cos((winkel*DEG_TO_RAD)) * (-1) * hyp;
      position_y = position_y + sin((winkel*DEG_TO_RAD)) * (-1) * hyp;
      }

    Serial.print("X: ");
    Serial.print(position_x);
    Serial.print("\t\t");
    Serial.print("Y: ");
    Serial.print(position_y);
    Serial.print("\t\t");
    Serial.print("Z: ");
    Serial.print(position_z);
    Serial.print("\t\t");
    Serial.print("2D Winkel: ");
    Serial.println(euler.x());
    Serial.print("\t\t");
    Serial.print("3D Winkel: ");
    Serial.println(euler.y());

     if (sdInitSuccess) { //proceed only if card is initialized
      if(check){
        if(SD.exists("Messung.csv")){
        SD.remove("Messung.csv");
        }
        myFile = SD.open("Messung.csv", FILE_WRITE);
        myFile.print("x");
          myFile.print(";");
          myFile.print("y");
          myFile.print(";");
          myFile.println("z");
          myFile.close(); //this writes to the card
          check = false;
      }
      
        myFile = SD.open("Messung.csv", FILE_WRITE);
        if (myFile) {
          Serial.println("File opened successfully.");
          Serial.println("Writing to File");
          myFile.print(position_x);
          myFile.print(";");
          myFile.print(position_y);
          myFile.print(";");
          myFile.println(position_z);
          myFile.close(); //this writes to the card
          Serial.println("Done");
          Serial.println();
        }
        else { //else show error
          Serial.println("Error opeing file.\n");
        }
      }
      else {
        Serial.println("SD Card not initialized.");
        Serial.println("Type \"i\" to initialize.\n");
      }
        counter = 0;
     }
  }
 

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void ReedSwitch(){
      counter = counter + 1;
      Serial.println(counter);
    }
