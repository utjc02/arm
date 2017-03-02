#define MOTOR_J 1
#define MOTOR_H 2
#define MOTOR_V 3
#define MOTOR_R 4

#define DIR_LOOSE 1
#define DIR_TIGHT 2

#define DIR_BACK 1
#define DIR_FRWD 2

#define STOP_REASON_BLOCK 0
#define STOP_REASON_SENS_IR 1
#define STOP_REASON_TIME 2
#define STOP_REASON_GYRO 3

#define IR_OBJECT 0
#define IR_EMPTY 1
#define pinIR 3

#define GYRO_MAX_UP 52
#define GYRO_MAX_DW 10

#define PRECISON 0.20
#define CORRELATION 3

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL


#define LED_PIN 13 
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// Motors
int pinCW[2] = {7, 4}; 
int pinCCW[2] = {8, 9}; 
int pinPWM[2] = {5, 6};
int pinCS[2] = {2, 3};


// CurrentSensorMatrix Format Example:
// motorCurrentMax[4] = {MOTOR_J__DIR_LOOSE,  MOTOR_J__DIR_TIGHT, MOTOR_H__DIR_BACK,  MOTOR_H__DIR_FRWD};

int motorCurrentMax[4] = {50, 50, 31, 61};

// SpeedMatrix Format Example:
// motorSpeedStart[4] = {MOTOR_J__DIR_LOOSE, MOTOR_J__DIR_TIGHT,  MOTOR_H__DIR_BACK,  MOTOR_H__DIR_FRWD};

int motorSpeedStart[4] = { 80,  80, 70,  60};
int motorSpeedMax[4] =   {120, 120, 105, 70};
int motorSpeedMin[4] =   {110,  70, 45,  20};

int motorSpeedInc[2] = {1, 1};
int motorSpeedDelay[2] = {250, 250};

uint8_t motorDir[2] = {DIR_LOOSE, DIR_BACK};
float startMPUGyroZ = 0; 
float endMPUGyroZ = 0; 
float curMPUGyroZ = 0;
float absTreshold; 
float startTrMPUGyroZ;
float endTrMPUGyroZ;
float mpuGyroZ = 0; 

boolean startPosition = false;
boolean endPosition = false;

volatile bool mpuInterrupt = false; 

boolean getMPUGyroZ ()
{    
      boolean flagReturn = false;
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
      
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
      
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
       
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else 
      if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) 
             fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpuGyroZ = ypr[2] * 180/M_PI;
        flagReturn = true;
      }
     return flagReturn;
}


void dmpDataReady() {
      mpuInterrupt = true; 
}


void setup() 
{
    Wire.begin();
    TWBR = 24;

    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
/*  
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    delay(500);    
  ////////////
  int i;  

  int motorSpeed;
  int sensorIRValue;
  uint8_t motorCurrentSensorValue;
  boolean flagStart = true;
  boolean flagStartPosition = true;
  
  // PINs setup
  pinMode(pinIR, INPUT);

  for (i=0; i<2; i++)
  {
    pinMode(pinCW[i], OUTPUT);
    pinMode(pinCCW[i], OUTPUT);
    pinMode(pinPWM[i], OUTPUT);
    
    digitalWrite(pinCW[i], LOW);
    digitalWrite(pinCCW[i], LOW);    
  }

  initMotor0();
  initMotor1();
  
  sensorIRValue = digitalRead(pinIR);
  motorSpeed = motorSpeedStart[2*MOTOR_H+1];


  // **** MOTOR_H FRWD ****
  Serial.println("START MOTOR FORWARD");
  while (sensorIRValue == IR_EMPTY)
  {      
    sensorIRValue = digitalRead(pinIR);
    Serial.print(F("  IR = "));
    Serial.print(sensorIRValue);   
    
    motorCurrentSensorValue = analogRead(pinCS[MOTOR_H]);
    
    if (mpuInterrupt && flagStartPosition)
    {
      while(!getMPUGyroZ());
      startMPUGyroZ = mpuGyroZ;
      if (startMPUGyroZ != 0)
        {
          flagStartPosition = false;
          Serial.print(F("  StartMpuGyroZ = "));
          Serial.println(startMPUGyroZ);
        }
    }
    
    motorRun(MOTOR_H, DIR_FRWD, motorSpeed);
    
    if (flagStart && (motorSpeed < motorSpeedMax[2*MOTOR_H+1]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*MOTOR_H+1];
      flagStart = false;
    }     
  }
  
   if (mpuInterrupt)
    {
      endMPUGyroZ = 0;
      while(!getMPUGyroZ());
      endMPUGyroZ = mpuGyroZ;
      Serial.print(F("  EndMpuGyroZ = "));
      Serial.println(endMPUGyroZ);
     }
    
  motorOff(MOTOR_H, STOP_REASON_SENS_IR); 
  
  // **** MOTOR_J TIGHT ****
  Serial.println(F("START MOTOR_J TIGHT")); 
  motorCurrentSensorValue = analogRead(pinCS[MOTOR_J]);
  motorSpeed = motorSpeedStart[2*MOTOR_J+1];
  flagStart = true;
  while (motorCurrentSensorValue < motorCurrentMax[2*MOTOR_J+1])
  {      

    motorCurrentSensorValue = motorRun(MOTOR_J, DIR_TIGHT, motorSpeed);
   
    if (flagStart && (motorSpeed < motorSpeedMax[2*MOTOR_J+1]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*MOTOR_J+1];
      flagStart = false;
    }
  }
  
  motorOff(MOTOR_J, STOP_REASON_BLOCK); 

  // CALCULATE Thresholds
  absTreshold = (startMPUGyroZ - endMPUGyroZ) * PRECISON;
  startTrMPUGyroZ = startMPUGyroZ - absTreshold + CORRELATION;
  endTrMPUGyroZ = endMPUGyroZ + absTreshold + CORRELATION;
  Serial.println(F("\nThresholds: "));
  Serial.print(F("  startABS: "));
  Serial.print(startMPUGyroZ);
  Serial.print(F("  startTRE: "));
  Serial.println(startTrMPUGyroZ);
  
  Serial.print(F("  endABS: "));
  Serial.print(endMPUGyroZ);
  Serial.print(F("  endTRE: "));
  Serial.println(endTrMPUGyroZ);

 delay(500);    
  
  // **** MOTOR_H BACK with SUBJECT****
  Serial.println(F("START MOTOR_H BACK with SUBJECT"));
  motorSpeed = motorSpeedStart[2*MOTOR_H];
  flagStart = true;
  while (!startPosition)
  {
    if (mpuInterrupt)
    {
      while(!getMPUGyroZ());
      curMPUGyroZ = mpuGyroZ;
      Serial.print(F("  CurMpuGyroZ = "));
      Serial.print(curMPUGyroZ);
      if (curMPUGyroZ >= GYRO_MAX_UP)
        startPosition = true;
     }
    else
    {
     motorCurrentSensorValue = motorRun(MOTOR_H, DIR_BACK, motorSpeed); 
     if (flagStart && (motorSpeed < motorSpeedMax[2*MOTOR_H]))
       motorSpeed++;
     else
     {
      motorSpeed = motorSpeedMin[2*MOTOR_H];
      flagStart = false;
     }
    } 
  }
    
  motorOff(MOTOR_H, STOP_REASON_GYRO); 

  // **** MOTOR_H FRWD with SUBJECT ****
  Serial.println(F("START MOTOR_H FRWD with SUBJECT"));
  motorSpeed = motorSpeedStart[2*MOTOR_H+1];
  flagStart = true;
  while (!endPosition)
  {
    if (mpuInterrupt)
    {
      while(!getMPUGyroZ());
      curMPUGyroZ = mpuGyroZ;
      Serial.print(F("  GYROZ: "));
      Serial.print(curMPUGyroZ);
      if (curMPUGyroZ <= endTrMPUGyroZ)
        endPosition = true;
     }
    else
    {
     motorCurrentSensorValue = motorRun(MOTOR_H, DIR_FRWD, motorSpeed); 
     if (flagStart && (motorSpeed < motorSpeedMax[2*MOTOR_H+1]))
       motorSpeed++;
     else
     {
      motorSpeed = motorSpeedMin[2*MOTOR_H+1];
      flagStart = false;
     }
    } 
  }
  
  motorOff(MOTOR_H, STOP_REASON_GYRO); 


  // **** MOTOR_J LOOSE ****
  Serial.println(F("START MOTOR_J LOOSE"));
  
  if (motorRunTime(MOTOR_J, DIR_LOOSE, 20) == STOP_REASON_BLOCK)
    motorOff(MOTOR_J, STOP_REASON_BLOCK);
  else  
    motorOff(MOTOR_J, STOP_REASON_TIME);


  // MOTOR_H BACK
  Serial.println(F("START MOTOR_H BACK EMPTY"));
  startPosition = false;
  while (!startPosition)
  {
    if (mpuInterrupt)
    {
      while(!getMPUGyroZ());
      curMPUGyroZ = mpuGyroZ;
      Serial.print(F("  GYROZ: "));
      Serial.print(curMPUGyroZ);
      if (curMPUGyroZ >= GYRO_MAX_UP)
        startPosition = true;
     }
    else
    {
     motorRun(MOTOR_H, DIR_BACK, 70);
    } 
  }
  
  motorOff(MOTOR_H, STOP_REASON_GYRO); 

   Serial.println(F("FINISH"));
  delay(30000);
  

}

void loop()
{
  
}

void initMotor0()
{
  uint8_t motorCurrentSensorValue = 0;
  uint8_t motorSpeed;
  
  Serial.println(F("INIT MOTOR_J")); 
  motorSpeed = motorSpeedStart[2*MOTOR_J];
  
  while (motorCurrentSensorValue < motorCurrentMax[2*MOTOR_J])
  {      
    motorCurrentSensorValue = motorRun(MOTOR_J, DIR_LOOSE, motorSpeed); 
  }
  motorOff(MOTOR_J, STOP_REASON_BLOCK); 

  motorRunTime(MOTOR_J, DIR_TIGHT, 40);
  motorOff(MOTOR_J, STOP_REASON_TIME); 

  Serial.print("DONE INIT MOTOR_J ");
  delay(200);   
}

void initMotor1()
{
  uint8_t motorCurrentSensorValue = 0;
  uint8_t motorSpeed;
  boolean startPosition = false;
   
  Serial.println(F("INIT MOTOR_H"));

  while (!startPosition)
  {
    if (mpuInterrupt)
    {
      while(!getMPUGyroZ());
      curMPUGyroZ = mpuGyroZ;
      Serial.print(F("  GYROZ: "));
      Serial.print(curMPUGyroZ);
      if (curMPUGyroZ >=  GYRO_MAX_UP)
        startPosition = true;
     }
    else
    { 
     motorRun(MOTOR_H, DIR_BACK, motorSpeedStart[2*MOTOR_H]);
    } 
  }
  
  motorOff(MOTOR_H, STOP_REASON_GYRO); 

  Serial.print("DONE INIT MOTOR_H");
  delay(1000);   
}


void motorOff(uint8_t motor, int sensor) 
{

 digitalWrite(pinCW[motor], LOW);
 digitalWrite(pinCCW[motor], LOW);
 analogWrite(pinPWM[motor], 0);

 switch (sensor) 
 {
  case STOP_REASON_BLOCK:
    Serial.println(F("STOP: BLOCK"));
    break;
  case STOP_REASON_SENS_IR:
    Serial.println(F("STOP: IR"));
    break;  
  case STOP_REASON_TIME:
    Serial.println(F("STOP: TIME"));
    break;  
  case STOP_REASON_GYRO:
    Serial.println(F("STOP: GYRO"));
    break;
  default:
    Serial.println(F("STOP: NA"));  
 }

 delay(1000);

}

uint8_t reverseMotorDirection(uint8_t motor, uint8_t motorDir1)
{
  if (motorDir1 == DIR_LOOSE)
  {
      setMotorDirection(motor, DIR_TIGHT);
      return DIR_TIGHT;
  }
  else
  {
      setMotorDirection(motor, DIR_LOOSE);
      return DIR_LOOSE;
  }
}

void setMotorDirection(uint8_t motor, uint8_t motorDir)
{
   if (motorDir == DIR_LOOSE)
      {
        digitalWrite(pinCW[motor], HIGH);
        digitalWrite(pinCCW[motor], LOW);
      }  
    else
      { 
        // DIR_TIGHT
        digitalWrite(pinCW[motor], LOW);
        digitalWrite(pinCCW[motor], HIGH);
      }  
}


uint8_t motorRunTime(uint8_t motor, uint8_t curMotorDir, int runTime)
{
  int i;
  int motorSpeed = motorSpeedStart[2*motor];
  int motorCurrentSensorValue;
  boolean flagStart = true;
  
  for(i=0; i<runTime; i++)
  {
   
    motorCurrentSensorValue = motorRun(motor, curMotorDir, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      return STOP_REASON_BLOCK;
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  return STOP_REASON_TIME;
}


uint8_t motorRun(uint8_t motor, uint8_t curMotorDir, uint8_t curMotorSpeed)
{        
    int motorCurrentSensorValue;   
    Serial.print(F("  MOTOR: "));
    Serial.print(motor);
    if (curMotorDir == DIR_LOOSE)
    {
      Serial.print(F("  DIR: L/B "));
    }
    else
    {
      Serial.print(F("  DIR: T/F"));
    }
    Serial.print(F("  SPEED: "));
    Serial.print(curMotorSpeed);

    motorCurrentSensorValue = analogRead(pinCS[motor]);
    Serial.print("  CURRENT: ");
    Serial.println(motorCurrentSensorValue);
     
    setMotorDirection(motor, curMotorDir);
    analogWrite(pinPWM[motor], curMotorSpeed);
    delay(25);
    return motorCurrentSensorValue;
}
