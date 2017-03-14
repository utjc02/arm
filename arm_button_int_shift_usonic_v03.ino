#define PIN_LED 8

// MOTORs

#define STEP_MS 25

#define MOTOR_G 0
#define MOTOR_H 1
#define MOTOR_R 2
#define MOTOR_V 3
#define MOTOR_ALL 255

#define DIR_LOOSE 0
#define DIR_TIGHT 1

#define DIR_BACK 0
#define DIR_FRWD 1

#define DIR_UP 0
#define DIR_DN 1

#define DIR_CW 0
#define DIR_CCW 1

#define STOP_REASON_BLOCK 0
#define STOP_REASON_SENS_IR 1
#define STOP_REASON_TIME 2
#define STOP_REASON_GYRO 3
#define STOP_REASON_BUTTON 4

// ULTRASONIC SENSOR

#define PIN_TRIG 7
#define PIN_ECHO 8

// SHIFT REGISTER

#define PIN_SER 13
#define PIN_LATCH 12
#define PIN_CLK 11

// MOTORs PWM and CURRENT SENSORS

uint8_t pinPWM[4] = {5, 6, 9, 10};
uint8_t pinCS[4] = {0, 1, 2, 3}; 

// MOTOR CURRENT MATRIX
// CurrentSensorMatrix Format Example:
// motorCurrentMax[8] = {MOTOR_G__DIR_LOOSE,  MOTOR_G__DIR_TIGHT, 
//                       MOTOR_H__DIR_BACK,  MOTOR_H__DIR_FRWD,
//                       MOTOR_R__DIR_CW, MOTOR_R__DIR__CCW};

uint8_t motorCurrentMax[8] =    {48, 48,    45, 45,   70, 70,   50, 50};

// MOTOR SPEED MATRIX
// SpeedMatrix Format Example:
// motorSpeedStart[8] = {MOTOR_G__DIR_LOOSE, MOTOR_G__DIR_TIGHT,  
//                       MOTOR_H__DIR_BACK,  MOTOR_H__DIR_FRWD,
//                       MOTOR_R__DIR_CW, MOTOR_R__DIR__CCW,
//                       MOTOR_V__DIR_UP,  MOTOR_V__DIR_DN,};

uint8_t motorSpeedStart[8] = { 80,  80,    80,  80,       50,   50,    20, 20};
uint8_t motorSpeedMax[8] =   {120, 120,    110,  110,     80,   80,    40, 40};
uint8_t motorSpeedMin[8] =   {110,  70,    30,  30,       60,   60,    30, 30};
uint8_t motorSpeedInc[4] = {1, 1, 1, 1};
uint8_t motorSpeedDelay[4] = {250, 250, 250, 250};

uint8_t motorDir[4] = {DIR_LOOSE, DIR_BACK, DIR_UP, DIR_CW};


volatile bool flagIntButton = true; 

void intButton() 
{
  Serial.println();
  Serial.println(F(" INT: BUTTON "));
  digitalWrite(PIN_LED, LOW);
  motorOff(MOTOR_ALL, STOP_REASON_BUTTON); 
}

void setup() 
{
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SER, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
 
  attachInterrupt(1, intButton, RISING);
  digitalWrite(PIN_LED, HIGH);
 
 // initMotorH();
  initMotorR();
 // testMotorAll();
}


void loop()
{

}


void motorOff(uint8_t motor, uint8_t reason) 
{
 
 digitalWrite(PIN_LATCH, LOW);
 shiftOut(PIN_SER, PIN_CLK, LSBFIRST, B00000000);
 digitalWrite(PIN_LATCH, HIGH);
 analogWrite(pinPWM[MOTOR_G], 0);
 analogWrite(pinPWM[MOTOR_H], 0);
 analogWrite(pinPWM[MOTOR_R], 0);

 Serial.print(F("MOTOR: "));
 if (motor == MOTOR_ALL)
   Serial.print(F("ALL "));
 else
   Serial.print(motor);
   
 switch (reason) 
 {
  case STOP_REASON_BLOCK:
    Serial.println(F(" STOP: BLOCK"));
    break;
  case STOP_REASON_SENS_IR:
    Serial.println(F(" STOP: IR"));
    break;  
  case STOP_REASON_TIME:
    Serial.println(F(" STOP: TIME"));
    break;  
  case STOP_REASON_GYRO:
    Serial.println(F(" STOP: GYRO"));
    break;
  case STOP_REASON_BUTTON:
    Serial.println(F(" STOP: BUTTON"));
    break;
  default:
    Serial.println(F(" STOP: NA"));  
 }
 
}

void setMotorDirection(uint8_t motor, uint8_t motorDir)
{
   uint8_t shiftDir;
   digitalWrite(PIN_LATCH, LOW);
   shiftOut(PIN_SER, PIN_CLK, MSBFIRST, 0);
   digitalWrite(PIN_LATCH, HIGH); 
   
   shiftDir = (1 << ((2 * motor) + motorDir));
   digitalWrite(PIN_LATCH, LOW);
   shiftOut(PIN_SER, PIN_CLK, MSBFIRST, shiftDir);
   digitalWrite(PIN_LATCH, HIGH);  

//   Serial.print(F("  MOTOR: "));
//   Serial.print(motor);
//   Serial.print(F(" SET SHIFT DIR: "));
//   Serial.println(shiftDir);
}


uint8_t motorRun(uint8_t motor, uint8_t curMotorDir, uint8_t curMotorSpeed)
{        
    int motorCurrentSensorValue;   
    Serial.print(F("  MOTOR: "));
    Serial.print(motor);
    if (curMotorDir == DIR_LOOSE)
    {
      Serial.print(F("  DIR: L/B/CW "));
    }
    else
    {
      Serial.print(F("  DIR: T/F/CCW"));
    }
    Serial.print(F("  SPEED: "));
    Serial.print(curMotorSpeed);

    motorCurrentSensorValue = analogRead(pinCS[motor]);
    Serial.print("  CURRENT: ");
    Serial.println(motorCurrentSensorValue);
     
    setMotorDirection(motor, curMotorDir);
    analogWrite(pinPWM[motor], curMotorSpeed);
    delay(STEP_MS);
    return motorCurrentSensorValue;
}

void testMotorAll ()
{

  motorRun(MOTOR_H, DIR_FRWD,  70);
  delay(1000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(100);

  motorRun(MOTOR_G, DIR_LOOSE,  50);
  delay(2000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(100);
  motorRun(MOTOR_G, DIR_TIGHT,  50);
  delay(2000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

  motorRun(MOTOR_R, DIR_CW,  100);
  delay(750);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);
  motorRun(MOTOR_R, DIR_CCW,  70);
  delay(750);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

  motorRun(MOTOR_H, DIR_BACK,  50);
  delay(900);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

}


void initMotorR()
{
  Serial.println(F("INIT MOTOR_R"));
  
  motorRunTimeUSonic(MOTOR_R, DIR_CW, 1500); 
 delay(500);   
 motorRunTimeUSonic(MOTOR_R, DIR_CCW, 1500); 

  Serial.print("DONE INIT MOTOR_R");
  delay(1000);   
}

void initMotorH()
{
   
  Serial.println(F("INIT MOTOR_H"));
  
  motorRunTime(MOTOR_H, DIR_FRWD, 100); 
  delay(2000);   

  motorRunTime(MOTOR_H, DIR_BACK, 100); 
  
  Serial.print("DONE INIT MOTOR_H");
  delay(1000);   
}

uint8_t motorRunTime(uint8_t motor, uint8_t motorDir, int runTime)
{
  int i;
  int motorSpeed = motorSpeedStart[2*motor];
  int motorCurrentSensorValue;
  boolean flagStart = true;
  
  for(i=0; i<runTime; i+=STEP_MS)
  {
   
    motorCurrentSensorValue = motorRun(motor, motorDir, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }  
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_TIME); 
  return STOP_REASON_TIME;
}

long usonicDistanceCm()
{
  long duration;
  long distanceCm;
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  pinMode(PIN_ECHO, INPUT);
  duration = pulseIn(PIN_ECHO, HIGH);
  distanceCm = (duration/2) / 29.1;
  return distanceCm;
}

uint8_t motorRunUSonic(uint8_t motor, uint8_t curMotorDir, uint8_t curMotorSpeed)
{        
    int motorCurrentSensorValue; 
    long usonicDistanceValue;  
    Serial.print(F("  MOTOR: "));
    Serial.print(motor);
    if (curMotorDir == DIR_LOOSE)
    {
      Serial.print(F("  DIR: L/B/CW "));
    }
    else
    {
      Serial.print(F("  DIR: T/F/CCW"));
    }
    Serial.print(F("  SPEED: "));
    Serial.print(curMotorSpeed);

    motorCurrentSensorValue = analogRead(pinCS[motor]);
    Serial.print("  CURRENT: ");
    Serial.print(motorCurrentSensorValue);
    
    usonicDistanceValue = usonicDistanceCm();
    
    Serial.print("  USONIC: ");
    Serial.println(usonicDistanceValue);
     
    setMotorDirection(motor, curMotorDir);
    analogWrite(pinPWM[motor], curMotorSpeed);
    delay(STEP_MS);
    return motorCurrentSensorValue;
}

uint8_t motorRunTimeUSonic(uint8_t motor, uint8_t motorDir, int runTime)
{
  int i;
  int motorSpeed = motorSpeedStart[2*motor];
  int motorCurrentSensorValue;
  boolean flagStart = true;
  
  for(i=0; i<runTime; i+=STEP_MS)
  {
   
    motorCurrentSensorValue = motorRunUSonic(motor, motorDir, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }  
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_TIME); 
  return STOP_REASON_TIME;
}


