//testMode Branch
#include <SPI.h>
//#include "as5048a.h"
#include "math.h"
#include "pins.h"

SPISettings settings;
#define _cs 53
word foreArmPosition, mainArmPosition;

/*
mainArm       Max Forward 3400;
foreArm       Max Forward 950   Min Back -4425
Width between Max 7150          Min 650

*/




//bool errorFlag;
const int AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
const int AS5048A_PROGRAMMING_CONTROL           = 0x0003;
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
const int AS5048A_DIAG_AGC                      = 0x3FFD;
const int AS5048A_MAGNITUDE                     = 0x3FFE;
const int AS5048A_ANGLE                         = 0x3FFF;
//FOREARM_DIR HIGH = Back, LOW = Forward
//MAINARM_DIR HIGH = Forward, LOW = Back
/* 6000 = 90 degrees, 1 step = 0.015 degrees
   Vertical for ForeArm = 5400 steps from back limit switch (home position is -1075)
   gotoHome() Moves ForeArm to back limit switch then forward 4325 steps, then moves mainArm back to meet ForeArm. MainArm is vertical.
*/

//AS5048A angleSensor(53);

#define BACKLIGHT_PIN     13

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

int foreArmSteps, mainArmSteps, rotationalSteps;
float foreArmTarget, mainArmTarget, rotationalTarget;
float foreArmPos, mainArmPos, rotationalPos;

int halfStepDurationMS = 1200;                          // Calculated value to set 1/2 step pulse timer (mS x 2)

long stepsLeft = 0;                                  // Number of Steps
boolean running = false;                             // Arm moving?

long timeNow = 0;
long cycleTime = 2;
//int xPos, yPos, zPos;

void setup()
{
  initSteppers();
  //initEndstops();
  

  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial.println("#project Armed");
  Serial.println("Enter m=MainArm,f=ForeArm,s=Step speed, e=enable/disable");

  AS5048Ainit();
  setTimer1(halfStepDurationMS);

  

  //gotoHome();
  //test90();
  //foreArmPos = -1075;
  //mainArmPos = 0;
  for(byte i = 0; i < 10; i++){  //Fill the angleAverage filter
    getRotation();
    delay(200);
  }
  printAngle();
}

//g1,0,200,170 //Main arm straight
//long timeLast = 0;
//m2000
//f4000
//g1,0,270,160 UP & extended
//g1,0,100,-100
void loop ()
{
  static boolean lastRunState = false;
  if (running != lastRunState) {
    Serial.println("Move Complete");
    printAngle();
    if (lastRunState == true) {
      steppersEnabled(false);
    }
    lastRunState = running;
  }
  //drawSquare();
  if (Serial.available()) {
    char cmd = Serial.read();
    int amount = Serial.parseInt();

    Serial.print(amount);
    Serial.println(cmd);
    if (cmd == 'p') {
      
      printAngle();
      
    } else if (cmd == 'a') {
      
      
      moveLimb(ROTATION_ENABLE_PIN, ROTATION_STEP_PIN, ROTATION_DIR_PIN, 1000);
      moveLimb(MAINARM_ENABLE_PIN, MAINARM_STEP_PIN, MAINARM_DIR_PIN, 1000);
      moveLimb(FOREARM_ENABLE_PIN, FOREARM_STEP_PIN, FOREARM_DIR_PIN, 1000);
      moveLimb(ROTATION_ENABLE_PIN, ROTATION_STEP_PIN, ROTATION_DIR_PIN, -2000);
      moveLimb(MAINARM_ENABLE_PIN, MAINARM_STEP_PIN, MAINARM_DIR_PIN, -2000);
      moveLimb(FOREARM_ENABLE_PIN, FOREARM_STEP_PIN, FOREARM_DIR_PIN, -2000);
      moveLimb(ROTATION_ENABLE_PIN, ROTATION_STEP_PIN, ROTATION_DIR_PIN, 1000);
      moveLimb(MAINARM_ENABLE_PIN, MAINARM_STEP_PIN, MAINARM_DIR_PIN, 1000);
      moveLimb(FOREARM_ENABLE_PIN, FOREARM_STEP_PIN, FOREARM_DIR_PIN, 1000);
    } else if (cmd == 'b') {
      
      digitalWrite(ROTATION_ENABLE_PIN    , LOW);
      boolean targetDir = (amount >= 0) ? LOW : HIGH;
      digitalWrite(ROTATION_DIR_PIN, targetDir);
      amount = abs(amount);
      for(int i = 0; i < amount; i++){
         digitalWrite(ROTATION_STEP_PIN, HIGH);
         delay(1);      
         digitalWrite(ROTATION_STEP_PIN, LOW);
         delay(1);
      }
      digitalWrite(ROTATION_ENABLE_PIN    , HIGH);
      printAngle();
      
    } else if (cmd == 'm') {
      
      digitalWrite(MAINARM_ENABLE_PIN    , LOW);
      boolean targetDir = (amount >= 0) ? LOW : HIGH;
      digitalWrite(MAINARM_DIR_PIN, targetDir);
      amount = abs(amount);
      for(int i = 0; i < amount; i++){
         digitalWrite(MAINARM_STEP_PIN, HIGH);
         delay(1);      
         digitalWrite(MAINARM_STEP_PIN, LOW);
         delay(1);
      }
      digitalWrite(MAINARM_ENABLE_PIN    , HIGH);
      printAngle();
      
    } else if (cmd == 'f') {
      
      digitalWrite(FOREARM_ENABLE_PIN    , LOW);
      boolean targetDir = (amount >= 0) ? HIGH : LOW;
      digitalWrite(FOREARM_DIR_PIN, targetDir);
      amount = abs(amount);
      for(int i = 0; i < amount; i++){
         digitalWrite(FOREARM_STEP_PIN, HIGH);
         delay(1);      
         digitalWrite(FOREARM_STEP_PIN, LOW);
         delay(1);
      }
      digitalWrite(FOREARM_ENABLE_PIN    , HIGH);
      printAngle();
      
    } else if (cmd == 's') {
      
      halfStepDurationMS = amount;
      setTimer1(halfStepDurationMS);
      Serial.print("halfStepDurationMS set to "); Serial.println(halfStepDurationMS);
      
    } else if (cmd == 'e') {
      
      boolean state = (amount <= 0) ? LOW : HIGH;
      steppersEnabled(state);
      
    } else if (cmd == 'g') {

    
      int getForeArmTarget = Serial.parseInt();
      int getMainArmTarget = Serial.parseInt();
      int getRotationalTarget = Serial.parseInt();
      printAngle();
      Serial.print("Converting from Cartesian : ");
      Serial.print(getForeArmTarget);  Serial.print(" : ");
      Serial.print(getMainArmTarget);  Serial.print(" : ");
      Serial.println(getRotationalTarget);
      Serial.println("Moving Steppers To...");
    
      goDirectlyTo(getForeArmTarget,getMainArmTarget,getRotationalTarget);
    }
    
  }
  
  
  if (millis() - timeNow > cycleTime) {
    //static byte cnt = 0;
    getRotation();
    //cnt++;
    /*word foreArmAngle, mainArmAngle;
    getArmAngles(&foreArmAngle, &mainArmAngle);*/
    //if(cnt == 100){
      
    //  cnt = 0;
    //}
    //printAvgAngle();
    timeNow = millis();
  }
  /*static bool travel = 1;
    if(!running) {
    if(travel) {
      driveStepper(1000);
      travel = !travel;
    }
    else {
      driveStepper(-1000);
      travel = !travel;
    }
    } */
    static long demoCycle = 60000*5;
static long demoTime = -demoCycle;
  if (millis() - demoTime > demoCycle) {
    //static byte cnt = 0;
    //getRotation();
    //cnt++;
    /*word foreArmAngle, mainArmAngle;
    getArmAngles(&foreArmAngle, &mainArmAngle);*/
    //if(cnt == 100){
      
    //  cnt = 0;
    //}
    //printAvgAngle();
    demoTime = millis();
    //demoMode();
  }
  
}

void demoMode(){
  moveLimb(ROTATION_ENABLE_PIN, ROTATION_STEP_PIN, ROTATION_DIR_PIN, 1000);
  moveLimb(MAINARM_ENABLE_PIN, MAINARM_STEP_PIN, MAINARM_DIR_PIN, 1000);
  moveLimb(FOREARM_ENABLE_PIN, FOREARM_STEP_PIN, FOREARM_DIR_PIN, 1000);
  moveLimb(ROTATION_ENABLE_PIN, ROTATION_STEP_PIN, ROTATION_DIR_PIN, -2000);
  moveLimb(MAINARM_ENABLE_PIN, MAINARM_STEP_PIN, MAINARM_DIR_PIN, -2000);
  moveLimb(FOREARM_ENABLE_PIN, FOREARM_STEP_PIN, FOREARM_DIR_PIN, -2000);
  moveLimb(ROTATION_ENABLE_PIN, ROTATION_STEP_PIN, ROTATION_DIR_PIN, 1000);
  moveLimb(MAINARM_ENABLE_PIN, MAINARM_STEP_PIN, MAINARM_DIR_PIN, 1000);
  moveLimb(FOREARM_ENABLE_PIN, FOREARM_STEP_PIN, FOREARM_DIR_PIN, 1000);
}

void moveLimb(byte enablePin, byte stepPin, byte dirPin, int amount){
  digitalWrite(enablePin    , LOW);
  boolean targetDir = (amount >= 0) ? LOW : HIGH;
  digitalWrite(dirPin, targetDir);
  amount = abs(amount);
  for(int i = 0; i < amount; i++){
     digitalWrite(stepPin, HIGH);
     delay(1);      
     digitalWrite(stepPin, LOW);
     delay(1);
  }
  digitalWrite(enablePin    , HIGH);
}

void printAngle(){
  Serial.print(foreArmPos);
  Serial.print(" : ");
  Serial.println(mainArmPos);
}

void test90()
{
  int dTime = 450;
  digitalWrite(MAINARM_DIR_PIN    , HIGH);
  for (unsigned int i = 6000; i != 0; i--) {
    digitalWrite(MAINARM_STEP_PIN    , HIGH);
    delayMicroseconds(dTime);
    digitalWrite(MAINARM_STEP_PIN    , LOW);
    delayMicroseconds(dTime);
  }
}

void gotoHome()
{
  int dTime = 450;
  digitalWrite(FOREARM_DIR_PIN    , HIGH);
  while (digitalRead(FOREARM_MIN_PIN)) { //Travel all the way back till endStop
    digitalWrite(FOREARM_STEP_PIN    , HIGH);
    delayMicroseconds(dTime);
    digitalWrite(FOREARM_STEP_PIN    , LOW);
    delayMicroseconds(dTime);
  }

  foreArmSteps = 0;
  mainArmSteps = 0;

  delay(1000);

  digitalWrite(FOREARM_DIR_PIN    , LOW);
  while (foreArmSteps <= 4325) {                         //Travel all the way up till endStop
    digitalWrite(FOREARM_STEP_PIN    , HIGH);
    delayMicroseconds(dTime);
    digitalWrite(FOREARM_STEP_PIN    , LOW);
    delayMicroseconds(dTime);
    foreArmSteps++;
    if (!digitalRead(FOREARM_MAX_PIN)) {                  // If we hit the main Arm, move it into position

      Serial.println(foreArmSteps);

      unsigned int diff = 4325 - foreArmSteps;
      digitalWrite(MAINARM_DIR_PIN    , HIGH);
      for (unsigned int i = diff; i != 0; i--) {
        digitalWrite(MAINARM_STEP_PIN    , HIGH);
        delayMicroseconds(dTime);
        digitalWrite(MAINARM_STEP_PIN    , LOW);
        delayMicroseconds(dTime);
      }
    }
  }


  digitalWrite(MAINARM_DIR_PIN    , LOW);
  while (digitalRead(FOREARM_MAX_PIN)) {
    digitalWrite(MAINARM_STEP_PIN    , HIGH);
    delayMicroseconds(dTime);
    digitalWrite(MAINARM_STEP_PIN    , LOW);
    delayMicroseconds(dTime);
  }
  delay(1000);
  foreArmPos = -1075;
  mainArmPos = 0;
}


