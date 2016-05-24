//testMode Branch

#include "as5048a.h"
#include "math.h"
#include "pins.h"

//FOREARM_DIR HIGH = Back, LOW = Forward
//MAINARM_DIR HIGH = Forward, LOW = Back
/* 6000 = 90 degrees, 1 step = 0.015 degrees
 * Vertical for ForeArm = 5400 steps from back limit switch (home position is -1075)
 * gotoHome() Moves ForeArm to back limit switch then forward 4325 steps, then moves mainArm back to meet ForeArm. MainArm is vertical.
 */

AS5048A angleSensor(53);

#define BACKLIGHT_PIN     13

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

int foreArmSteps, mainArmSteps, rotationalSteps;
int foreArmTarget, mainArmTarget, rotationalTarget;
int foreArmPos, mainArmPos, rotationalPos;

int halfStepDurationMS = 1200;                          // Calculated value to set 1/2 step pulse timer (mS x 2)   
                                            
long stepsLeft = 0;                                  // Number of Steps
boolean running = false;                             // Arm moving?

long timeNow = 0;
long cycleTime = 1000;
//int xPos, yPos, zPos;

void setup() 
{  
  initSteppers();
  //initEndstops();
  angleSensor.init();

  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial.println("#project Armed");
  Serial.println("Enter m=MainArm,f=ForeArm,s=Step speed, e=enable/disable");


  setTimer1(halfStepDurationMS); 
  
 

  //gotoHome();
  //test90();
  //foreArmPos = -1075;
  //mainArmPos = 0;
  
}

void drawSquare() 
{
  goDirectlyTo(0, 100, 0);
  delay(1000);
  for(byte i = 100; i <= 200; i++) {
    //delay(10);
    while(running == true) {
      delay(1);
    } //wait for move to complete
    goDirectlyTo(0, i, 0);
  }
  for(byte i = 0; i <= 100; i++) {
    while(running == true) {
      delay(1);
    } //wait for move to complete
    goDirectlyTo(0, 200, i);
  }
  for(byte i = 200; i >= 100; i--) {
    while(running == true) {
      delay(1);
    } //wait for move to complete
    goDirectlyTo(0, i, 100);
  }
  for(byte i = 100; i >= 0; i--) {
    while(running == true) {
      delay(1);
    } //wait for move to complete
    goDirectlyTo(0, 100, i);
  }
  
}
//long timeLast = 0;


void loop () 
{
  static boolean lastRunState = false;
  if(running != lastRunState){
    Serial.println("Move Complete");   
    if(lastRunState == true){
    steppersEnabled(false);
    }
    lastRunState = running;
  }
  //drawSquare();
  if(Serial.available()){
    char cmd = Serial.read();
    int amount = Serial.parseInt();
    
    Serial.print(amount);
    Serial.println(cmd);
    if(cmd == 'm'){
        
        //digitalWrite(MAINARM_ENABLE_PIN    , LOW);
      boolean targetDir = (amount >= 0)? LOW:HIGH;    
      digitalWrite(MAINARM_DIR_PIN,targetDir);
      mainArmSteps = amount;
      mainArmSteps = abs(mainArmSteps);
      Serial.print("Moving steps: "); Serial.print(mainArmSteps);
      running = true;
      steppersEnabled(true);
      ENABLE_STEPPER_DRIVER_INTERRUPT();
      //while(running==true){}     
      Serial.print("DONE with halfStepDurationMS set to "); Serial.print(halfStepDurationMS);
      //digitalWrite(MAINARM_ENABLE_PIN    , HIGH);
    } else if(cmd == 'f'){
      //digitalWrite(FOREARM_ENABLE_PIN    , LOW);
      boolean targetDir = (amount >= 0)? LOW:HIGH;    
      digitalWrite(FOREARM_DIR_PIN,targetDir);
      foreArmSteps = amount;
      foreArmSteps = abs(foreArmSteps);
      Serial.print("Moving steps: "); Serial.print(foreArmSteps);
      running = true;
      steppersEnabled(true);
      ENABLE_STEPPER_DRIVER_INTERRUPT();
      //while(running==true){}     
      Serial.print("DONE with halfStepDurationMS set to "); Serial.print(halfStepDurationMS);
      //digitalWrite(FOREARM_ENABLE_PIN    , HIGH);
    } else if(cmd == 's'){
      halfStepDurationMS = amount;
      setTimer1(halfStepDurationMS);
      Serial.print("halfStepDurationMS set to "); Serial.println(halfStepDurationMS);
    } else if(cmd == 'e'){
      steppersEnabled(false);
    }
    
    /*
    int getForeArmTarget = Serial.parseInt();
    int getMainArmTarget = Serial.parseInt();
    int getRotationalTarget = Serial.parseInt();
    
    Serial.print("Converting from Cartesian : ");
    Serial.print(getForeArmTarget);  Serial.print(" : ");
    Serial.print(getMainArmTarget);  Serial.print(" : ");
    Serial.println(getRotationalTarget);  
    Serial.println("Moving Steppers To...");
    
    goDirectlyTo(getForeArmTarget,getMainArmTarget,getRotationalTarget);   
    */ 
  }
  if(millis() - timeNow > cycleTime){
      angleSensor.getRawRotation();
      
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
  while(digitalRead(FOREARM_MIN_PIN)){   //Travel all the way back till endStop
    digitalWrite(FOREARM_STEP_PIN    , HIGH);
    delayMicroseconds(dTime); 
    digitalWrite(FOREARM_STEP_PIN    , LOW);
    delayMicroseconds(dTime);
  }

  foreArmSteps = 0;
  mainArmSteps = 0;  

  delay(1000);

  digitalWrite(FOREARM_DIR_PIN    , LOW);
  while(foreArmSteps <= 4325) {                          //Travel all the way up till endStop 
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
  while(digitalRead(FOREARM_MAX_PIN)) {  
      digitalWrite(MAINARM_STEP_PIN    , HIGH);
      delayMicroseconds(dTime); 
      digitalWrite(MAINARM_STEP_PIN    , LOW);
      delayMicroseconds(dTime);
  }
  delay(1000);
  foreArmPos = -1075;
  mainArmPos = 0;
}


