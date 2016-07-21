/*****************************************************************
 * Stepper Motor Model: 42BYGHW609
 * Spec's: Step^,  V,    I,     R,    L,  Hold Torque
 *          1.8^, 3.4v, 1.7A, 2 Ohms, 3mH, 4kg.cm
 * Driver: DRV8825
 *    12v, 0.8vRef (1.6A), upto 1/32th Micro Stepping
 *    
 * Maximum 1.8^ Stepping Speed: T=L*A*2/V
 * 0.8mS = 3mH * 1.6A * 2 / 12v 
 *     
 * Pulley Reduction 16T - 60T   (3.75-1)
 *  uStep,      16T,   60T  , Max Step Speed
 *      1,     1.8^,  0.48^ ,     0.8mS
 *    1/2,     0.9^,  0.24^ ,     0.4mS
 *    1/4,    0.45^,  0.12^ ,     0.2mS
 *    1/8,   0.225^,  0.06^ ,     0.1mS
 *   1/16,  0.1125^,  0.03^ ,      50uS  
 *   1/32, 0.05625^,  0.015^,      25uS
 *    
 * Ref: 
 * http://www.daycounter.com/Calculators/Stepper-Motor-Calculator.phtml
 */

//Stepper Controller ISR
ISR(TIMER1_COMPA_vect)
{
  static float hyst = 0.25;
  volatile static bool foreArmStepped = 1;
  volatile static bool mainArmStepped = 1;
  if(foreArmTarget <= (foreArmPos - hyst) || foreArmTarget >= (foreArmPos + hyst)){ //foreArmSteps > 0){
    /*if(stepsLeft == downCounter){  
      downCounter--;
      OCR1A += accelDelta;         // Deccerate
    }
    else if(upCounter > 0){         
      upCounter--;
      OCR1A -= accelDelta;         // Accelerate
    }*/
    
    
    if(foreArmStepped) {
      digitalWrite(FOREARM_STEP_PIN, HIGH);  
      foreArmStepped = !foreArmStepped;
    } else {
      digitalWrite(FOREARM_STEP_PIN, LOW);   
      foreArmStepped = !foreArmStepped;
      //foreArmSteps--;
    }   
    boolean targetDir = (foreArmTarget > foreArmPos)? HIGH:LOW;    
    //boolean targetDir = (degreeElbow > foreArmPos)? LOW:HIGH;    
    digitalWrite(FOREARM_DIR_PIN,targetDir);
  } 
  if(mainArmTarget <= (mainArmPos - hyst) || mainArmTarget >= (mainArmPos + hyst)){
    /*if(stepsLeft == downCounter){  
      downCounter--;
      OCR1A += accelDelta;         // Deccerate
    }
    else if(upCounter > 0){         
      upCounter--;
      OCR1A -= accelDelta;         // Accelerate
    }*/
    
    
    if(mainArmStepped) {
      digitalWrite(MAINARM_STEP_PIN, HIGH);  
      mainArmStepped = !mainArmStepped;
    } else {
      digitalWrite(MAINARM_STEP_PIN, LOW);   
      mainArmStepped = !mainArmStepped;
      //mainArmSteps--;
    }   
    boolean targetDir = (mainArmTarget > mainArmPos)? LOW:HIGH;
    //targetDir = (degreeShoulder > mainArmPos)? HIGH:LOW;
    digitalWrite(MAINARM_DIR_PIN,targetDir);
  }
  //if (foreArmTarget == foreArmPos && mainArmTarget == mainArmPos) {    
  if (foreArmTarget >= (foreArmPos - hyst) && foreArmTarget <= (foreArmPos + hyst) && mainArmTarget >= (mainArmPos - hyst) && mainArmTarget <= (mainArmPos + hyst)) {                          
    running = false;
    //foreArmPos = foreArmTarget;
    //mainArmPos = mainArmTarget;
    DISABLE_STEPPER_DRIVER_INTERRUPT();
  }
}

void initEndstops()
{
  digitalWrite(FOREARM_MIN_PIN, HIGH);
  digitalWrite(FOREARM_MAX_PIN, HIGH);
  digitalWrite(MAINARM_MIN_PIN, HIGH);
  digitalWrite(MAINARM_MAX_PIN, HIGH);
  digitalWrite(ROTATION_MIN_PIN, HIGH);
  digitalWrite(ROTATION_MAX_PIN, HIGH);
}

void initSteppers()
{
  pinMode(FOREARM_STEP_PIN  , OUTPUT);
  pinMode(FOREARM_DIR_PIN    , OUTPUT);
  pinMode(FOREARM_ENABLE_PIN    , OUTPUT);
  
  pinMode(MAINARM_STEP_PIN  , OUTPUT);
  pinMode(MAINARM_DIR_PIN    , OUTPUT);
  pinMode(MAINARM_ENABLE_PIN    , OUTPUT);
  
  pinMode(ROTATION_STEP_PIN  , OUTPUT);
  pinMode(ROTATION_DIR_PIN    , OUTPUT);
  pinMode(ROTATION_ENABLE_PIN    , OUTPUT);

  /*
  pinMode(E_STEP_PIN  , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN    , OUTPUT);
  
  pinMode(Q_STEP_PIN  , OUTPUT);
  pinMode(Q_DIR_PIN    , OUTPUT);
  pinMode(Q_ENABLE_PIN    , OUTPUT);
  */
  
  steppersEnabled(true);
  //digitalWrite(E_ENABLE_PIN    , LOW);
  //digitalWrite(Q_ENABLE_PIN    , LOW);
}

void steppersEnabled(boolean state)
{
  digitalWrite(FOREARM_ENABLE_PIN    , !state);
  digitalWrite(MAINARM_ENABLE_PIN    , !state);
  digitalWrite(ROTATION_ENABLE_PIN    , !state);
}

void setTimer1(int timeCompare)
{
  cli();
   // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11);
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0);
  TCCR1A &= ~(3<<COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = timeCompare;
  TCNT1 = 0;
  sei();
}



/*
void driveStepper(long lnSteps){
    int dir = (lnSteps > 0)? HIGH:LOW;
    digitalWrite(FOREARM_DIR_PIN,dir);
  
    lnSteps = abs(lnSteps);
  
    //OCR1A = ocr1a;                // Set compare timer value
    //upCounter = accelDuring;
    //downCounter = accelDuring;
    running = true;
    stepsLeft = lnSteps;
}
*/
