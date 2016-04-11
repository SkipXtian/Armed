
float MAINARM_LENGTH=187; //Shoulder to elbow length
float FOREARM_LENGTH=203; //Elbow to wrist length
float GRIPPER_OFFSET=0;  //Length from wrist to hand PLUS base centre to shoulder (BACK BOLT FOR TESTING)


int accelDuring = 200;              // Amount of stepps to Accelerate, deccelerate for
int downCounter = accelDuring;                       // Count step to Deccelerate
int upCounter = accelDuring;                         // Count step to Accelerate
int accelDelta;                                      // The Delta to add each step during acceleration, deccelerate.

float _x, _y, _z, _r, _t;
const float radDegrees = 57.29578;

//Set the target POS of the arm and enable the interrupt 
void goDirectlyTo(float x, float y, float z) 
{
  float degreeBase,degreeShoulder,degreeElbow;
  if (solve(x, y, z, degreeBase, degreeShoulder, degreeElbow)) {
    
    rotationalTarget = int(degreeBase/0.015);
    mainArmTarget = int(degreeShoulder/0.015);
    foreArmTarget = int(degreeElbow/0.015);
    
    //Set foreArm direction and calculate steps
    boolean targetDir = (foreArmTarget > foreArmPos)? LOW:HIGH;    
    digitalWrite(FOREARM_DIR_PIN,targetDir);
    
    foreArmSteps = foreArmTarget - foreArmPos;
    foreArmSteps = abs(foreArmSteps);
    
    Serial.print("fArm Dir = "); Serial.println(targetDir);
    Serial.print("fArm Steps = "); Serial.println(foreArmSteps);

    //Set foreArm direction and calculate steps
    targetDir = (mainArmTarget > mainArmPos)? HIGH:LOW;
    digitalWrite(MAINARM_DIR_PIN,targetDir);
    
    mainArmSteps = mainArmTarget - mainArmPos;
    mainArmSteps = abs(mainArmSteps);
    
    Serial.print("mArm Dir = "); Serial.println(targetDir);
    Serial.print("mArm Steps = "); Serial.println(mainArmSteps);

    if(foreArmSteps != 0 || mainArmSteps != 0){
      running = true;
      ENABLE_STEPPER_DRIVER_INTERRUPT();
    }
    
    _x = x; _y = y; _z = z;
  } else {
    Serial.println("Not within Range");
  }
}

// Get polar coords from cartesian ones
void cart2polar(float a, float b, float& r, float& theta)
{
    // Determine magnitude of cartesian coords
    r = sqrt(a*a + b*b);

    // Don't try to calculate zero-magnitude vectors' angles
    if(r == 0) return;

    float c = a / r;
    float s = b / r;

    // Safety!
    if(s > 1) s = 1;
    if(c > 1) c = 1;
    if(s < -1) s = -1;
    if(c < -1) c = -1;

    // Calculate angle in 0..PI
    theta = acos(c);

    // Convert to full range
    if(s < 0) theta *= -1;
}

// Get angle from a triangle using cosine rule
bool cosangle(float opp, float adj1, float adj2, float& theta)
{
    // Cosine rule:
    // C^2 = A^2 + B^2 - 2*A*B*cos(angle_AB)
    // cos(angle_AB) = (A^2 + B^2 - C^2)/(2*A*B)
    // C is opposite
    // A, B are adjacent
    float den = 2*adj1*adj2;

    if(den==0) return false;
    float c = (adj1*adj1 + adj2*adj2 - opp*opp)/den;

    if(c>1 || c<-1) return false;

    theta = acos(c);

    return true;
}

// Solve angles!
bool solve(float x, float y, float z, float& a0, float& a1, float& a2)
{
    // Solve top-down view
    float r, th0;
    cart2polar(y, x, r, th0);

    // Account for the wrist length!
    r -= GRIPPER_OFFSET;

    // In arm plane, convert to polar
    float ang_P, R;
    cart2polar(r, z, R, ang_P);

    // Solve arm inner angles as required
    float B, C;
    if(!cosangle(FOREARM_LENGTH, MAINARM_LENGTH, R, B)) return false;
    if(!cosangle(R, MAINARM_LENGTH, FOREARM_LENGTH, C)) return false;

    // Solve for angles from horizontal
    a0 = th0;
    a1 = ang_P + B;
    a2 = C + a1 - PI;

    // Solve for angles from vertical in degrees
    a0 *= radDegrees;
    a1 = 90-a1*radDegrees;
    a2 = -90-a2*radDegrees;

    return true;
}


//Check to see if possible
bool isReachable(float x, float y, float z) 
{
  float radBase,radShoulder,radElbow;
  return (solve(x, y, z, radBase, radShoulder, radElbow));
}

