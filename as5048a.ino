
/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048Ainit(){
  setZeroPosition(13750, 8530);
  // 1MHz clock (AMS should be able to accept up to 10MHz)
  settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
  
  //setup pins
  pinMode(_cs, OUTPUT);

  //SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
  SPI.begin();
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void AS5048Aclose(){
  SPI.end();
}

/**
 * Utility function used to calculate even parity of word
 */
byte spiCalcEvenParity(word value){
  byte cnt = 0;
  byte i;

  for (i = 0; i < 16; i++)
  {
    if (value & 0x1)
    {
      cnt++;
    }
    value >>= 1;
  }
  return cnt & 0x1;
}

void getArmAngles(word *_foreArmAngle, word *_mainArmAngle){
  word command = 0b0100000000000000; // PAR=0 R/W=R
  command = command | AS5048A_ANGLE;

  //Add a parity bit on the the MSB
  command |= ((word)spiCalcEvenParity(command)<<15);

  //Split the command into two bytes
  byte right_byte = command & 0xFF;
  byte left_byte = ( command >> 8 ) & 0xFF;

  //SPI - begin transaction
  SPI.beginTransaction(settings);

  //Send the command
    digitalWrite(_cs, LOW);
    SPI.transfer(left_byte);
    SPI.transfer(right_byte);
    SPI.transfer(left_byte);
    SPI.transfer(right_byte);
    digitalWrite(_cs,HIGH);
  
  //Now read the response
  digitalWrite(_cs, LOW);
  byte a1_left_byte = SPI.transfer(0x00);
  byte a1_right_byte = SPI.transfer(0x00);
  byte a2_left_byte = SPI.transfer(0x00);
  byte a2_right_byte = SPI.transfer(0x00);
  digitalWrite(_cs, HIGH);

  //SPI - end transaction
  SPI.endTransaction();

  *_foreArmAngle = (( ( a1_left_byte & 0xFF ) << 8 ) | ( a1_right_byte & 0xFF )) & ~0xC000;
  //Serial.print(" : ");
  *_mainArmAngle = (( ( a2_left_byte & 0xFF ) << 8 ) | ( a2_right_byte & 0xFF )) & ~0xC000;
  //Return the data, stripping the parity and error bits
  //return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
}

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int} between -2^13 and 2^13
 */ /*word *_foreArmAngle, word *_mainArmAngle*/
void getRotation(){
  //word data;
  //int rotation;
  word foreArmAngle, mainArmAngle;
  getArmAngles(&foreArmAngle, &mainArmAngle);
  int fArmRotation = (int)foreArmAngle - (int)foreArmPosition;
  if(fArmRotation > 8191) fArmRotation = -((0x3FFF)-fArmRotation); //more than -180
  //if(rotation < -0x1FFF) rotation = rotation+0x3FFF;
  int mArmRotation = (int)mainArmAngle - (int)mainArmPosition;
  if(mArmRotation > 8191) mArmRotation = -((0x3FFF)-mArmRotation); //more than -180
  //return rotation;
  Serial.print(fArmRotation*-1);
  Serial.print(" : ");
  Serial.println(mArmRotation);
}
/*
 * Set the zero position
 */
void setZeroPosition(word arg_fArmposition, word arg_mArmposition){
  foreArmPosition = arg_fArmposition % 0x3FFF;
  mainArmPosition = arg_mArmposition % 0x3FFF;
}

/*
 * Returns the raw angle directly from the sensor
 */
/*void getRawRotation(){
  AS5048Aread(AS5048A_ANGLE);
}*

/**
 * returns the value of the state register
 * @return 16 bit word containing flags
 */
/*word AS5048AgetState(){
  return AS5048Aread(AS5048A_DIAG_AGC);
}

/**
 * Print the diagnostic register of the sensor
 */
/*void AS5048AprintState(){
  word data;

  data = AS5048AgetState();
  if(AS5048Aerror()){
    Serial.print("Error bit was set!");
  }
  Serial.println(data, BIN);
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
/*byte AS5048AgetGain(){
  word data = AS5048AgetState();
  return (byte) data & 0xFF;
}

/*
 * Get and clear the error register by reading it
 */
/*word AS5048AgetErrors(){
  return AS5048Aread(AS5048A_CLEAR_ERROR_FLAG);
}



/*
 * Returns the current zero position
 */
/*word AS5048AgetZeroPosition(){
  return position;
}

/*
 * Check if an error has been encountered.
 */
/*bool AS5048Aerror(){
  return errorFlag;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 */
word AS5048Aread(word registerAddress){
  word command = 0b0100000000000000; // PAR=0 R/W=R
  command = command | registerAddress;

  //Add a parity bit on the the MSB
  command |= ((word)spiCalcEvenParity(command)<<15);

  //Split the command into two bytes
  byte right_byte = command & 0xFF;
  byte left_byte = ( command >> 8 ) & 0xFF;

  //SPI - begin transaction
  SPI.beginTransaction(settings);

  
  //Send the command
    digitalWrite(_cs, LOW);
    SPI.transfer(left_byte);
    SPI.transfer(right_byte);
    SPI.transfer(left_byte);
    SPI.transfer(right_byte);
    digitalWrite(_cs,HIGH);
  
  //Now read the response
  digitalWrite(_cs, LOW);
  byte a1_left_byte = SPI.transfer(0x00);
  byte a1_right_byte = SPI.transfer(0x00);
  byte a2_left_byte = SPI.transfer(0x00);
  byte a2_right_byte = SPI.transfer(0x00);
  digitalWrite(_cs, HIGH);

  //SPI - end transaction
  SPI.endTransaction();

  Serial.print((( ( a1_left_byte & 0xFF ) << 8 ) | ( a1_right_byte & 0xFF )) & ~0xC000);
  Serial.print(" : ");
  Serial.println((( ( a2_left_byte & 0xFF ) << 8 ) | ( a2_right_byte & 0xFF )) & ~0xC000);
  //Return the data, stripping the parity and error bits
  //return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
}


/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
/*word AS5048Awrite(word registerAddress, word data) {

  word command = 0b0000000000000000; // PAR=0 R/W=W
  command |= registerAddress;

  //Add a parity bit on the the MSB
  command |= ((word)spiCalcEvenParity(command)<<15);

  //Split the command into two bytes
  byte right_byte = command & 0xFF;
  byte left_byte = ( command >> 8 ) & 0xFF;

#ifdef AS5048A_DEBUG
  Serial.print("Write (0x");
  Serial.print(registerAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif

  //SPI - begin transaction
  SPI.beginTransaction(settings);

  //Start the write command with the target address
  digitalWrite(_cs, LOW);
  SPI.transfer(left_byte);
  SPI.transfer(right_byte);
  digitalWrite(_cs,HIGH);
  
  word dataToSend = 0b0000000000000000;
  dataToSend |= data;

  //Craft another packet including the data and parity
  dataToSend |= ((word)spiCalcEvenParity(dataToSend)<<15);
  right_byte = dataToSend & 0xFF;
  left_byte = ( dataToSend >> 8 ) & 0xFF;

#ifdef AS5048A_DEBUG
  Serial.print("Sending data to write: ");
  Serial.println(dataToSend, BIN);
#endif

  //Now send the data packet
  digitalWrite(_cs,LOW);
  SPI.transfer(left_byte);
  SPI.transfer(right_byte);
  digitalWrite(_cs,HIGH);
  
  //Send a NOP to get the new data in the register
  digitalWrite(_cs, LOW);
  left_byte =-SPI.transfer(0x00);
  right_byte = SPI.transfer(0x00);
  digitalWrite(_cs, HIGH);

  //SPI - end transaction
  SPI.endTransaction();

  //Return the data, stripping the parity and error bits
  return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
}
*/
