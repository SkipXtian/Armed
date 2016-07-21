
void drawSquare()
{
  goDirectlyTo(0, 100, 0);
  delay(1000);
  for (byte i = 100; i <= 200; i++) {
    //delay(10);
    while (running == true) {
      delay(1);
    } //wait for move to complete
    goDirectlyTo(0, i, 0);
  }
  for (byte i = 0; i <= 100; i++) {
    while (running == true) {
      delay(1);
    } //wait for move to complete
    goDirectlyTo(0, 200, i);
  }
  for (byte i = 200; i >= 100; i--) {
    while (running == true) {
      delay(1);
    } //wait for move to complete
    goDirectlyTo(0, i, 100);
  }
  for (byte i = 100; i >= 0; i--) {
    while (running == true) {
      delay(1);
    } //wait for move to complete
    goDirectlyTo(0, 100, i);
  }

}
