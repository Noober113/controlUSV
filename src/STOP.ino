/*
 * control motor
 */
void STOP() {
  state = 0;
  analogWrite(pinMotorL, 10);
  analogWrite(pinMotorR, 70);
  Serial.println("MOTORR L AND R : 10 and 70");
  Serial.println("*************************STOP*************************");
}

void go_ahead(int speed_set) {
  analogWrite(pinMotorL, 10);
  analogWrite(pinMotorR, 70);
  delay(500);
  pcf8574.digitalWrite(P4, 1); //left motor
  pcf8574.digitalWrite(P5, 1); //left motor
  pcf8574.digitalWrite(P6, 1); //right motor
  pcf8574.digitalWrite(P7, 1);
  analogWrite(pinMotorL, speed_set);       //io2
  analogWrite(pinMotorR, speed_set + 60);  //io0
  Serial.println("Go ahead");
}
void turn_right(int speed_set) {
  pca9685.setPWM(SER0, 0, 10);  //turn right 
  pca9685.setPWM(SER1, 0, 10);  //turn right
  delay(500);
  analogWrite(pinMotorL, 15);
  analogWrite(pinMotorR, 75);
  // pcf8574.digitalWrite(P4, 1); //left motor
  // pcf8574.digitalWrite(P5, 1); //left motor
  // pcf8574.digitalWrite(P6, 0); //right motor
  // pcf8574.digitalWrite(P7, 0); //right motor
  // analogWrite(pinMotorL, speed_set);  //io2
  // analogWrite(pinMotorR, 70);         //io0

  Serial.println("Turn right");
}
void turn_left(int speed_set) {
  pca9685.setPWM(SER0, 0, 170);  //turn right 
  pca9685.setPWM(SER1, 0, 170);  //turn right
  delay(500);
  analogWrite(pinMotorL, 15);
  analogWrite(pinMotorR, 75);
  // pcf8574.digitalWrite(P4, 0); //left motor
  // pcf8574.digitalWrite(P5, 0); //left motor
  // pcf8574.digitalWrite(P6, 1); //right motor
  // pcf8574.digitalWrite(P7, 1); //right motor
  // analogWrite(pinMotorR, 10);         //io2
  // analogWrite(pinMotorL, speed_set);  //io0
  Serial.println("Turn left");
}
void backward(int speed_set) {
  analogWrite(pinMotorL, 10);
  analogWrite(pinMotorR, 70);
  delay(500);
  pcf8574.digitalWrite(P4, 0); //left motor
  pcf8574.digitalWrite(P5, 0); //left motor
  pcf8574.digitalWrite(P6, 0); //right motor
  pcf8574.digitalWrite(P7, 0); //right motor
  analogWrite(pinMotorR, speed_set);         //io2
  analogWrite(pinMotorL, speed_set);  //io0
  Serial.println("backward");
}