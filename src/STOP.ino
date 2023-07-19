/*
 * control motor
 */
void STOP() {
  state = 0;
  analogWrite(pinMotorL, 70);
  analogWrite(pinMotorR, 10);
  Serial.println("MOTORR L AND R : 10 and 70");
  Serial.println("*************************STOP*************************");
}

void go_ahead(int speed_set) {
  pcf8574.digitalWrite(P4, 1); //left motor
  pcf8574.digitalWrite(P5, 1); //left motor
  pcf8574.digitalWrite(P6, 1); //right motor
  pcf8574.digitalWrite(P7, 1);
  control_servo(180);
  analogWrite(pinMotorR, speed_set);       //io2
  analogWrite(pinMotorL, speed_set + 60);  //io0
  Serial.println("Go ahead");
}
void turn_right(int angle) {
  control_servo(angle);//5
  analogWrite(pinMotorL, 73);
  analogWrite(pinMotorR, 13);
  pcf_MUI.digitalWrite(P0, 0);
  pcf_MUI.digitalWrite(P1, 0); //left motor
  analogWrite(pinMotorT,  speed_motorT.toInt()+3);  
  Serial.println("Turn right");
}
void turn_left(int angle) {
  control_servo(angle);//270
  analogWrite(pinMotorL, 73);
  analogWrite(pinMotorR, 13);
  pcf_MUI.digitalWrite(P0, 1);
  pcf_MUI.digitalWrite(P1, 1);
  analogWrite(pinMotorT, speed_motorT.toInt()+3);         
  Serial.println("Turn left");
}
void backward(int speed_set) {
  analogWrite(pinMotorL, 70);
  analogWrite(pinMotorR, 10);
  delay(200);
  control_servo(180);
  //analogWrite(pinMotorL, speed_set);  //io0
  Serial.println("backward");
}