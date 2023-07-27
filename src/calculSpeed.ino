/*
   Gere la vitesse en fonction de la distance restante avec le prochain point
//Quản lý tốc độ theo khoảng cách còn lại với điểm tiếp theo
   Control les moteurs
*/
void calculSpeed() {
  if (distanceToNextPoint >= 15)  //Plus de 3km
  {
    if (speed_edit.toInt() == 0) {
      speed_set = 15;
    } else {
      speed_set = speed_edit.toInt();
    }
  } else if (distanceToNextPoint >= 5 && distanceToNextPoint < 15)  // Entre 101m et 3km
  {
    if (speed_edit.toInt() == 0) {
      speed_set = 15;
    } else {
      speed_set = speed_edit.toInt()-1;
    }                                  //Vitesse Max
  } else if (distanceToNextPoint < 5)  //Entre 5m et 20m
  {
    if (speed_edit.toInt() == 0) {
      speed_set = 13;
    } else {
      speed_set = speed_edit.toInt()-3;
    }                                   // Vitesse Lente
  } else if (distanceToNextPoint <= 1)  //Moins de 5m
  {
    speed_set = 10;  // STOP
  }

  //controlMotor();
  //keep_balance();
  analogWrite(pinMotorR, speed_set + 1);
  analogWrite(pinMotorL, speed_set + 60);
}

void controlMotor() {
  if (tribord) {
    analogWrite(pinMotorR, speed_set);
    Serial.println("Right motor : " + String(speed_set));
  } else {
    analogWrite(pinMotorR, 10);
    Serial.println("Right motor : 10");
  }
  delay(10);
  if (babord) {
    analogWrite(pinMotorL, speed_set);
    Serial.println("Left motor : " + String(speed_set));
  } else {
    analogWrite(pinMotorL, 70);  //Arret du moteur
    Serial.println("Left motor : 70");
  }
  delay(10);
}
// void keep_balance() {
//   setpoint = 180;
//   pid.Compute();
//   if (output >= 0) {
//     analogWrite(pinMotorT, 12);
//     delay(500);
//     pcf_MUI.digitalWrite(P0, 1);  // reverse
//     pcf_MUI.digitalWrite(P1, 1);
//     analogWrite(pinMotorT, output);
//   } else if (output < 0) {
//     int val = output * (-1);
//     analogWrite(pinMotorT, 10);
//     delay(500);
//     pcf_MUI.digitalWrite(P0, 0);  // reverse
//     pcf_MUI.digitalWrite(P1, 0);
//     analogWrite(pinMotorT, val);
//   }
// }
