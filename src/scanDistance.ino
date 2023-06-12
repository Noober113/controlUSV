// void scanDistance() {
//   for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
//     pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
//     pca9685.setPWM(SER0, 0, pwm0);
//     int distance = readDistance(2, 1);
//     if (distance < 10) {
//       Serial.println("obstacle");
      
//     }
//     delay(30);
//   }
//   for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
//     pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
//     pca9685.setPWM(SER0, 0, pwm1);
//     int distance = readDistance(2, 1);
//     Serial.println(distance);
//     if (distance < 10) {
//       Serial.println("obstacle");
      
//     }
//     // } else if (distance < 10 && posDegrees < 50) {
//     //   Serial.println("obstacle is right");
//     // } else if (distance < 10 && posDegrees < 100 && posDegrees > 50) {
//     //   Serial.println("obstacle is adhead");
//     // } else if (distance == 0) {
//     //   Serial.println("Bad signal");
//       break;
//     delay(30);
//   }
// }
