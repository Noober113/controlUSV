
void calculOrderheadinging(float capActuel) {

  if (gps.location.isValid()) {
    /*********************Calcul de la distance avec le prochain point tính toán khoảng cách với điểm tiếp theo***********************/
    distanceToNextPoint = (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      nextLati,
      nextLong);

    /*********** Tính toán của chặn đường (chênh lệch chặn đường) với điểm tiếp theo****************/
    courseToNextPoint = TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      nextLati,
      nextLong);  //0 mean go a head 90 degrees indicating a right turn, 180 degrees indicating a U-turn, and 270 degrees indicating a left turn.
  }
  Serial.print("Distance : ");
  Serial.println(distanceToNextPoint);
  Serial.print("Course : ");
  Serial.println(courseToNextPoint);

  /********************* tính toán để quyết định đi bên trái hay bên phải****************/
  courseTo180 = 180 - courseToNextPoint;
  cap180 = dans360(capActuel + courseTo180);
  Serial.print("Objectif de CAP : " + String(cap180));
  //sent to esp motor
  //myData.id = BOARD_ID;
  //esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  pwm0 = map(cap180, 0, 360, SERVOMIN, SERVOMAX);
  if (pwm_temp > pwm0) {
    while (pwm_temp > pwm0) {

      pca9685.setPWM(SER0, 0, pwm_temp);
      pca9685.setPWM(SER1, 0, pwm_temp);
      pwm_temp--;
      delay(5);
    }
    pwm_temp = pwm0;
  } else {
    while (pwm_temp < pwm0) {
      pca9685.setPWM(SER0, 0, pwm_temp);
      pca9685.setPWM(SER1, 0, pwm_temp);
      pwm_temp++;
      delay(5);
    }
    pwm_temp = pwm0;
  }

  /*********************Ordre d'allez à babord, à tribord ou tout droit**************************/
  if (cap180 > 185 && cap180 < 273)  //|| cap180<-5
  {
    Serial.println(" Turn left");  // rẻ trái
    ;//cap180=270;
    //control_servo(cap180);
  } 
  else if (cap180 >= 273)        // rẻ phải
  {
    //cap180=350;
    Serial.println(" Turn right");
    //control_servo(cap180);
  } 
  else if (cap180 < 88)  // rẻ phải
  {
    //cap180=44;
    Serial.println(" Turn right");
    //control_servo(cap180);
  } 
  else if (cap180 >= 88 && cap180 < 175) {
    Serial.println(" Turn right");
    //cap180=132;    
    //control_servo(cap180);
  } 
  else {
    Serial.println(" Go ahead");
    //cap180=180;
    //control_servo(cap180);
  }
}


/***************************Remets l'orientation dans [0;360[*************/
short dans360(short valeur) {
  if (valeur >= 360) {
    valeur = valeur - 360;
  }
  if (valeur <= 0) {
    valeur = 180 + (360 - courseToNextPoint) + magSensor();
    if (valeur >= 360) {
      valeur = valeur - 360;
    }
  }
  return valeur;
}
void control_servo(int cap) {
  pwm0 = map(cap, 0, 360, SERVOMIN, SERVOMAX);
  if (pwm_temp > pwm0) {
    while (pwm_temp > pwm0) {

      pca9685.setPWM(SER0, 0, pwm_temp);
      pca9685.setPWM(SER1, 0, pwm_temp);
      pwm_temp--;
      delay(5);
    }
    pwm_temp = pwm0;
  } else {
    while (pwm_temp < pwm0) {
      pca9685.setPWM(SER0, 0, pwm_temp);
      pca9685.setPWM(SER1, 0, pwm_temp);
      pwm_temp++;
      delay(5);
    }
    pwm_temp = pwm0;
  }
}