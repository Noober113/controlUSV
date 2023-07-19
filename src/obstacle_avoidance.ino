void obstacle_avodance() {
  switch (state_avodance) {
    case 0:
      if (distance_1 < obtacle_distance) {
        if (distance_2 < obtacle_distance || distance_4 < obtacle_distance) {
          if (distance_3 < obtacle_distance) {
            Serial.println("Go backward");
            backward(20);
            state_avodance = 0;
          } else {
            state_avodance = 1;
          }
        } else {
          if (distance_3 < obtacle_distance) {
            state_avodance = 3;
            //mui tau
          } else {
            state_avodance = 1;
          }
        }
      } else {
        state_avodance = 2;
      }
      break;
    case 1:
      if (distance_1 < obtacle_distance) {
        if (distance_2 < obtacle_distance) {
          Serial.println("turn right");
          turn_right(5);  //increase speed
          state_avodance = 0;
        } else {
          Serial.println("turn right");
          turn_right(5);
          state_avodance = 0;
        }
      } else {
        if (distance_2 < obtacle_distance) {
          if (distance_4 < obtacle_distance) {
            Serial.println("turn right");
            turn_right(5);
            state_avodance = 0;
          } else {
            Serial.println("Turn right");
            turn_right(5);
            state_avodance = 0;
          }
        } else if (distance_4 < obtacle_distance) {
          Serial.println("Turn left");
          turn_left(270);
          state_avodance = 0;
        } else {
          Serial.println("Done");
          delay(100);
          analogWrite(pinMotorT, 10);
          pcf_MUI.digitalWrite(P0, 1);
          pcf_MUI.digitalWrite(P1, 1);
          state = 1;
          state_avodance = 0;
          break;
        }
      }
      break;
    case 2:
      if (distance_3 < obtacle_distance) {
        if (distance_2 < obtacle_distance || distance_4 < obtacle_distance) {
          Serial.println("turn left");
          turn_left(350);  //increase speed
          state_avodance = 0;
        } else {
          Serial.println("turn left");
          turn_left(350);
          analogWrite(pinMotorL, 75);
          analogWrite(pinMotorR, 15);
          // Serial.println("Increase speed");
          // go_ahead(17);
          state_avodance = 0;
          //mui tau
        }
      } else {
        if (distance_2 < obtacle_distance) {
          if (distance_4 < obtacle_distance) {
            Serial.println("Turn right");
            turn_right(5);
            state_avodance = 0;
          } else {
            Serial.println("Turn right");
            turn_right(90);
            analogWrite(pinMotorL, 75);
            analogWrite(pinMotorR, 15);
            state_avodance = 0;
          }
        } else if (distance_4 < obtacle_distance) {
          Serial.println("Turn left");
          turn_left(270);
          analogWrite(pinMotorL, 75);
          analogWrite(pinMotorR, 15);
          state_avodance = 0;
        } else {
          Serial.println("Done");
          analogWrite(pinMotorT, 10);
          delay(100);
          pcf_MUI.digitalWrite(P0, 1);
          pcf_MUI.digitalWrite(P1, 1);
          state = 1;
          state_avodance = 0;
          break;
        }
      }
      break;
    case 3:
      if (distance_1 < obtacle_distance) {
        if (distance_3 < obtacle_distance) {
          Serial.println("Increase Speed");
          go_ahead(13);  //increase speed
          state_avodance = 0;
          if (distance_3 < 30) {
            control_servo(180);
          }
          if (distance_1 < 30) {
            control_servo(165);
            analogWrite(pinMotorL, 75);
            analogWrite(pinMotorR, 15);
          }
        } else {
          Serial.println("turn right");
          turn_right(90);
          // Serial.println("Increase Speed");
          // go_ahead(17);
          analogWrite(pinMotorL, 75);
          analogWrite(pinMotorR, 15);
          state_avodance = 0;
        }
      } else {
        if (distance_2 < obtacle_distance) {
          if (distance_4 < obtacle_distance) {
            Serial.println("Turn right");
            turn_right(5);
            state_avodance = 0;
          } else {
            Serial.println("Turn right");
            turn_right(5);
            analogWrite(pinMotorL, 75);
            analogWrite(pinMotorR, 15);
            state_avodance = 0;
          }
        } else if (distance_4 < obtacle_distance) {
          Serial.println("Turn left");
          turn_left(270);
          analogWrite(pinMotorL, 75);
          analogWrite(pinMotorR, 15);
          state_avodance = 0;
        } else {
          Serial.println("Done");
          analogWrite(pinMotorT, 10);
          delay(100);
          pcf_MUI.digitalWrite(P0, 1);
          pcf_MUI.digitalWrite(P1, 1);
          
          state = 1;
          state_avodance = 0;
          break;
        }
      }
      break;
  }
}
