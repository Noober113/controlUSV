void obstacle_avodance() {
  switch (state_avodance) {
    case 0:
      if (distance_1 < 30) {
        if (distance_2 < 30||distance_4 < 30) {
          if (distance_3 < 30) {
            Serial.println("Go backward");
            backward(20);
            state_avodance = 0;
            break;
          } else {
            state_avodance = 1;
            break;
          }
        } else {
          if (distance_3 < 30) {
            state_avodance = 3;
            break;  //mui tau
          } else {
            state_avodance = 1;
            break;
          }
        }
      } else {
        //Serial.println("Done");
        state_avodance = 2;
        break;
      }
      break;
    case 1:
      if (distance_1 < 30) {
        if (distance_2 < 30||distance_4 < 30) {
          Serial.println("turn right");
          turn_right(20);  //increase speed
          state_avodance = 1;
        } else {
          Serial.println("Increase speed in 10s");
          go_ahead(20);
          delay(10000);
          if(cap180<170 && distanceToNextPoint >20){
            Serial.println("Location near obtackle");
            j=j+1;
            state=1;
            break;                       
          }
          state_avodance = 1;
          break;  //mui tau
        }
      } else {
        if (distance_2 < 30||distance_4 < 30) {
          Serial.println("Turn right");
          turn_right(15);
          state_avodance = 1;
          break;
        } else {
          Serial.println("Done");
          state = 1;
          break;
        }
      }
      break;
    case 2:
      if (distance_3 < 30) {
        if (distance_2 < 30||distance_4 < 30) {
          Serial.println("turn left");
          turn_left(20);  //increase speed
          state_avodance = 2;
        } else {
          Serial.println("Turn right");
          turn_right(15);
          state_avodance = 2;
          break;  //mui tau
        }
      } else {
        if (distance_2 < 30||distance_4 < 30) {
          Serial.println("Turn left");
          turn_left(15);
          state_avodance = 1;
          break;
        } else {
          Serial.println("Done");
          state = 1;
          break;
        }
      }
      break;
    case 3:
      if (distance_1 < 30) {
        if (distance_3 < 30) {
          Serial.println("Increase Speed");
          go_ahead(20);  //increase speed
          state_avodance = 3;
        } else {
          Serial.println("Turn right");
          turn_right(15);
          state_avodance = 2;
          break;  //mui tau
        }
      } else {
        if (distance_2 < 30||distance_4 < 30) {
          Serial.println("Turn left");
          turn_left(15);
          state_avodance = 1;
          break;
        } else {
          Serial.println("Done");
          state = 1;
          break;
        }
      }
      break;
  }
}
