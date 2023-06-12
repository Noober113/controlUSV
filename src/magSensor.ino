/*
 * Lecture des données de la boussole (et enregistre la position) 
 */
float magSensor()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  

  float declinationAngle = 1.3;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  headingDegrees = map(headingDegrees, 0, 360, 360, 0);
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;
}
