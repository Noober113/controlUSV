void encodeGPS()
{
  //Serial.print("%");
  if (gps.encode(gpsSerial.read()))
  {
    if (gps.time.isValid())
    {
      Minute = gps.time.minute();
      Second = gps.time.second();
      Hour   = gps.time.hour();
      //Serial.println("Time Valide");
    }
    // get date drom GPS module
    if (gps.date.isValid())
    {
      //Serial.println("Date Valide");
      Day   = gps.date.day();
      Month = gps.date.month();
      Year  = gps.date.year();
    }
    if (gps.location.isValid())
    {
      Longitude = String(gps.location.lng(), 7);
      //Serial.println("Longitude : " + Longitude);
      LongitudeDouble = gps.location.lng();
      //Serial.println("Longitude 10k: " + Longitude10k);
      Latitude = String(gps.location.lat(), 7);
      //Serial.println("Latitude : " + Latitude);
      LatitudeDouble = gps.location.lat();
      //Serial.println("Latitude 10k: " + Latitude10k);
      //Serial.println("Longitude : " + Longitude + " Latitude : " + Latitude);;
      //Serial.print(",");
      etatGPS = 2;
    }
    else
    {
      etatGPS = 1;
    }
    if (gps.location.isValid())
    {
      fixage = "OK";
    }
    if (gps.speed.isValid())
    {
      speed1 = String(gps.speed.kmph(), 2);
    }
    if (last_second != gps.time.second()) // if time has changed
    {
      last_second = gps.time.second();
      setTime(Hour, Minute, Second, Day, Month, Year);// set current UTC time
      adjustTime(time_offset);                        // add the offset to get local time
      // update time array
      Time[0]  = hour()   / 10 + '0';
      Time[1]  = hour()   % 10 + '0';
      Time[3]  = minute() / 10 + '0';
      Time[4] = minute() % 10 + '0';
      Time[6] = second() / 10 + '0';
      Time[7] = second() % 10 + '0';
      // update date array
      Date[0]  =  day()   / 10 + '0';
      Date[1]  =  day()   % 10 + '0';
      Date[3]  =  month() / 10 + '0';
      Date[4] =  month() % 10 + '0';
      Date[8] = (year()  / 10) % 10 + '0';
      Date[9] =  year()  % 10 + '0';
    }
  }

}
