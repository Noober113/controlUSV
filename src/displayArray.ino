void displayArray() {
  Serial.println("--------------------------------------------------------------");
  for (int i = 0; i < nbLatLng; i++) {
    Serial.print((double)latLng[0][i],20);
    Serial.print("\t");
    Serial.println((double)latLng[1][i],20);
  }
  Serial.println("--------------------------------------------------------------");
}