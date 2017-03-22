void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println(Serial.available());
  byte byteArray[0x01]; 
  byte index = 0x00;
  if(Serial.available() > 0){
      byte val = Serial.read();
//      Serial.print("Val");
      Serial.println(val, HEX);
      byteArray[index] = val;
      index = index + 0x01;
 
  }
//  for(byte i = 0x00; i <= index ; i++){
//    Serial.print(byteArray[i], HEX);
//  }
}
