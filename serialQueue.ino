void setup() {
  Serial.begin(9600);
//  printBuffer();
}

#define QUEUE_SIZE 0x05
#define BUFFER_SIZE 30
boolean headerFound = false;
static byte serialQueue[QUEUE_SIZE][BUFFER_SIZE];
static byte serialBuffer[BUFFER_SIZE];

void printBuffer(){
  for(byte y = 0x00; y < QUEUE_SIZE; y++){
    for(byte x = 0x00; x < BUFFER_SIZE; x++){
      Serial.print(serialQueue[y][x],HEX);
    }
    Serial.println();
  }
  
}
void loop() {
  // you can read one
  //trying serialQueue
  static byte index = 0; //for queue
  static size_t pos = 0; //for buffer
  byte serialData; 
  
  if(Serial.available()>0){
//    while(Serial.available()){
      serialData = Serial.read(); // 1 byte
      if(serialData == 0xFF){ // header found start reading
        headerFound = true;
      }
      if(headerFound){
        serialBuffer[pos++] = serialData;
      }
      if(serialData == 0xFE){
        serialBuffer[pos] = 0xFE; //adds footer
        if(index != QUEUE_SIZE){ //if not max size
          for(byte x = 0x00; x < pos; x++){
            serialQueue[index][x] = serialBuffer[x]; //store data to perma queue
//          Serial.println(serialQueue[index][x], HEX);
          }
          index++; 
          pos = 0;
          headerFound = false;
          Serial.println(index); // patched index
          printBuffer();
        }
        else{
          Serial.println("Max");
        }
//      }
//      Serial.print(serialData,HEX);
    }
  }
}

