#define QUEUE_SIZE 0x08  // @ max 4
#define BUFFER_SIZE 16

boolean headerFound = false;
static byte serialQueue[QUEUE_SIZE][BUFFER_SIZE];
static byte serialBuffer[BUFFER_SIZE];
//for queue
byte head = 0x01; // 1 head = 0 AT ARRAY
byte tail = 0x01;
byte shiftCounter = 0x00;
boolean isEmpty = true;

void setup() {
  Serial.begin(9600);
  shiftCounter = 0x00;
}

void printBuffer(){
  byte x = 0x00;
  byte halt = false;
  for(byte y = 0x00; y < QUEUE_SIZE; y++){
    for(byte x = 0x00; x < BUFFER_SIZE; x++){  
      while(!halt){
        Serial.print(serialQueue[y][x],HEX);
        if(serialQueue[y][x] == 0xFE)
          halt = true; 
        x = x + 0x01;
      }
    }
    halt = false;
    Serial.println();
  }
}

void positionCounter(byte data){ // position in serial queue
  Serial.print("D: ");
  Serial.println(data,HEX);
  while((data & 0x01) != 0x01){
    shiftCounter = shiftCounter + 0x01;
    data  = data >> 1;
  }
  Serial.print("S: ");
  Serial.println(shiftCounter,HEX);
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
        //check if header == tail and isempty // newly init
        if(head == tail && isEmpty){
          isEmpty = false;
          positionCounter(tail);
          for(byte x = 0x00; x < pos; x++){
            //store data to perma queue
            serialQueue[shiftCounter][x] = serialBuffer[x]; 
          }
          // shift tail
          shiftCounter = 0x00;
          pos = 0;
          tail = tail << 1;
          headerFound = false;
        }
        // header == tail !empty //full queue
        else if(head == tail && !isEmpty){
          Serial.println("Full Queue");
          printBuffer();
        }
        else{
          positionCounter(tail);
          for(byte x = 0x00; x < pos; x++){
            //store data to perma queue
            serialQueue[shiftCounter][x] = serialBuffer[x]; 
//            Serial.println(serialQueue[shiftCounter][x],HEX);
          }
          shiftCounter = 0x00;
          pos = 0;
          // shift tail
          tail = tail << 1;
          headerFound = false;
          if(tail == 0x00){
            tail = 0x01;
          }
        }
        
    }
  }
}

