#define QUEUE_SIZE 0x08
#define BUFFER_SIZE 16

boolean headerFound = false;
static byte serialQueue[QUEUE_SIZE][BUFFER_SIZE];
static byte serialBuffer[BUFFER_SIZE];
//for queue
byte serialHead = 0x01; // 1 head = 0 AT ARRAY
byte serialTail = 0x01;
byte shiftCounter = 0x00;
boolean isEmpty = true;
boolean isService = false; // status to check serialQueue ; set only when no packet is received or queue is full


void setup() {
  Serial.begin(9600);
  memset(serialBuffer,0x00, sizeof(serialBuffer));
  memset(serialQueue, 0x00, sizeof(serialQueue));
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
    serialData = Serial.read(); // 1 byte
    
    if(serialData == 0xFF){ // serialhead found start reading
      headerFound = true;
    }
    
    if(headerFound){
      serialBuffer[pos++] = serialData;
    }
    
    if(serialData == 0xFE){
      serialBuffer[pos] = 0xFE; //adds footer
      
      //check if header == tail and isempty 
      // newly init
      if(serialHead == serialTail && isEmpty){
        isEmpty = false;
        positionCounter(serialTail);
        for(byte x = 0x00; x < pos; x++){
          //store data to perma queue
          serialQueue[shiftCounter][x] = serialBuffer[x]; 
        }
        // shift tail
        shiftCounter = 0x00;
        pos = 0;
        serialTail = serialTail << 1;
        headerFound = false;
      }
      
      // serialHead == serialTail !empty //full queue
      else if(serialHead == serialTail && !isEmpty){
        Serial.println("Full Queue");
        printBuffer();
        isService = true;
        pos = 0;
        shiftCounter = 0x00;
        Serial.println(serialHead, HEX);
        Serial.println(serialTail, HEX);

        //testing purposes only
//          head = head << 1; //moves head to next //deletes 1 slot away start at 0
//          isEmpty = false;
//          Serial.println(head, HEX);
//          Serial.println(tail, HEX);
//          pos = 0;
      }
      
      else{
        positionCounter(serialTail);
        
        for(byte x = 0x00; x < pos; x++){
          //store data to perma queue
          serialQueue[shiftCounter][x] = serialBuffer[x]; 
//            Serial.println(serialQueue[shiftCounter][x],HEX);
        }
        Serial.println("tis con");
        shiftCounter = 0x00;
        pos = 0;
        // shift serialTail
        serialTail = serialTail << 1;
        headerFound = false;
        if(serialTail == 0x00){
          serialTail = 0x01;
        }
        
      }
    }
  }
  else{
    isService = true;
  }
  
  if(isService){
//    Serial.println("Check if queue is empty"); // this is a flooding message
    isService = false;
  }
}

