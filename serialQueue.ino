#define QUEUE_SIZE 0x08
#define BUFFER_SIZE 16

boolean headerFound = false;
static byte serialQueue[QUEUE_SIZE][BUFFER_SIZE];
static byte serialBuffer[BUFFER_SIZE];
//for queue
byte serialHead = 0x00;
byte serialTail = 0x00;
boolean isEmpty = true;
boolean isService = false; // status to check serialQueue ; set only when no packet is received or queue is full
boolean isFull = false; 


void setup() {
  Serial.begin(9600);
  memset(serialBuffer,0x00, sizeof(serialBuffer));
  memset(serialQueue, 0x00, sizeof(serialQueue));
//  shiftCounter = 0x00;
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

void loop() {
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
      
      // newly init
      if(serialHead == serialTail && isEmpty){
        isEmpty = false;
        for(byte x = 0x00; x < pos; x++){
          //store data to perma queue
          serialQueue[serialTail][x] = serialBuffer[x]; 
        }
        
        pos = 0;
        serialTail = serialTail + 0x01; // increment tail
        headerFound = false;
      }
      
      // serialHead == serialTail !empty //full queue
      else if(serialHead == serialTail && !isEmpty){
        Serial.println("Full Queue");
        isFull = true;
        printBuffer();
        isService = true;
        pos = 0;
        
        //testing purposes only
//          head = head << 1; //moves head to next //deletes 1 slot away start at 0
//          isEmpty = false;
//          Serial.println(head, HEX);
//          Serial.println(tail, HEX);
//          pos = 0;
      }
      
      else{//store data to perma queue
        for(byte x = 0x00; x < pos; x++){
          serialQueue[serialTail][x] = serialBuffer[x]; 
          Serial.print(serialQueue[serialTail][x],HEX);
        }
        Serial.println();
//        Serial.print("H:");
//        Serial.println(serialHead);
//        Serial.print("T:");
//        Serial.println(serialTail);
        pos = 0;
        serialTail = serialTail + 0x01; //increment tail
        headerFound = false;
        if(serialTail == QUEUE_SIZE){
          serialTail = 0x00;
        }
      }
    }
  }
  else{
    isService = true;
  }
  
  if(isService){
    Serial.println("Checking if queue is empty"); // this is a flooding message
    isService = false;
  }
}

