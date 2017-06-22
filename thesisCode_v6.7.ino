/*Uhm hi, you start counting at 0, okay, just for the gateway settings*/
/*Programmed for Gizduino IOT-644*/
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <SD.h>
#include <avr/interrupt.h>
#include <math.h>
#include <Servo.h>
#define MAX_COMMAND_CODE 0x05 //note: starts from 0; 
#define MAX_COMMAND 0x03 // 12 ports * 3 modes if sensor/actuator ; placed as 2 temporarily
#define PORT_COUNT 0x0C // 12 ports
#define PACKET_QUEUE_SIZE 0x02 // Queue for packet (index starts @ 0) can be changed :D
#define SERIAL_QUEUE_SIZE 0x05 // Queue for serial (index starts @ 0) can be changed :D
#define BUFFER_SIZE 0x13 // bytes queue will hold; preferably wag nalang galawin ^^
#define MAX_CONFIG_PART 0x05 // receiving config
#define MAX_ATTEMPT 0x03 // contacting sink
#define SUCCESS_LED_PIN 20 // startup pin to indicate success
#define ERROR_LED_PIN 21 //startup pin to indicate error
unsigned int timeoutVal = 0x2005; //5 seconds

int CS_PIN = 27; // for SPI; 27 is used for Gizduino IOT-644
byte errorFlag = 0x00; // bits correspond to an error
byte logicalAddress = 0x00; //contains address of the network
byte physicalAddress = 0x01; //unique address of the node (frodam 0-255) (source address)
byte sinkAddress = 0x00; // specifically sink node in case in the future the nodes talk each other na
byte destAddress = 0x00; //destination address (source address is set to destination address)
byte sourceAddress = 0x00; //when receiving packet
byte configVersion = 0x00; //node config version
volatile byte attemptCounter = 0x00; //attempts count for contacting sink
volatile byte attemptIsSet = false; //tells that the attempt is just counted (else mag spam siya until mag %0 ulit)
byte configPartNumber = 0x00; // stores the config part last served
byte configSentPartCtr = 0x00; //config part being read
boolean timerReset = false; //set when there is time config

boolean requestConfig = false; // checks if node is requesting config

byte apiCount = 0x01; //API version
byte commandCount = 0x00; //command being served (at retrieve)
byte commandCounter = 0x00; //commandCounter for commands; for receiving (@ receive)
byte packetCommandCounter = 0x00; //command counter where count will be placed @ packet
byte packetPos = 0x06; //position of packet; default at command na

unsigned int eventRegister = 0x0000; // if set, event happened; 0-15
unsigned int onDemandRegister = 0x0000; // if set, then there is an odm; 0-15
unsigned int timerRegister = 0x0000; //if set, there is timer triggered; 0 - 15
unsigned int configChangeRegister = 0x0000; // if set, port config was changed

byte portConfigSegment[(int) PORT_COUNT]; // contains port type, odm/event/time mode
unsigned int actuatorValueOnDemandSegment[(int) PORT_COUNT]; // stores data to write on actuator if ODM
unsigned int portValue[(int) PORT_COUNT]; //stores port values but in BCD
unsigned int timerSegment[(int) PORT_COUNT]; //timer segment
unsigned int eventSegment[(int) PORT_COUNT * 2]; //event segment - 24 slots (0-23(17h)) 0-B (if threshold mode) C-17 (if range mode)
int convertedEventSegment[(int) PORT_COUNT * 2]; // decimal values of event segment
unsigned int actuatorDetailSegment[(int) PORT_COUNT]; //actuator details segment (event)
unsigned int actuatorValueTimerSegment[(int) PORT_COUNT]; // stores data to write on actuator if timer
unsigned int portDataChanged; // stores if the data is changed
unsigned int eventRequest = 0x0000; //if event is requested at port
volatile unsigned int timerRequest = 0x0000; // if timer is requested at port
volatile unsigned int timerGrant = 0x0000; // if timer is granted at port

byte segmentCounter = 0x00;  // counter for parsing the parts of the packet
byte tempModeStorage = 0x00; // stores the port config 00-07
byte portNum = 0x00;//parsing var
byte partCounter = 0x00; //data counter for more than 1 byte data when receiving configs
byte commandValue = 0x00; // for api = 3, contains the command value
byte checker = 0x00; // for api = 2 checker for threshold/range mode; api = 3 counter for receive (port num i think)

boolean headerFound = false;
byte serialQueue[SERIAL_QUEUE_SIZE][BUFFER_SIZE];
byte serialBuffer[BUFFER_SIZE];
//for serial queue
byte serialHead = 0x00;
byte serialTail = 0x00;
boolean isEmpty = true;
boolean isService = false; // status to check serialQueue ; set only when no packet is received or queue is full
boolean isFull = false;

//for packet queue
//reuse headerFound
byte packetQueue[PACKET_QUEUE_SIZE][BUFFER_SIZE];
//byte packetBuffer[BUFFER_SIZE];
byte packetQueueHead = 0x00;
byte packetQueueTail = 0x00;
boolean packetQisEmpty = true;
boolean packetQisService = false;
boolean packetQisFull = false;

byte packetTypeFlag = 0x00; //determines the type of packet received
SoftwareSerial xbee(2,3);

volatile unsigned long timeCtr; // counter for overflows
long portOverflowCount [(int) PORT_COUNT + 0x01]; //stores the overflow counters to be checked by interrupt; last one is for timeout

//Messages used which would be saved to the program space (flash) to save space
const char segmentBranch0[] PROGMEM = "@ PORT CONFIG SEGMENT";
const char segmentBranch1[] PROGMEM = "@ TIME SEGMENT";
const char segmentBranch2[] PROGMEM = "@ EVENT SEGMENT";
const char segmentBranch3[] PROGMEM = "@ ACTUATOR SEGMENT";
const char portConfigTypeBranch0[] PROGMEM = "Time based";
const char portConfigTypeBranch1[] PROGMEM = "Event base";
const char portConfigTypeBranch2[] PROGMEM = "ODM";
const char odmDetail0[] PROGMEM = "Port Num: ";
const char odmDetail1[] PROGMEM = "onDemandRegister: ";
const char odmDetail2[] PROGMEM = "Value of AND: ";
const char odmDetail3[] PROGMEM = "Value of TEMP: ";
const char errorMessage0[] PROGMEM = "Invalid port configuration!";
const char errorMessage1[] PROGMEM = "No SD Card Read";
const char errorMessage2[] PROGMEM = "Cannot access SD Card";
const char infoMessage0[] PROGMEM = "Range Mode";
const char infoMessage1[] PROGMEM = "Threshold Mode";
const char infoMessage2[] PROGMEM = "Keep Alive Message Requested";
const char infoMessage3[] PROGMEM = "All read";
const char infoMessage4[] PROGMEM = "Getting Next Command";
const char infoMessage5[] PROGMEM = "All Commands Served";
const char infoMessage6[] PROGMEM = "Port Details: ";
const char infoMessage7[] PROGMEM = "Setting Timer Segment: ";
const char infoMessage8[] PROGMEM = "Updated Timer Segment: ";
const char infoMessage9[] PROGMEM = "Full Timer Segment: ";
const char infoMessage10[] PROGMEM = "Updated Event Segment1: ";
const char infoMessage11[] PROGMEM = "Full Event Segment: ";
const char infoMessage12[] PROGMEM = "Logical Address: ";
const char infoMessage13[] PROGMEM = "Physical Address: ";
const char infoMessage14[] PROGMEM = "Config Version: ";
const char infoMessage15[] PROGMEM = "Actuator onDemand Segment: ";
const char infoMessage16[] PROGMEM = "Port Value: ";
const char infoMessage17[] PROGMEM = "Sensor";
const char infoMessage18[] PROGMEM = "Actuator";
const char infoMessage19[] PROGMEM = "Incoming Port Configuration";
const char infoMessage20[] PROGMEM = "Actuator Timer Segment: ";
const char actuatorDetail0[] PROGMEM = "Actuator Segment: ";

const char* const messages[] PROGMEM = {segmentBranch0, segmentBranch1, segmentBranch2, segmentBranch3, portConfigTypeBranch0, portConfigTypeBranch1, portConfigTypeBranch2, odmDetail0, odmDetail1, odmDetail2, odmDetail3,
                                        errorMessage0, infoMessage0, infoMessage1, infoMessage2, actuatorDetail0, infoMessage3, infoMessage4, infoMessage5, infoMessage6, infoMessage7,
                                        infoMessage8, infoMessage9, infoMessage10, infoMessage11, errorMessage1, errorMessage2, infoMessage12, infoMessage13, infoMessage14, infoMessage15, infoMessage16, infoMessage17, infoMessage18, infoMessage19, infoMessage20
                                       };
char buffer[32]; //update according to longest message

Servo servo0, servo1, servo2, servo3, servo4, servo5;

void printQueue(byte temp[][BUFFER_SIZE], byte queueSize) { // you can remove the queue_Size and buffer size; prints serialBuffer
  byte x = 0x00;
  byte halt = false;
  for (byte y = 0x00; y < queueSize; y++) {
    while (!halt) {
      if (temp[y][x] == 0xFE) {
        xbee.print(temp[y][x], HEX);
        halt = true;
      }
      else {
        xbee.print(temp[y][x], HEX);
      }

      if (x != BUFFER_SIZE)
        x = x + 0x01;
      else {
        halt = true;
      }
    }
    halt = false;
    x = 0x00;
    xbee.println();
  }
}

void printBuffer(byte temp[]) { // just prints
  byte x = 0x00;
  byte halt = false;
  char convertedValue; //to print as hex
//  byte convertedValue;
  while (!halt) {
    convertedValue = temp[x];
    
    //actual printing
    if (temp[x] == 0xFE) {
      xbee.print(convertedValue);
      halt = true;
    }
    else {
      xbee.print(convertedValue);
    }

    if (x != BUFFER_SIZE)
      x = x + 0x01;
    else {
      halt = true;
    }
  }
  xbee.println();
}

void sendPacketQueue() { //send entire packetqueue
  while (!packetQisEmpty) {  
    printBuffer(packetQueue[packetQueueHead]);
    packetQueueHead = (packetQueueHead + 0x01) % PACKET_QUEUE_SIZE; // increment head
    packetQisFull = false;
    if (packetQueueHead == packetQueueTail && (packetQisFull == false)) {
      packetQisEmpty = true;
    }
  }
  if (packetQisEmpty == true && requestConfig == true && attemptCounter == 0x00 && errorFlag == 0x00) {
    //attempt counter was added since you only need to restart it every time na restart and no error
    initializeTimer();
    xbee.println("initialize timer");
  }
}

void initializePacket(byte pQueue[]) { // one row //adds necessary stuff at init; needs one part row
  //  sinkAddress = 0x00;// testing
  pQueue[0] = 0xFF; // header
  pQueue[1] = physicalAddress; //source
  pQueue[2] = sinkAddress; //destination ; assumes always sink node
  pQueue[3] = 0x03; //api used to reply is currently 3
}

//insert count @ packet
//pQueue[4] = (packetCommandCounter - 0x01);
//if port data pinapadala
//reset count
//packetCommandCounter = 0x00; // reset counter
//packetPos = 0x05; // reset to put command

void closePacket(byte pQueue[]) {
  if (packetCommandCounter != 0x00) { //possibly sending portData
    pQueue[5] = packetCommandCounter-1; //to remove offset
  }
  
  pQueue[packetPos] = 0xFE; //footer
  packetPos = 0x06;
  packetCommandCounter = 0x00; //reset command counter
  packetQueueTail = (packetQueueTail + 0x01) % PACKET_QUEUE_SIZE; // point to next in queue
}

void insertToPacket(byte pQueue[], byte portNumber) { // for port data
  //command, count, port num, port value
  //adds count for each command
  //checks if equal to buffersize - 2 (coz may footer pa)
  pQueue[4] = 0x0F; //command
  packetQisEmpty = false; // kasi may laman na siya by that time
  
  if ((packetPos != (BUFFER_SIZE - 2)) && ((packetPos + 0x03) <= (BUFFER_SIZE - 2))) { //if not full and kapag nilagay mo dapat hindi mapupuno
//    packetPos = packetPos + 0x01;//packetPos starts at command's pos
    pQueue[packetPos] = portNumber;
    packetPos = packetPos + 0x01;
    pQueue[packetPos] = highByte(portValue[portNumber]);
    packetPos = packetPos + 0x01;
    pQueue[packetPos] = lowByte(portValue[portNumber]);
    packetCommandCounter = packetCommandCounter + 0x01; //increment count
    packetPos = packetPos + 0x01;
  }
  else { //puno na or hindi na kasya
    
    closePacket(packetQueue[packetQueueTail]);
    if (packetQueueHead != packetQueueTail) { //if queue is not full
      packetPos = 0x06;
      initializePacket(packetQueue[packetQueueTail]); //create new packet
      insertToPacket(packetQueue[packetQueueTail], portNumber);
//      packetPos = packetPos + 0x01;
    }
    else { //queue is full
      xbee.print("Tail:");
      xbee.println(packetQueueTail, HEX);
      xbee.print("Head:");
      xbee.println(packetQueueHead, HEX);
      xbee.println("Flush all");
      packetQisFull = true;
      sendPacketQueue(); //flush all
      initializePacket(packetQueue[packetQueueTail]); //create new packet
      insertToPacket(packetQueue[packetQueueTail], portNumber); // add new one
//      xbee.println("Double check");
//      printQueue(packetQueue, PACKET_QUEUE_SIZE);
    }
  }
}

void formatReplyPacket(byte pQueue[], byte command) { // format reply with command param only and tell that packet queue has something
  pQueue[4] = command;
  packetPos = 0x05;
//  printBuffer(packetQueue[packetQueueTail]);
  packetQisEmpty = false;
}

void loadConfig() { //loads config file and applies it to the registers
  byte fileCounter = 0x00;
  byte index = 0x00;
  byte hiByte = 0x00;
  byte loByte = 0x00;

  if (!SD.begin(CS_PIN)) { //in case sd card is not init
    strcpy_P(buffer, (char*)pgm_read_word(&(messages[25])));
    xbee.println(buffer); //error
    errorFlag |= 0x01;
  }
  else {
    File configFile = SD.open("conf.log");
    if (configFile) { // check if there is saved config
      while (configFile.available()) {
        int fileTemp = configFile.read();
        //reads the file per byte, in the case of the int (2 bytes) it is divided into 2 parts, hiByte and loByte.
        // then it is appended together

        if (fileCounter == 0x00) { //logical address
          logicalAddress = fileTemp;
          fileCounter = 0x01;
        }
        else if (fileCounter == 0x01) { //physical
          physicalAddress = fileTemp;
          fileCounter = 0x02;
        }
        else if (fileCounter == 0x02) { //sink address
          sinkAddress = fileTemp;
          fileCounter = 0x03;
        }
        else if (fileCounter == 0x03) { //config version
          configVersion = fileTemp;
          fileCounter = 0x04;
        }
        else if (fileCounter == 0x04) { //portconfigsegment(c)
          index = fileTemp;
          fileCounter = 0x05;
        }
        else if (fileCounter == 0x05) {
          portConfigSegment[index] = fileTemp;
//          xbee.print("config");
//          xbee.println(fileTemp,HEX);
//          xbee.println((fileTemp & 0x07), HEX);
          if ((fileTemp & 0x07) != 0x00){ //means config was changed
//            xbee.println("Setting configChange");
            configChangeRegister |= (1 << index); //to set for later
          }
          if (index != (PORT_COUNT - 1)) {
            fileCounter = 0x04;
          }
          else {
            fileCounter = 0x06;
          }
        }
        else if (fileCounter == 0x06) { //actuatorValueOnDemandSegment(c)
          index = fileTemp;
          fileCounter = 0x07;
        }
        else if (fileCounter == 0x07) {
          hiByte = fileTemp;
          fileCounter = 0x08;
        }
        else if (fileCounter == 0x08) {
          loByte = fileTemp;
          actuatorValueOnDemandSegment[index] = word(hiByte, loByte);
          if (index != (PORT_COUNT - 1)) {
            fileCounter = 0x06;
          }
          else {
            fileCounter = 0x09;
          }
        }
        else if (fileCounter == 0x09) { //portValue(c)
          index = fileTemp;
          fileCounter = 0x0A;
        }
        else if (fileCounter == 0x0A) {
          hiByte = fileTemp;
          fileCounter = 0x0B;
        }
        else if (fileCounter == 0x0B) {
          loByte = fileTemp;
          portValue[index] = word(hiByte, loByte);
          if (index != (PORT_COUNT - 1)) {
            fileCounter = 0x09;
          }
          else {
            fileCounter = 0x0C;
          }
        }
        else if (fileCounter == 0x0C) { //timerSegment(c)
          index = fileTemp;
          fileCounter = 0x0D;
        }
        else if (fileCounter == 0x0D) {
          hiByte = fileTemp;
          fileCounter = 0x0E;
        }
        else if (fileCounter == 0x0E) {
          loByte = fileTemp;
          timerSegment[index] = word(hiByte, loByte);
          if (index != (PORT_COUNT - 1)) {
            fileCounter = 0x0C;
          }
          else {
            fileCounter = 0x0F;
          }
        }
        else if (fileCounter == 0x0F) { //eventSegment(c*2)
          index = fileTemp;
          fileCounter = 0x10;
        }
        else if (fileCounter == 0x10) {
          hiByte = fileTemp;
          fileCounter = 0x11;
        }
        else if (fileCounter == 0x11) {
          loByte = fileTemp;
          eventSegment[index] = word(hiByte, loByte);
          if (index != ((PORT_COUNT * 2) - 1)) {
            fileCounter = 0x0F;
          }
          else {
            fileCounter = 0x12;
          }
        }
        else if (fileCounter == 0x12) { //actuatorDetailSegment(c)
          index = fileTemp;
          fileCounter = 0x13;
        }
        else if (fileCounter == 0x13) {
          hiByte = fileTemp;
          fileCounter = 0x14;
        }
        else if (fileCounter == 0x14) {
          loByte = fileTemp;
          actuatorDetailSegment[index] = word(hiByte, loByte);
          if (index != (PORT_COUNT - 1)) {
            fileCounter = 0x12;
          }
          else {
            fileCounter = 0x15;
          }
        }
        else if (fileCounter == 0x15) { //actuatorValueTimerSegment
          index = fileTemp;
          fileCounter = 0x16;
        }
        else if (fileCounter == 0x16) {
          hiByte = fileTemp;
          fileCounter = 0x17;
        }
        else if (fileCounter == 0x17) {
          loByte = fileTemp;
          actuatorValueTimerSegment[index] = word(hiByte, loByte);
          if (index != (PORT_COUNT - 1)) {
            fileCounter = 0x15;
          }
          else {
            fileCounter = 0x18; //reset
          }
        }
      }
      //      printRegisters();
      checkPortConfig(); // to apply it to flags
      // to inform sink node of successful loadup
      initializePacket(packetQueue[packetQueueHead]);
      formatReplyPacket(packetQueue[packetQueueHead], 0x0B);
      closePacket(packetQueue[packetQueueHead]);
      xbee.print("@ "); // to print entire packet later
      sendPacketQueue();
      digitalWrite(SUCCESS_LED_PIN, HIGH);
      //      printBuffer(packetQueue, PACKET_QUEUE_SIZE);

      configFile.close();
      initializeTimer();
    }
    else { //cannot access sd card/file not found

      //request config from sink node
      requestConfig = true;
      calculateOverflow(timeoutVal, PORT_COUNT);
      // last value of portOverflowCount array is for timeout
      xbee.print("@ "); // to print entire packet later
      initializePacket(packetQueue[packetQueueTail]);
      formatReplyPacket(packetQueue[packetQueueTail], 0x11);
      closePacket(packetQueue[packetQueueTail]);
      //      printQueue(packetQueue, PACKET_QUEUE_SIZE);
      xbee.println("Request");
      sendPacketQueue();
      //      while(requestConfig){
      checkTimeout();
      //      }
    }
  }
}

void writeConfig() { // writes node configuration to SD card
  //  if(SD.begin(CS_PIN)){
  if (SD.exists("conf.log")) { //otherwise will overwrite
    SD.remove("conf.log");
//    xbee.println("Creating new");
  }
  File configFile = SD.open("conf.log", FILE_WRITE);
//  xbee.println("writing");
  if (configFile) {
    configFile.write(logicalAddress);
    configFile.write(physicalAddress);
    configFile.write(sinkAddress);
    //      configFile.println(destAddress, HEX);
    configFile.write(configVersion);
    configFile.flush(); // pre save

    for (byte c = 0x00; c < PORT_COUNT; c++) {
      configFile.write(c);
      configFile.write(portConfigSegment[c]);
    }
    configFile.flush();
    for (byte c = 0x00; c < PORT_COUNT; c++) {
      configFile.write(c);
      configFile.write(highByte(actuatorValueOnDemandSegment[c]));
      configFile.write(lowByte(actuatorValueOnDemandSegment[c]));
    }
    configFile.flush();
    for (byte c = 0x00; c < PORT_COUNT; c++) {
      configFile.write(c);
      configFile.write(highByte(portValue[c]));
      configFile.write(lowByte(portValue[c]));
    }
    configFile.flush();
    for (byte c = 0x00; c < PORT_COUNT; c++) {
      configFile.write(c);
      configFile.write(highByte(timerSegment[c]));
      configFile.write(lowByte(timerSegment[c]));
    }
    configFile.flush();
    for (byte c = 0x00; c < PORT_COUNT * 2; c++) {
      configFile.write(c);
      configFile.write(highByte(eventSegment[c]));
      configFile.write(lowByte(eventSegment[c]));
    }
    configFile.flush();
    for (byte c = 0x00; c < PORT_COUNT; c++) {
      configFile.write(c);
      configFile.write(highByte(actuatorDetailSegment[c]));
      configFile.write(lowByte(actuatorDetailSegment[c]));
    }
    configFile.flush();
    for (byte c = 0x00; c < PORT_COUNT; c++) {
      configFile.write(c);
      configFile.write(highByte(actuatorValueTimerSegment[c]));
      configFile.write(lowByte(actuatorValueTimerSegment[c]));
      //      xbee.println(lowByte(actuatorValueTimerSegment[c]),HEX);
    }
    configFile.close();
//    xbee.println("Writing finish");
    //      printRegisters();
  }
  else {
    //      strcpy_P(buffer, (char*)pgm_read_word(&(messages[26])));
    //      xbee.println(buffer); //error
    byte temp = 0x08;
    errorFlag |= 0x08; //cannot access sd card or file not found
//    xbee.println(errorFlag, HEX);
  }
  //  }
  //  else{
  //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[25])));
  //    xbee.print(buffer);
  //  }

}

void manipulatePortData(byte index, byte configType) { //checks port type, actuates and senses accordingly 
  //configType is to know where to get actuator value (can be hardcoded but you can get it from portconfigsegment din)
  //it checks the port config of index to indicate if sensor or actuator
  unsigned int actuatorValue = 0x0000;

  if ((portConfigSegment[index] & 0x08) == 0x08) { //actuator
//    xbee.println("Actuator!");
    if (configType == 0x00) { // time
      actuatorValue = actuatorValueTimerSegment[index];
    }
    else if (configType == 0x01) { // event
      actuatorValue = actuatorDetailSegment[index];
    }
    else if (configType == 0x02) { //odm
      actuatorValue = actuatorValueOnDemandSegment[index];
    }
    else if (configType == 0x03){ //just probing
      actuatorValue = 0;
    }
    
    actuatorValue = actuatorValue & 0x0FFF; // get only details
//    xbee.print("wr");
//    xbee.println(actuatorValue);
//    xbee.print("actuatorValue: ");
//    xbee.println(actuatorValue, HEX);
//    xbee.print("index: ");
//    xbee.println(index,HEX);

    if (index < 0x06) { // digital actuator
      if (actuatorValue == 0x00) {
        digitalWrite(index + 0x04, LOW); //port 0 is at pin 4
      }
      else if (actuatorValue > 0x00) {
        digitalWrite(index + 0x04, HIGH);
      }
      portValue[index] = digitalRead(index + 0x04);
//      xbee.print("Port VALUE");
//      xbee.println(portValue[index],HEX);
    }
    else if (index >= 0x06) { //analog actuator inserted on A0-A5
      xbee.println("Analog");
//      xbee.print("Actuator Value: ");
      actuatorValue = bcdToDecimal(actuatorValue);
//      xbee.println(actuatorValue); //read as hex
      //convert to int
      
      if(index == 0x06){ // A0 - 14 >>> 6 
        servo0.write(actuatorValue); //takes int
        portValue[index] = servo0.read();
      }
      else if(index == 0x07){ // A1 - 15 >>> 7
        servo1.write(actuatorValue);
        portValue[index] = servo1.read();
      }
      else if(index == 0x08){ // A2 - 16 >>>  8
        servo2.write(actuatorValue);
        portValue[index] = servo2.read();
      }
      else if(index == 0x09){ // A3 - 17 >>>  9
        portValue[index] = digitalRead(17);
      }
      else if(index == 0x0A){ // A4 - 18 >>> A
        portValue[index] = digitalRead(18);
      }
      else if(index == 0x0B){ // A5 - 19 >>>  B
        portValue[index] = digitalRead(19);
      }

      xbee.print("Index: ");
      xbee.println(index, HEX);
      xbee.print("Port VALUE");
      xbee.println(portValue[index],HEX);
    }
  }
  else { //sensor
//    xbee.println("Sensor @ manip!");
    if (index < 0x06) { //Digital sensor
      portValue[index] = digitalRead(index + 0x04);
    }
    else if (index >= 0x06) { // analog sensor
      int portVal;
      long maxx = 999;
      long diver = 1023;
      
      if(index == 0x06){
        portValue[index] = servo0.read(); //0 - 180 degrees
      }
      else if(index == 0x07){
        portValue[index] = servo1.read(); //0 - 180 degrees
      }
      else if(index == 0x08){
         portValue[index] = servo2.read(); //0 - 180 degrees
      }
      else if(index == 0x09){
        portVal = analogRead(A3);
        xbee.print("...");
        xbee.println(portVal);
//        portValue[index] = constrain(portVal, 0, 999); //constrains values para within 0-999 (ng bcd)
        portValue[index] = (portVal * maxx) / diver;
        xbee.println(portValue[index]);
      }
      else if(index == 0x0A){
        portVal = analogRead(A4);
        portValue[index] = (portVal * maxx) / diver;
      }
      else if(index == 0x0B){
        portVal = analogRead(A5);
        portValue[index] = (portVal * maxx) / diver;
      }
    }
  }
  if(configType != 0x03){
    portDataChanged |= (1 << index); //set port data changed
  }
}

void convertEventDetailsToDecimal(byte portNum) { // from bcd to decimal
  unsigned int temp = eventSegment[portNum];
//  xbee.print("Port #: ");
//  xbee.println(portNum, HEX);
//  xbee.print("Event: ");
//  xbee.println(temp, HEX);
//  printRegisters();
  if ((temp & 0x8000) == 0x8000) { //range mode
//    xbee.println((temp & 0x0FFF), HEX);
    temp = (temp & 0x0FFF); // get data only
    convertedEventSegment[portNum] = bcdToDecimal(temp);
    temp = eventSegment[portNum + 0x0C]; //next part
    temp = (temp & 0x0FFF);
    convertedEventSegment[portNum + 0x0C] = bcdToDecimal(temp);
    xbee.println("@ Ra");
    xbee.println(convertedEventSegment[portNum]);
    xbee.println(convertedEventSegment[portNum + 0x0C]);
  }
  else { //threshold
//    xbee.println("@ Th ");
    temp = (temp & 0x0FFF);
    
    convertedEventSegment[portNum] = bcdToDecimal(temp); //converts it to decimal
    
//    xbee.print("cONVERTED:");
//    xbee.println(temp, HEX);
//    xbee.println(convertedEventSegment[portNum]);
  }

}

boolean checkEventCondition(byte eventCondition, int tempPortValue,int eventValue) {
  byte conditionReached = false;
//  xbee.println("@ checkevent");
//  xbee.print("Portvalue: ");
//  xbee.println(tempPortValue);
//  xbee.print("eventvalue: ");
//  xbee.println(eventValue);

  if (eventCondition == 0x00) { //less than not equal
    // portData < eventValue
      if (tempPortValue < eventValue) {
        conditionReached = true;
      }
//      xbee.println("<");
  }
  else if (eventCondition == 0x01) { // less than equal
      //portData <= eventValue
      if (tempPortValue <= eventValue) {
        conditionReached = true;
      }
//      xbee.println("<=");
  }
  else if (eventCondition == 0x02) { // greater than not equal
      // portData > eventValue
      if (tempPortValue > eventValue) {
        conditionReached = true;
      }
//      xbee.println(">");
  }
  else if (eventCondition == 0x03) { // greater than equal
      // portData >= eventValue
      if (tempPortValue >= eventValue) {
        conditionReached = true;
      }
//      xbee.println(">=");
  }
//  xbee.print("Read port val: ");
//  xbee.println(tempPortValue);
//  xbee.print("Event Condition: ");
//  xbee.println(eventCondition, HEX);
//  xbee.print("Event Value: ");
//  xbee.println(eventValue);
//  xbee.print("Condition Reached: ");
//  xbee.println(conditionReached);
  return conditionReached;
}

boolean checkPortConfig() { //checks saved config per pin (after being retrieved from serial queue)
  unsigned int actuatorValue;
  byte configCheck = 0x00; // stores which bit is being checked
  byte configType;
  boolean applyConfig = false;


  for (byte x = 0x00; x < PORT_COUNT; x++) {
    unsigned int bitMask = (1 << x);
    //    xbee.println(bitMask, HEX);
    //    xbee.println(configChangeRegister, HEX);
    //    xbee.println(configChangeRegister & bitMask, HEX);
    if ((configChangeRegister & bitMask) == bitMask) { // config was changed
      byte temp = portConfigSegment[x];
//      xbee.print("full port config");
//      xbee.println(temp, HEX);
      configType = temp & 0x07; // get all config
      
      while (configCheck != 0x03) { //checks if config is sent per pin
        byte checker = (configType & (1 << configCheck));
        if (checker == 0x01) { // time based
//          xbee.println("@ time!");
          //xbee.print("timerSeg: ");
          //xbee.println(timerSegment[portNum], HEX);
          calculateOverflow(timerSegment[x], x);
//          xbee.println(portOverflowCount[x]);
          timerRequest |= (1 << x); // sets timer request
          applyConfig = true;
          //xbee.println(timerRequest, HEX);
          timerReset = true;
          configChangeRegister = configChangeRegister & ~bitMask; //turns off config changed flag
        }
        else if (checker == 0x02) { // event
//          xbee.println("@ event check port config");
//          convertEventDetailsToDecimal(x);
          eventRequest |= (1 << x); //set event request
          applyConfig = true;
          configChangeRegister = configChangeRegister & ~bitMask; //turns off config changed flag
          //xbee.println(eventRequest, HEX);
        }
        else if (checker == 0x04) { // odm
//          xbee.println("@ odm");
          manipulatePortData(x, 0x02); // odm type
          applyConfig = true;


          //          commented the following block kasi hindi mo need yun
          //          xbee.print("Port Config: ");
          //          xbee.println(portConfigSegment[x], HEX);
          //          portConfigSegment[x] = portConfigSegment[x] & 0xFB; // turn off odm at port config
          //          xbee.print("Updated Port Config: ");
          //          xbee.println(portConfigSegment[x], HEX);
          portDataChanged |= (1 << x); //inform that it has been updated
//          xbee.print("PortDataChange: ");
//          xbee.println(portDataChanged, HEX);
          configChangeRegister = configChangeRegister & ~bitMask; //turns off config changed flag
//          xbee.print("configChangeRegister: ");
//          xbee.println(configChangeRegister, HEX);
        }
        configCheck = configCheck + 0x01;
      }
      configCheck = 0x00; // reset again
    }
    else { // config was not changed
//      xbee.println("skipped");
    }
  }
//  xbee.print("End of port config flag: ");
//  xbee.println(configChangeRegister, HEX); //dapat 0
  return applyConfig;
}
/************* Utilities *************/

void printRegisters() { // prints all variables stored in the sd card
  //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[27])));
  //    xbee.print(buffer);
  //    xbee.println(logicalAddress,HEX);
  //
  //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[28])));
  //    xbee.print(buffer);
  //    xbee.println(physicalAddress,HEX);
  //
  //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[29])));
  //    xbee.print(buffer);
  //    xbee.println(configVersion,HEX);

  for (byte x = 0x00; x < PORT_COUNT; x++) {
    strcpy_P(buffer, (char*)pgm_read_word(&(messages[19])));
    xbee.print(buffer);
    xbee.println(portConfigSegment[x], HEX);
  }

//  for (byte x = 0x00; x < PORT_COUNT; x++) {
//    strcpy_P(buffer, (char*)pgm_read_word(&(messages[35])));
//    xbee.print(buffer);
//    xbee.println(actuatorValueTimerSegment[x], HEX);
//  }

  //  for (byte x = 0x00; x < PORT_COUNT; x++) {
  //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[30])));
  //    xbee.print(buffer);
  //    xbee.println(actuatorValueOnDemandSegment[x], HEX);
  //  }
  //
  //    for(byte x = 0x00; x <PORT_COUNT; x++){ //
  //      strcpy_P(buffer, (char*)pgm_read_word(&(messages[31])));
  //      xbee.print(buffer);
  //      xbee.println(portValue[x]);
  //    }
  //
  //  for (byte x = 0x00; x < PORT_COUNT; x++) {
  //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[22])));
  //    xbee.print(buffer);
  //    xbee.println(timerSegment[x], HEX);
  //  }
  //
//    for (byte x = 0x00; x < PORT_COUNT * 2; x++) {
//      strcpy_P(buffer, (char*)pgm_read_word(&(messages[24])));
//      xbee.print(buffer);
//      xbee.println(eventSegment[x], HEX);
//    }
//  
//    for (byte x = 0x00; x < PORT_COUNT; x++) {
//      strcpy_P(buffer, (char*)pgm_read_word(&(messages[15])));
//      xbee.print(buffer);
//      xbee.println(actuatorDetailSegment[x], HEX);
//    }
}

void checkPortModesSent() { // checks the configs of the ports then set segmentCounter accordingly, expecting the next parameters
  if ((tempModeStorage & 0x01) == 0x01) { //if timebased is set
    strcpy_P(buffer, (char*)pgm_read_word(&(messages[4])));
    xbee.println(buffer);
    segmentCounter = 0x07;
  }
  else if ((tempModeStorage & 0x02) == 0x02) { // event
    strcpy_P(buffer, (char*)pgm_read_word(&(messages[5])));
    xbee.println(buffer);
    segmentCounter = 0x08;
  }
  else if ((tempModeStorage & 0x04) == 0x04) { // on demand
    strcpy_P(buffer, (char*)pgm_read_word(&(messages[6])));
    xbee.println(buffer);
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
    //    xbee.print(buffer);
    //    xbee.println(portNum, HEX);

    byte temp = portConfigSegment[portNum];
    //Checking bit 3 if it is actuator or sensor
    if ((temp & 0x08) == 0x08) { //actuator
      segmentCounter = 0X0D; // get actuator segment
      strcpy_P(buffer, (char*)pgm_read_word(&(messages[33])));
      xbee.print(buffer);
    }
    else if ((temp & 0x08) == 0x00) { // sensor
      strcpy_P(buffer, (char*)pgm_read_word(&(messages[32])));
      xbee.print(buffer);
      tempModeStorage = tempModeStorage ^ 0x04; // switch off odm flag;  assumes dapat 0 na value ni tempMode Storage
      checkOtherCommands(); // check if there are still other configurations
    }
  }
  else { // invalid port config
    strcpy_P(buffer, (char*)pgm_read_word(&(messages[11])));
    xbee.println(buffer);
    segmentCounter = 0x00;
    tempModeStorage = 0x00;
  }
}

void checkOtherCommands() { // checks if there is still commands
  if (commandCounter < commandCount) {
    commandCounter = commandCounter + 1;
    segmentCounter = 0x06; //to port config segment
    strcpy_P(buffer, (char*)pgm_read_word(&(messages[17])));
    xbee.println(buffer);
  }
  else {
    segmentCounter = 0xFF; // go to footer
//    strcpy_P(buffer, (char*)pgm_read_word(&(messages[18])));
//    xbee.println(buffer);
  }
}

void retrieveSerialQueue(byte queue[], byte head) { // read from queue and store to variables
  byte x = 0x00;
  byte halt = false;
  segmentCounter = 0x00;
//  printBuffer(serialQueue[serialTail]);
  //  for(byte x = 0x00; x < BUFFER_SIZE; x++){
  while (!halt) {
    //      xbee.print("SGM CTR: ");
    //      xbee.println(segmentCounter, HEX);
    byte data = queue[x];
//     xbee.print("CTR: ");
//      xbee.println(segmentCounter, HEX);
//    xbee.print(data , HEX);
    
    if (data == 0xFF && segmentCounter == 0x00) {
      segmentCounter = 0x01;
    }
    else if (segmentCounter == 0x01) { //SOURCE
     
      sourceAddress = data;
      segmentCounter = 0x02;
    }
    else if (segmentCounter == 0x02) { //DESTINATION
      if (data != physicalAddress) { //in case di pala para sa kanya; kasi broadcast mode si xbee; technically drop ze packet
        segmentCounter = 0x00;
      }
      else {
        segmentCounter = 0x03; //I think.. ilagay nalang yung physical address ni node mismo
        destAddress = data;
      }
    }
    else if (segmentCounter == 0x03) { //API
      apiCount = data;
      if (apiCount == 2) { // if CONFIG mode, else iba na ieexpect niya na mga kasama na segment
        segmentCounter = 0x04; // check config version
      }
      else if (apiCount == 3) { // if Node Discovery Mode, expects  command parameter
        segmentCounter = 0x0B; //command parameter
      }
      else { //pero technically dapat error na ito
        segmentCounter = 0x05; // go straight to count ; for debug purpose lang
      }
    }
    else if (segmentCounter == 0x04) { // CONFIG VERSION IF API = 2
      //sets packet type to node config
      packetTypeFlag |= 0x02;

      if (configVersion <= data) { // if newer yung config version
        configVersion = data; //take it
        segmentCounter = 0x05; // check count
      }
      else { //Current node config is most recent
        segmentCounter = 0x00;
      }
    }
    else if (segmentCounter == 0x05) { // COUNT
      if (data < MAX_COMMAND) { // if not greater than max command count per packet :D
        commandCount = data;
        segmentCounter = 0x06; //get port configuration
      }
      else { // More than maximum commands that node can handle
        segmentCounter = 0x00;
      }
    }
    else if (segmentCounter == 0x06) { // PORT CONFIGURATION SEGMENT
      //        strcpy_P(buffer, (char*)pgm_read_word(&(messages[0])));
      //        xbee.println(buffer);
      xbee.print("Port config: ");
      xbee.println(data, HEX);
      portNum = 0xF0 & data; // to get port number; @ upper byte
      portNum = portNum >> 4; // move it to the right
//      xbee.print("Port NUM: ");
//      xbee.println(portNum, HEX);
      configChangeRegister |= (1 << portNum); // to inform which ports was changed
      
      setPortConfigSegment(portNum, data); // stored to port config segment
      tempModeStorage = data & 0x07; // stores the modes sent; @ lower byte
//      xbee.print("tempModeStorage: ");
//      xbee.println(tempModeStorage, HEX);

      checkPortModesSent(); // checks modes sent serving timer > event > odm
    }
    else if (segmentCounter == 0x07) { // TIME SEGMENT
      //        strcpy_P(buffer, (char*)pgm_read_word(&(messages[1])));
      //        xbee.println(buffer);
      setTimerSegment(portNum, data);
      if (partCounter == 0x01) { //next part of the time is found
        //Checking bit 3 if it is actuator or sensor
        byte temp = portConfigSegment[portNum];
        //xbee.print("Stored time: ");
        //xbee.println(timerSegment[portNum], HEX);
        if ((temp & 0x08) == 0x08) { //actuator
          segmentCounter = 0x17; // get actuator segment
          partCounter = partCounter ^ partCounter;
          strcpy_P(buffer, (char*)pgm_read_word(&(messages[33])));
          xbee.print(buffer);
        }
        else if ((temp & 0x08) == 0x00) { // sensor
          strcpy_P(buffer, (char*)pgm_read_word(&(messages[32])));
          xbee.print(buffer);
          tempModeStorage = tempModeStorage ^ 0x01; // xor to turn off time base flag
          partCounter = 0x00; //reset part counter
          if (tempModeStorage != 0) { //may iba pang modes; hanapin natin
            checkPortModesSent();
          }
          else { //kung time based lang yung port na yun
            checkOtherCommands(); // check if there are still other configurations
          }
        }

        //          tempModeStorage = tempModeStorage ^ 0x01; // xor to turn off time base flag
        //          partCounter = 0x00; //reset part counter
        //          if(tempModeStorage!=0) { //may iba pang modes; hanapin natin
        //            checkPortModesSent();
        //          }
        //          else{ //kung time based lang yung port na yun
        //            checkOtherCommands(); // check if there are still other configurations
        //          }
      }
      else { // find next part of time
        partCounter = partCounter + 0x01;
      }
    }
    else if (segmentCounter == 0x08) { // EVENT SEGMENT
      setEventSegment(portNum, data);
      if (partCounter == 0x00) {
        checker = data & 0x80; // if 0x80 then, range mode else threshold mode
        partCounter = partCounter | 0x01; // increment to get next part
      }
      else if (partCounter == 0x01) { //partCounter == 0x01
        if (checker == 0x80) {
          //            strcpy_P(buffer, (char*)pgm_read_word(&(messages[12])));
          //            xbee.println(buffer);
          segmentCounter = 0x09; // next range value
          partCounter = 0x00;
          checker = 0x00;
        }
        else {
//          strcpy_P(buffer, (char*)pgm_read_word(&(messages[13])));
//          xbee.println(buffer);
          segmentCounter = 0x0A; // threshold mode; one value only
          partCounter = 0x00;
        }
      }
    }
    else if (segmentCounter == 0x09) { // RANGE MODE (EVENT MODE) SECOND VALUE
      setRangeSegment(portNum, data);
      if (partCounter == 0x00) {
        partCounter = partCounter | 0x01;
      }
      else if (partCounter == 0x01) {
        segmentCounter = 0x0A; // to actuator details
        partCounter = 0x00; // reset part counter
        checker = 0x00; // coz it was not reset
      }
    }
    else if (segmentCounter == 0x0A) { // ACTUATOR DETAILS
      setActuatorDetailSegment(portNum, data); 
      if (partCounter == 0x01) { // next part of actuator detail segment is found
        partCounter = 0x00; //reset part counter
        checker = 0x00;
        tempModeStorage = tempModeStorage ^ 0x02; // xor to turn off event flag
//        xbee.print("Saved");
//        xbee.println(actuatorDetailSegment[portNum], HEX);
        if (tempModeStorage != 0) { //may iba pang mode, most likely odm
          checkPortModesSent();
        }
        else { //kung event triggered lang yung port na yun
          checkOtherCommands(); // check if there are still other configurations
        }
      }
      else {
        partCounter = 0x01; // increment part counter
      }
    }
    else if (segmentCounter == 0x0B) { //COMMAND PARAMETER FOR API == 3
      if (data == 0x00) { // Request keep alive message
        commandValue = 0x00;
        packetTypeFlag |= 0x04; //set packet type to node discovery
        segmentCounter = 0x16; // node discovery check
      }
      if (data == 0x02) { //Network Config
        commandValue = 0x02;
        packetTypeFlag |= 0x04; //set packet type to node discovery
        segmentCounter = 0x0E; // go to logical address
      }
      if (data == 0x0E) { // receive configuration
        packetTypeFlag |= 0x01; //set packet type to a startup config
        commandValue = 0x0E; //sets 1st bit to indicate a config is coming
        segmentCounter = 0x0C; //check how many parts
      }
    }
    else if (segmentCounter == 0x0C) { //part checker
      configSentPartCtr = data;

      if (data == 0x00) {
        segmentCounter = 0x15; //actuator detail
      }
      else if (data == 0x01) {
        segmentCounter = 0x14;//event segment
      }
      else if (data == 0x02) {
        segmentCounter = 0x13; //timer segment
      }
      else if (data == 0x03) {
        segmentCounter = 0x18; // actuator value timer
      }
      else if (data == 0x04) {
        segmentCounter = 0x0E; //logical to actuator value on demand
      }
      
    }
    else if (segmentCounter == 0x0D) { // ODM - ACTUATOR SEGMENT
      setActuatorValueOnDemandSegment(portNum, data);

      if (partCounter == 0x01) { // if last part
        tempModeStorage = tempModeStorage ^ 0x04; // switch off odm flag;  assumes dapat 0 na value ni tempMode Storage
        partCounter = partCounter ^ partCounter; // reset part counter
        checkOtherCommands(); // check if there are still other configurations
      }
      else {
        partCounter = partCounter | 0x01;
      }
    }
    else if (segmentCounter == 0x0E) { //API == 3 GET CONFIG - logical address
      logicalAddress  = data;
      if (packetTypeFlag && 0x04 == 0x04) { // if node discovery - network config, go to sink node addr
        segmentCounter = 0x16;
      }
      else { // if requesting config
        segmentCounter = 0x10;
      }
    }
    else if (segmentCounter == 0x10) { //GET CONFIG VERSION
      configVersion = data;
      segmentCounter = 0x11;
    }
    else if (segmentCounter == 0x11) { //PORT CONFIG
      setPortConfigSegment(checker, data);
      partCounter = 0x00;
      if (checker != PORT_COUNT - 1) {
        checker = checker + 0x01;
      }
      else {
        segmentCounter = 0x12;
        checker = 0x00;
        partCounter = partCounter ^ partCounter; // reset part counter
      }
    }
    else if (segmentCounter == 0x12) { // PORT CONFIG - ACUATOR VALUE ON DEMAND
      setActuatorValueOnDemandSegment(checker, data);
      if (partCounter == 0x01) { // if last part ng data
        if (checker != PORT_COUNT - 1) {
          checker = checker + 0x01; // next port
          partCounter = partCounter ^ partCounter; // reset data
        }
        else {
          segmentCounter = 0xFF; // Footer
          checker = checker ^ checker; // clear port
          partCounter = partCounter ^ partCounter; //reset
        }
      }
      else {
        partCounter = partCounter | 0x01; // move to next
      }
    }
    else if (segmentCounter == 0x13) { //PORT CONFIG - TIMER SEGMENT
      setTimerSegment(checker, data);
      if (partCounter == 0x01) {
        if (checker != PORT_COUNT - 1) {
          checker = checker + 0x01; // next port
          partCounter = partCounter ^ partCounter; // reset data
        }
        else {
          segmentCounter = 0xFF; // Footer
          checker = checker ^ checker; // clear port
          partCounter = partCounter ^ partCounter; //reset
        }
      }
      else {
        partCounter = partCounter | 0x01; // move to next
      }
    }
    else if (segmentCounter == 0x14) { // PORT CONFIG - EVENT SEGMENT
      //        xbee.print("Serial Data: ");
      //        xbee.println(data,HEX);
      setEventSegment(checker, data);
      if (partCounter == 0x01) {
        if (checker != ((PORT_COUNT * 0x02) - 0x01)) {
          checker = checker + 0x01; // next port
          partCounter = partCounter ^ partCounter; // reset data
        }
        else {
          segmentCounter = 0xFF; // go to footer
          checker = checker ^ checker; // clear port
          partCounter = partCounter ^ partCounter; //reset
        }
      }
      else {
        partCounter = partCounter | 0x01;
      }
    }
    else if (segmentCounter == 0x15) { // PORT CONFIG - ACTUATOR DETAIL
      //        strcpy_P(buffer, (char*)pgm_read_word(&(messages[3])));
      //        xbee.println(buffer);
      setActuatorDetailSegment(checker, data);
      if (partCounter == 0x01) {
        if (checker != PORT_COUNT - 1) {
          checker++; // next port
          partCounter = partCounter ^ partCounter; // reset data
        }
        else {
          segmentCounter = 0xFF; // go to footers
          checker = checker ^ checker; // clear port
          partCounter = partCounter ^ partCounter; //reset
        }
      }
      else {
        partCounter = partCounter | 0x01; // move to next
      }
    }
    else if (segmentCounter == 0x16) { // NODE DISCOVERY - NETWORK CONFIGURATION - SINK ADDR
      if (commandValue == 0x02) { //network config
        sinkAddress = data; //change sink node addr
      }
      else if (commandValue == 0x00) { //keep alive
        //retain
      }
      segmentCounter = 0xFF;
    }
    else if (segmentCounter == 0x17) { // TIMER - ACTUATOR SEGMENT
//      xbee.print("PC: ");
//      xbee.println(partCounter, HEX);
      setActuatorValueTimerSegment(portNum, data);

      if (partCounter == 0x01) { // if last part
        tempModeStorage = tempModeStorage ^ 0x01; // switch off time base flag
        partCounter = partCounter ^ partCounter; // reset part counter
        if (tempModeStorage != 0) { //may iba pang modes; hanapin natin
          checkPortModesSent();
        }
        else { //kung time based lang yung port na yun
          checkOtherCommands(); // check if there are still other configurations
        }
      }
      else {
        partCounter = partCounter | 0x01;
      }
    }
    else if (segmentCounter == 0x18) { // PORT CONFIG - ACTUATOR VALUE TIMER
      setActuatorValueTimerSegment(checker, data);
      if (partCounter == 0x01) {
        if (checker != (PORT_COUNT - 1)) {
          checker = checker + 0x01; // next port
          partCounter = partCounter ^ partCounter; // reset data
        }
        else {
          segmentCounter = 0xFF; // go to footer
          checker = checker ^ checker; // clear port
          partCounter = partCounter ^ partCounter; //reset
        }
      }
      else {
        partCounter = partCounter | 0x01;
      }
    }
    else if (segmentCounter == 0xFF && data == 0xFE) { // FOOTER
      strcpy_P(buffer, (char*)pgm_read_word(&(messages[16])));
      xbee.println(buffer);
      segmentCounter = 0x00; //reset to check next packet
      commandCounter = 0x00;
      tempModeStorage = 0x00;
      checker = 00;
      partCounter = 00;
      portNum = 00;
      //      if (apiCount != 0x03) { //all other apis except if api is 3, then ctr has to be 3
      //        writeConfig(); // saves configuration to SD card; therefore kapag hindi complete yung packet, hindi siya saved ^^v
      //      }
      //        printRegisters(); // prints all to double check
    }

    if ((x == BUFFER_SIZE) || (segmentCounter == 0xFF && data == 0xFE)){
      //max buffer or footer is found
      halt = true;
    }
    else {
      x = x + 0x01;
    }

  }
  //  }
}

/************ Setters  *************/

void setPortConfigSegmentInit(byte portNum, char portDetails) { //when initializing //CHANGED FROM INT
  byte tempPort = 0x00;
  tempPort = portNum << 4; // to move it pin position
  tempPort = tempPort | portDetails; // to set the 4th pin to output
  portConfigSegment[portNum] = tempPort;
}

void setPortConfigSegment(byte portNum, byte portDetails) {
  //  strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
  //  xbee.print(buffer);
  //  xbee.println(portNum, HEX);
  //  strcpy_P(buffer, (char*)pgm_read_word(&(messages[19])));
  //  xbee.print(buffer);
  //  xbee.println(portDetails, HEX);
  portConfigSegment[portNum] = portDetails;
}

void setPortValue(int portNum, int val) {
  portValue[portNum] = val;
}

void setTimerSegment(byte portNum, int val) { // BYTE FROM INT; partCounter is a global var
  if (partCounter == 0x00) {
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
    //    xbee.print(buffer);
    //    xbee.println(portNum);
    timerSegment[portNum] = val;
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[20])));
    //    xbee.print(buffer);
    //    xbee.println(timerSegment[portNum], HEX);
    timerSegment[portNum] = timerSegment[portNum] << 8;
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[21])));
    //    xbee.print(buffer);
    //    xbee.println(timerSegment[portNum], HEX);
  }
  else if (partCounter == 0x01) {
    int temp = val;
    timerSegment[portNum] = timerSegment[portNum] | val;
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[22])));
    //    xbee.print(buffer);
    //    xbee.println(timerSegment[portNum], HEX);
  }
}

void setEventSegment(byte portNum, int val) { //BYTE FROM INT
  //  strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
  //  xbee.print(buffer);
  //  xbee.println(portNum);
  if (partCounter == 0x00) {
    eventSegment[portNum] = val << 8;
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[23])));
    //    xbee.print(buffer);
    //    xbee.println(eventSegment[portNum], HEX);
  }
  else if ((partCounter & 0x01) == 0x01) {
    eventSegment[portNum] = eventSegment[portNum] | val;
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[24])));
    //    xbee.print(buffer);
    //    xbee.println(eventSegment[portNum], HEX);
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
    //    xbee.print(buffer);
    //    xbee.println(portNum,HEX);
  }
}

void setRangeSegment(byte portNum, int val) { // for range type of events //BYTE FROM INT
  //  strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
  //  xbee.print(buffer);
  //  xbee.println(portNum);
  if (partCounter == 0x00) {
    //    xbee.print("Initial Value: ");
    //    xbee.println(eventSegment[portNum+0x0C], HEX);
    eventSegment[portNum + 0x0C] = val << 8;
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[23])));
    //    xbee.print(buffer);
    //    xbee.println(eventSegment[portNum+0x0C], HEX);
  }
  else if ((partCounter & 0x01) == 0x01) {
    eventSegment[portNum + 0x0C] = eventSegment[portNum + 0x0C] | val;
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[23])));
    //    xbee.print(buffer);
    //    xbee.println(eventSegment[portNum+0x0C], HEX);
    //    xbee.print("Index in Array: ");
    //    xbee.println(portNum+0x0C,HEX);
  }
}

void setActuatorDetailSegment(byte portNum, int val) {
  if (partCounter == 0x00) { // store first part
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[15]))); // Actuator Segment:
    //    xbee.print(buffer);
    //    xbee.println(actuatorDetailSegment[portNum], HEX);
    actuatorDetailSegment[portNum] = val << 8 ;
    //    xbee.print("Updated Upper Value: ");
    //    xbee.println(actuatorDetailSegment[portNum],HEX);
  }
  else if (partCounter == 0x01) {
    //    xbee.println("LOWER ACTUATOR DETAIL");
    //    xbee.print("Serial Data: ");
    //    xbee.println(val, HEX);
    actuatorDetailSegment[portNum] = actuatorDetailSegment[portNum] | val;
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[15]))); // Actuator Segment:
    //    xbee.print(buffer);
    //    xbee.println(actuatorDetailSegment[portNum], HEX);
  }

}

void setActuatorValueOnDemandSegment(byte portNum, int val) {
  if (partCounter == 0x00) { // get port number and store first part
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
    //    xbee.print(buffer);
    //    xbee.println(portNum, HEX);
    actuatorValueOnDemandSegment[portNum] = val << 8 ;
    //    xbee.print("Updated Upper Value: ");
    //    xbee.println(actuatorValueOnDemandSegment[portNum],HEX);
  }
  else if (partCounter == 0x01) {
    //    xbee.println("LOWER ACTUATOR VALUE ON DEMAND DETAIL");
    //    xbee.print("Serial Data: ");
    //    xbee.println(val, HEX);
    actuatorValueOnDemandSegment[portNum] = actuatorValueOnDemandSegment[portNum] | val;
    //    xbee.print("Full Actuator value Segment : ");
    //    xbee.println(actuatorValueOnDemandSegment[portNum], HEX);
    strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
    //    xbee.print(buffer);
    //    xbee.println(portNum,HEX);
  }
}

void setActuatorValueTimerSegment(byte portNum, int val) {
  if (partCounter == 0x00) { // get port number and store first part
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
    //    xbee.print(buffer);
    //    xbee.println(portNum, HEX);
    actuatorValueTimerSegment[portNum] = val << 8 ;
    //    xbee.print("Updated Upper Value: ");
    //    xbee.println(actuatorValueTimerSegment[portNum], HEX);
  }
  else if (partCounter == 0x01) {
    //    xbee.println("LOWER ACTUATOR VALUE TIMER DETAIL");
    //    xbee.print("Serial Data: ");
    //    xbee.println(val, HEX);
    actuatorValueTimerSegment[portNum] = actuatorValueTimerSegment[portNum] | val;
    //    xbee.print("Full Actuator value Segment : ");
    //    xbee.println(actuatorValueTimerSegment[portNum], HEX);
    //    strcpy_P(buffer, (char*)pgm_read_word(&(messages[7])));
    //    xbee.print(buffer);
    //    xbee.println(portNum,HEX);
  }
}

void calculateOverflow(unsigned int tempTime, byte portOverflowIndex) {
  //convert time to seconds store to realTime
  //xbee.print("Time: ");
  //xbee.println(tempTime, HEX);
  byte timeUnit = tempTime >> 12; // checks which unit
  unsigned int timeKeeper = tempTime & (0x0FFF); // get time only masking it
  //convert bcd to dec
  long timeTemp = bcdToDecimal(timeKeeper);
  //xbee.print("timeUnit: ");
  //xbee.println(timeUnit, HEX);
  //xbee.print("timeKeeper: ");
  //xbee.println(timeKeeper,HEX);
  long realTime = 0x00; //32 bits ; max is 24hrs 86400 seconds
  float timeMS = 0.000;

  //CONVERTS TO SECONDS
  if (timeUnit == 0x01 ) { //ms
    timeMS = timeTemp / 1000.0;
    //    xbee.println(timeMS,3); //0.99
  }
  else if (timeUnit == 0x02) { //sec
    //it is as is
  }
  else if (timeUnit == 0x04) { //min
    timeTemp = timeTemp * 60;
  }
  else if (timeUnit == 0x08) { //hour
    timeTemp = timeTemp * 3600;
  }

  realTime = timeTemp;
  //  xbee.println(realTime);

  //GETTING THE OVERFLOW VALUE IF TIME >=17ms ELSE overflowValue = tickValue
  //9-16ms = 0 overflow - need realTime = totalTicks;
  //well, up to 100ms lang siya sooo I'll just leave it here
  int frequency = 15625; // clock frequency / prescaler
  long totalTicks;
  if (timeUnit == 0x01) { //if ms; because it is float
    totalTicks = timeMS * frequency;
  }
  else {
    totalTicks = realTime * frequency;
  }
  long overflowCount;
  if (timeTemp <= 16 && timeUnit == 0x01) { //if less than 16ms kasi hindi mag overflow
    overflowCount  = totalTicks;
  }
  else {
    overflowCount = totalTicks / pow(2, 8); //8 coz timer 2 has 8 bits
  }
  portOverflowCount[portOverflowIndex] = overflowCount;

  //  xbee.print("OFC: ");
  //  xbee.println(portOverflowCount[portOverflowIndex]);


  //check overflowCount if reached @ INTERRUPT

}

unsigned int bcdToDecimal(unsigned int nTime) { 
  unsigned int temp = 0;
  //999
  temp = ((nTime >> 8) % 16);
  temp *= 10;
  temp += ((nTime >> 4) % 16);
  temp *= 10;
  temp += (nTime % 16);
//  xbee.print("Converted");
//  xbee.println(temp);
  return temp;
}

void checkTimeout() {
  if (requestConfig == true  && attemptIsSet) { //trying to request config and it has not yet come
    if (attemptCounter <= MAX_ATTEMPT) { // requests again
      initializePacket(packetQueue[packetQueueTail]);
      formatReplyPacket(packetQueue[packetQueueTail], 0x11); // request config
      closePacket(packetQueue[packetQueueTail]);
      //      printQueue(packetQueue, PACKET_QUEUE_SIZE);
      attemptIsSet = false;
      //      xbee.println("Again!!");
    }
    else { // max reached
      attemptCounter = 0x00; // reset
      requestConfig = false;
      errorFlag |= 0x02;
//      initializePacket(packetQueue[packetQueueTail]);
//      formatReplyPacket(packetQueue[packetQueueTail], 0x09); //max attempts is reached
//      closePacket(packetQueue[packetQueueTail]);
//      xbee.println("Max reached");
    }
    sendPacketQueue();
  }
}

ISR(TIMER2_OVF_vect) {
  timeCtr++;
  //  xbee.println(timeCtr);
  if ((timeCtr % portOverflowCount[PORT_COUNT]) == 0 && (requestConfig == true) && (portOverflowCount[PORT_COUNT] != 0)) { // when requesting at startup
    //if it reached timeout and requestConfig is set and it is not zero
    attemptCounter = attemptCounter + 0x01; // increment counter
    attemptIsSet = true;
  }
  if (((timeCtr % portOverflowCount[0]) == 0) && ((timerRequest & 0x0001) == 0x0001)) { //if it is time and there is a request
    timerGrant |= (1 << 0); // turn on grant
    //    timerGrant |= 0x01;
//    xbee.println(portOverflowCount[0],HEX);
//    xbee.println(timeCtr);
//    xbee.print("G: ");
//    xbee.println(timerGrant, HEX);
    timerRequest = timerRequest & ~(1 << 0); // turn off request flag
//    xbee.println(timerRequest, HEX);
  }
  if (((timeCtr % portOverflowCount[1]) == 0) && ((timerRequest & 0x0002) == 0x0002)) {
    timerGrant |= (1 << 1);
    timerRequest = timerRequest & ~(1 << 1);
  }
  if (((timeCtr % portOverflowCount[2]) == 0) && ((timerRequest & 0x0004) == 0x0004)) {
    timerGrant |= (1 << 2);
    timerRequest = timerRequest & ~(1 << 2);
  }
  if (((timeCtr % portOverflowCount[3]) == 0) && ((timerRequest & 0x0008) == 0x0008)) {
    timerGrant |= (1 << 3);
    timerRequest = timerRequest & ~(1 << 3);
  }
  if (((timeCtr % portOverflowCount[4]) == 0) && ((timerRequest & 0x0010) == 0x0010)) {
    timerGrant |= (1 << 4);
    timerRequest = timerRequest & ~(1 << 4);
  }
  if (((timeCtr % portOverflowCount[5]) == 0) && ((timerRequest & 0x0020) == 0x0020)) {
    timerGrant |= (1 << 5);
    timerRequest = timerRequest & ~(1 << 5);
  }
  if (((timeCtr % portOverflowCount[6]) == 0) && ((timerRequest & 0x0040) == 0x0040)) {
    timerGrant |= (1 << 6);
    timerRequest = timerRequest & ~(1 << 6);
  }
  if (((timeCtr % portOverflowCount[7]) == 0) && ((timerRequest & 0x0080) == 0x0080)) {
    timerGrant |= (1 << 7);
    timerRequest = timerRequest & ~(1 << 7);
  }
  if (((timeCtr % portOverflowCount[8]) == 0) && ((timerRequest & 0x0100) == 0x0100)) {
    timerGrant |= (1 << 8);
    timerRequest = timerRequest & ~(1 << 8);
  }
  if (((timeCtr % portOverflowCount[9]) == 0) && ((timerRequest & 0x0200) == 0x0200)) {
    timerGrant |= (1 << 9);
    timerRequest = timerRequest & ~(1 << 9);
  }
  if (((timeCtr % portOverflowCount[0x0A]) == 0) && ((timerRequest & 0x0400) == 0x0400)) {
    timerGrant |= (1 << 0x0A);
    timerRequest = timerRequest & ~(1 << 0x0A);
  }
  if (((timeCtr % portOverflowCount[0x0B]) == 0) && ((timerRequest & 0x0800) == 0x0800)) {
    timerGrant |= (1 << 0x0B);
    timerRequest = timerRequest & ~(1 << 0x0B);
  }
}

void initializeTimer() {
  //  xbee.println("S Timer");
  //setting it up to normal mode
  // one OVF = 16ms ( 256 / (16MHZ/1024))
  cli(); //disable global interrupts

  TCCR2A = 0 ;
  TCCR2B = 0 ;
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // set prescaler to 1024
  TIMSK2 |= (1 << TOIE2); //enable interrupt to overflow
  sei(); //enable global interrupts
}

void setup() {
  //attaching the pin to
  servo0.attach(14); // A0
  servo1.attach(15); // A1
  servo2.attach(16); // A2
//  servo3.attach(17);
//  servo4.attach(18);
//  servo5.attach(19);
  
  //initiallize startup pins
  pinMode(SUCCESS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  Serial.begin(9600);
  
  //initialize port pins
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  //setting to low
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // to initiate serial communication
  xbee.begin(9600);
  
  timeCtr = 0;

  // NOTE: AVOID PUTTING STUFF ON PIN 0 & 1 coz that is where serial is (programming, debugging)
  for (byte c = 0x00; c < 0x0C; c++) {
    setPortConfigSegmentInit(c, 0x08); // set the port to OUTPUT
    //    xbee.print("Port Config Segment at Start: ");
    //    xbee.println(getPortConfigSegment(c), HEX);
  }

  //sets values of the arrays to 0, needed else it would not print anything
  memset(actuatorValueOnDemandSegment, 0, sizeof(actuatorValueOnDemandSegment));
  memset(actuatorValueTimerSegment, 0, sizeof(actuatorValueTimerSegment));
  memset(portValue, 0, sizeof(portValue));
  memset(timerSegment, 0, sizeof(timerSegment));
  memset(eventSegment, 0, sizeof(eventSegment));
  memset(actuatorDetailSegment, 0, sizeof(actuatorDetailSegment));
  memset(portOverflowCount, 0, sizeof(portOverflowCount));
  memset(serialBuffer, 0x00, sizeof(serialBuffer));
  memset(serialQueue, 0x00, sizeof(serialQueue));
  memset(convertedEventSegment, 0x00, sizeof(convertedEventSegment));
//  xbee.println(eventSegment[0]);

//      if(SD.begin(CS_PIN)){ // uncomment entire block to reset node (to all 0)
//        writeConfig(); //meron itong sd.begin kasi nagrurun ito ideally after config... therefore na sd.begin na ni loadConfig na ito sooo if gusto mo siya irun agad, place sd.begin
//        xbee.println("Fin");
//      }
//      else{
//        byte temp = 0x01;
//        errorFlag |= temp; // cannot access sd card
//        xbee.println(errorFlag, HEX);
//      }

  loadConfig(); //basically during the node's lifetime, lagi ito una, so if mag fail ito, may problem sa sd card (either wala or sira) therefore contact sink
//  printRegisters();
}

void loop() {
  boolean wait = 0;
  byte serialData;  //temporary store of data read from Serial
  static byte index = 0; //for queue
  static size_t pos = 0; //for buffer

  //Communication module - Receive
  if (xbee.available() > 0) {
    if(errorFlag == 0x00) { //if no errors, accept the bytes
      serialData = xbee.read(); // 1 byte
  //    xbee.print(serialData, HEX);
  
      if (serialData == 0xFF) { // serialhead found start reading
        headerFound = true;
      }
  
      if (headerFound) {
        serialBuffer[pos++] = serialData;
      }
  
      if (serialData == 0xFE) {
      serialBuffer[pos] = 0xFE; //adds footer
      //      xbee.print("T: ");
      //      xbee.println(serialTail, HEX);

      if (serialHead != ((serialTail + 0x01) % SERIAL_QUEUE_SIZE)) { // tail is producer
        isEmpty = false;

        for (byte x = 0x00; x < pos; x++) {
          //store data to perma queue
          serialQueue[serialTail][x] = serialBuffer[x];
          //          xbee.print(serialQueue[serialTail][x],HEX);
        }
        //        xbee.println();
        //        printBuffer(serialQueue);

        pos = 0;
        serialTail = (serialTail + 0x01) % SERIAL_QUEUE_SIZE; // increment tail
        headerFound = false;
//        xbee.println("Read");
//        printQueue(serialQueue, SERIAL_QUEUE_SIZE);
//        xbee.print("H: ");
//        xbee.println(serialHead, HEX);
//        xbee.print("T: ");
//        xbee.println(serialTail, HEX);
      }
      else {
        xbee.println("Full Queue");
        isFull = true;
        printQueue(serialQueue, SERIAL_QUEUE_SIZE);
        isService = true;
        pos = 0;
      }
    }
    }
    else{
      xbee.println("Packet dropped due to startup error");
    }
  }
  else {
    isService = true;
    if (requestConfig == true)
      checkTimeout(); //if nothing is received
  }

  if (isService) { // check serial queue
    isService = false;

    if (!isEmpty) { // there are messages at queue
      retrieveSerialQueue(serialQueue[serialHead], serialHead); //get message, store to variables, setting flags
//      printRegisters();
      serialHead = (serialHead + 0x01) % SERIAL_QUEUE_SIZE; // increment head
      if (serialHead == serialTail) { //check if empty
        isEmpty = true;
      }
      //insert checking muna here
      //this is processing na like waiting for the next part number, formatting replies
      if (requestConfig == true) { //if it is waiting for config
        //if the packet is a config packet
        if ((packetTypeFlag & 0x01) == 0x01) { // request startup config
          if (configSentPartCtr == configPartNumber) {
            attemptCounter = 0x00; //reset it coz may dumating na tama
            packetTypeFlag = packetTypeFlag & 0xFE;
            configPartNumber = configPartNumber + 0x01; //expect next packet
            if (configPartNumber-1 == MAX_CONFIG_PART-1) { // if it is max already
              writeConfig(); //save config
              requestConfig = false;  // turn off request
              attemptCounter = 0xFF; //para hindi magreset yung timer
              configPartNumber = 0x00;
              initializePacket(packetQueue[packetQueueTail]);
              formatReplyPacket(packetQueue[packetQueueTail], 0x0C); //acknowledge of full config
              closePacket(packetQueue[packetQueueTail]);
            }
          }
          else {
//              xbee.print("SerialHead: ");
//              xbee.println(serialHead,HEX);
//              xbee.print("SerialTail: ");
//              xbee.println(serialTail,HEX);
              xbee.println("broken config");
//              initializePacket(packetQueue[packetQueueTail]);
//              formatReplyPacket(packetQueue[packetQueueTail], 0x08); //sent configuration is broken
//              closePacket(packetQueue[packetQueueTail]);
              errorFlag |= 0x04;
              requestConfig = false;
          }
        }
        else { // unexpected packet (non config type) drop itttt
        }
      }
      else if ((packetTypeFlag & 0x02) == 0x02) { // node configuration
        boolean applyConfig = checkPortConfig();
        if (applyConfig) { // if successfully applied
          initializePacket(packetQueue[packetQueueTail]);
          formatReplyPacket(packetQueue[packetQueueTail], 0x06);
          closePacket(packetQueue[packetQueueTail]);
          writeConfig();
          packetTypeFlag = packetTypeFlag & 0xFD; // turn off node config flag

          if (timerReset) { // if needs to be reset
            xbee.println("timerReset");
            initializeTimer();
            timerReset = false;
          }
          else{
//            xbee.println("!timerReset");
          }
        } else {
          xbee.println("!config");
        }
      }
      else if ((packetTypeFlag & 0x04) == 0x04) { // node discovery
        if (commandValue == 0x00) { // Request Keep Alive
          initializePacket(packetQueue[packetQueueTail]);
          formatReplyPacket(packetQueue[packetQueueTail], 0x01);
          closePacket(packetQueue[packetQueueTail]);
        }
        else if (commandValue == 0x02) {// Network Config
          initializePacket(packetQueue[packetQueueTail]);
          formatReplyPacket(packetQueue[packetQueueTail], 0x03);
          closePacket(packetQueue[packetQueueTail]);
        }
        packetTypeFlag = packetTypeFlag & 0xFB; // turn off packet type flag for node discov
      }
      sendPacketQueue();
    }
    else { //main loop if no message (checking flags)
      if ((errorFlag & 0x0F) != 0x00){ //if there was any startup error
//        xbee.println("Startup error!");
        initializePacket(packetQueue[packetQueueTail]);
        //find which startup error
//        xbee.print("ERROR FLAG: ");
//        xbee.println(errorFlag, HEX);
        if((errorFlag & 0x01) == 0x01){
//          xbee.println("No SD");
          formatReplyPacket(packetQueue[packetQueueTail], 0x10);
          errorFlag = errorFlag & 0xFE; //turn off error
        }
        else if((errorFlag & 0x02) == 0x02){
//          xbee.println("Max attempt");
          formatReplyPacket(packetQueue[packetQueueTail], 0x09);
          errorFlag = errorFlag & 0xFD; //turn off error
        }
        else if((errorFlag & 0x04) == 0x04){
//          xbee.println("Sent config is broken");
          formatReplyPacket(packetQueue[packetQueueTail], 0x08);
          //insert here ano yung kulang na parts
          errorFlag = errorFlag & 0xFB; //turn off error
        }
        else if((errorFlag & 0x08) == 0x08){
//          xbee.println("Error writing file");
          formatReplyPacket(packetQueue[packetQueueTail], 0x07);
          errorFlag = errorFlag & 0xF7; //turn off error
        }
        closePacket(packetQueue[packetQueueTail]);
        requestConfig = false; //turn off requesting config
        attemptCounter = 0x00;
        attemptIsSet = false; // to turn off indicator that the attempt has been counted
        sendPacketQueue();
        digitalWrite(ERROR_LED_PIN, HIGH);
      }
      else{ // if there is no startup error
        
        if(timerGrant != 0x00){ //check timer grant
//        xbee.println("Timer Grant");
//        xbee.print("@main");
//        xbee.println(timerGrant,HEX);
          unsigned int timerGrantMask = 0x00;

          for (byte x = 0x00; x < PORT_COUNT; x++){
            timerGrantMask = (1 << x); 

            if((timerGrantMask & timerGrant) == timerGrantMask){ // if set
              manipulatePortData(x,0x00); //timer write / read
              timerGrant = timerGrant & ~(1<<x); // clear timer grant of bit
              timerRequest = timerRequest | (1 << x); //request again
//              xbee.println(timerGrant, HEX);
//              xbee.println(timerRequest, HEX);
            }
          }
//        xbee.print("End loop timerGrant: ");
//        xbee.println(timerGrant, HEX);// kahit hindi zero kasi interrupt ito
        }
        if (eventRequest != 0x00) { // check event request
          unsigned int eventRequestMask = 0x0000;
          int eventValue;
          byte eventCondition;
          boolean conditionReached = false;
          int tempPortValue;
          byte eventType;

          for (byte x = 0x00; x < PORT_COUNT; x++) {
            eventRequestMask = (1 << x);
//            xbee.println("-----");
//            xbee.print("PORT: ");
//            xbee.println(x, HEX); 

          //check if port is event based
          if ((eventRequestMask & eventRequest) == eventRequestMask) {

            //check condition
            convertEventDetailsToDecimal(x); // MOVE UPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
            eventValue = convertedEventSegment[x];
            eventCondition = ((eventSegment[x] & 0x3000) >> 12); //retain the condition
            eventType = ((eventSegment[x] & 0xF000) >> 12); //getting event values
            
            //read port value
            manipulatePortData(x, 0x03);
            
            conditionReached = checkEventCondition(eventCondition, portValue[x], eventValue);
  
            if ((eventType & 0x80) == 0x80) { //check if range mode
              xbee.println("Range!");
              
              eventCondition = ((eventSegment[x + 0x0C] & 0x3000) >> 12); //retain the condition
              eventValue = convertedEventSegment[x + 0x0C]; //get second value
  
              conditionReached &= checkEventCondition(eventCondition, tempPortValue, eventValue); //check again
              xbee.print("Condition Result:");
              xbee.println(conditionReached);
            }
  
            if (conditionReached) {
//              xbee.println("condition was true");
              portValue[x] = tempPortValue; //save port value
              portDataChanged |= eventRequestMask; // to tell that the port data has changed

              //convert actuator details to its proper place before manipulating
              byte actuatorPort = ((actuatorDetailSegment[x] & 0xF000) >> 12);
              portConfigSegment[actuatorPort] |= 0x08; //set the actuator port to actuator
              actuatorDetailSegment[actuatorPort] = actuatorDetailSegment[x] & 0x0FFF; //store the event details of the actuator port to its corresponding actuator details
                
              manipulatePortData(actuatorPort, 0x01); // write data to actuator port and store its port value
              portDataChanged |= (1 << actuatorPort); // tells port data of actuator port has changed
              eventRequest &= ~(1 << x); //turn off event request of sensor bit
//              xbee.print("Event Request: ");
//              xbee.println(eventRequest, HEX);
              conditionReached = false; // reset
            }
          }
          }
//          xbee.print("End loop eventRequest: ");
//          xbee.println(eventRequest, HEX);// dapat zero
      }
        if (portDataChanged != 0x00) { //to form packet
          unsigned int portDataChangedMask;
          initializePacket(packetQueue[packetQueueTail]);
          
          for (byte x = 0x00; x < PORT_COUNT; x++) { //find which port was changed
            portDataChangedMask = (1 << x);
  
            if ((portDataChanged & portDataChangedMask) == portDataChangedMask) { //portData was changed
              insertToPacket(packetQueue[packetQueueTail], x);
              portDataChanged = portDataChanged & ~portDataChangedMask; //turn off port data changed of bit
  //            xbee.print("data change after send: ");
  //            xbee.println(portDataChanged,HEX);
            }
          }
          closePacket(packetQueue[packetQueueTail]);
          sendPacketQueue();
        }
        //if non startup error, just send and toggle it off
        if (errorFlag > 0x0F){ //if error is on upper nibble
          byte errorFlagMask = 0x00;
          xbee.println("Not startup error");

          for (byte y = 0x10; y < PORT_COUNT; y++){ //mali ito
            errorFlagMask = (1 << y);
            if((errorFlag & errorFlagMask) == errorFlagMask){
              xbee.println("Trying to send it");
              errorFlag = errorFlag & ~(errorFlagMask);
            }
          }
          
        }
      }
    }
  }
}
