//le trying to play with timers and interrupts
#include <avr/interrupt.h>
#include <math.h>
volatile unsigned long timeCtr;
//long portOverflowCounts [(int) PORT_COUNT]; //stores the overflow counters to be checked by interrupt

void setup() {
  
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  Serial.begin(9600);
  
  timeCtr = 0;
//  memset(portOverflowCounts, 0, sizeof(portOverflowCounts));
  initializeTimer();
  calculateOverflow();
}
//vector no program address source
//10 $0012 TIMER2_COMPA
//11 $0014 TIMER2_COMPB
//12 $0016 TIMER2_OVF
ISR(TIMER2_COMPA_vect){
    
//  if((ctr%61) == 0){ // 1 second
//    digitalWrite(2, digitalRead(2)^1); // toggle pin 2 when reached 1 cycle
//    Serial.println("timer stop");
//  }
}

void calculateOverflow(){
   //if set yung timerRegister (set when may timeBased)
  //do this to timeSegment[bit that was set]
  //stop timer ah
  
  //convert time to seconds store to realTime
  unsigned int tempTime = 0x1100; //sample data ; 1.5hrs
  byte timeUnit = tempTime >> 12; // checks which unit 
  unsigned int timeKeeper = tempTime & ((1 << 12)-1); // get time only masking it
  //convert bcd to dec
  long timeTemp = bcd2dec(timeKeeper); //number in seconds
  long realTime = 0x00; //32 bits ; max is 24hrs 86400 seconds
  float timeMS = 0.000;

  //CONVERTS TO SECONDS
  if(timeUnit == 0x01 ){//ms 
    timeMS = timeTemp / 1000.0;
    Serial.println(timeMS,3); //0.99
  }
  else if(timeUnit == 0x02){//sec
    //it is as is
  }
  else if(timeUnit == 0x04){//min    
    timeTemp = timeTemp * 60;
  }
  else if(timeUnit == 0x08){//hour
    timeTemp = timeTemp * 3600;
  }
  
  realTime = timeTemp;
  Serial.println(realTime);

  //GETTING THE OVERFLOW VALUE IF TIME >=17ms ELSE overflowValue = tickValue
  //9-16ms = 0 overflow - need realTime = totalTicks;
  //well, up to 100ms lang siya sooo I'll just leave it here
  int frequency = 15625; // clock frequency / prescaler
  long totalTicks;
  if(timeUnit == 0x01){ //if ms; because it is float
    totalTicks = timeMS * frequency;
  }
  else{
    totalTicks = realTime * frequency;
  }
  long overflowCount;
  if(timeTemp <=16 && timeUnit == 0x01){ //if less than 16ms kasi hindi mag overflow
    overflowCount  = totalTicks;
  }
  else{
    //may iba diyan may delay ng 1 overflow ^^
    overflowCount = totalTicks/pow(2, 8); //8 coz timer 2 has 8 bits
  }
 
  Serial.print("Overflow Count");
  Serial.println(overflowCount);

  
  //check overflowCount if reached @ INTERRUPT
  
}
long bcd2dec(unsigned int nTime){
  byte x;
  long temp = 0;
  //999
  temp = ((nTime>>8)%16);
//  nTime = nTime << 8;
//  Serial.println(temp);
  temp *= 10;
//  Serial.println(temp);
  temp += ((nTime>>4)%16);
  temp *= 10;
  temp += (nTime %16);
  return temp;
}
ISR(TIMER2_COMPB_vect){
//  digitalWrite(3, digitalRead(2)^1); // inverse of pin 2
}
ISR(TIMER2_OVF_vect){
  timeCtr++;
  if((timeCtr%305) == 0){ //5sec
    Serial.println("Five seconds");
    digitalWrite(4, digitalRead(4) ^ 1);
  }
  if((timeCtr%219726) == 0){ //1hr
    Serial.println("1hr");
    digitalWrite(5, digitalRead(5)^1);
  }
  if((timeCtr%36621) == 0){ //10mins
    Serial.println("10mins");
    digitalWrite(6, digitalRead(6)^1);
  }
  if((timeCtr%61) == 0){ //1second
    Serial.println("0.5sec");
    digitalWrite(7, digitalRead(7)^1);
  }
}

void initializeTimer(){
  // I think this is what you need ^^v
  cli(); //disable global interrupts

  GTCCR = (1 << TSM) | (1<<PSRASY)| (1<<PSRSYNC); //halt all timer to setup
  TCCR2A |= (1 << WGM21) ; 
//  OCRA controls the MAX COUNTER VALUE
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // set prescaler to 1024
  TIMSK2 = (1 << OCIE2B) | (1 << OCIE2A) | (1 << TOIE2); //enable interrupt to OCR2A and B (certain ticks) and overflow
  TIFR2 = (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2); //interrupt flag registers
  OCR2A = 0xFF; //timer 2 top; do not change
  OCR2B  = 0xFF; //compare match
  TCNT2 = 1; //offset timer coz OCR2A is TOP
  GTCCR = 0; // restart timer
  sei(); //enable global interrupts   
  Serial.println("Init done"); 
//  tcnt2 = counter mismo // can be read and write
//  OCF2A = enable to allow interrupt 
  //can be modified to change the TOP (in counter)
}
void loop() {
  // put your main code here, to run repeatedly:
//  unsigned  long start, finished, elapsed;
//  Serial.println("Start");
//  start = millis();
//  delay(1000);
//  finished = millis();
//  Serial.println("Finished");
//  elapsed=finished-start;
//  Serial.print(elapsed);
//  Serial.println(" milliseconds elapsed");
//  Serial.println();
}

