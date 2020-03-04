#include <FlexCAN_T4.h>
#include <TimeLib.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> myCan; //initalize to CAN 2.0 mode

int counter = 0;
int msg_counter = 0;
uint8_t can_msg[20] = {0};
time_t cur_time;
unsigned int cur_millis = 0;
tmElements_t tm;
CAN_message_t msg;
char databuf[128];


void setup(void) {
  setTime(Teensy3Clock.get());
  
  Serial.begin(115200); delay(500);
  pinMode(6, OUTPUT); digitalWrite(6, LOW);
  pinMode(23, OUTPUT); digitalWrite(23, LOW);
  myCan.setRx(DEF);
  myCan.setTx(DEF);
  
  myCan.begin();
 // myCan.enableMBInterrupts();
    //myCan.disable
    //myCan.enableFIFO();
    //myCan.enableFIFOInterrupt();
 //  myCan.enableMBInterrupt(FIFO);
 // myCan.onReceive(FIFO, canSniff);
  myCan.setBaudRate(250000);
}

void loop() {

  msg.len = 8; msg.id = 0x321;
  msg.buf[0] = 1; msg.buf[1] = 2; msg.buf[2] = 3; msg.buf[3] = 4;
  msg.buf[4] = 5; msg.buf[5] = 6; msg.buf[6] = 7; msg.buf[7] = 8;
  //delay(500);
  //myCan.write(msg);
  //Serial.println("Send");
  
  if ( myCan.read(msg) ) {
    Serial.println("hello");
    //cur_time = now();
    //cur_millis = millis()%1000;
    //breakTime(cur_time, tm);
    
    msg_counter++;
    /*Could use memcpy in future to make it cleaner*/
//    can_msg[0] = 0; //can msg identifier
//    can_msg[1] = msg.id;
//    can_msg[2] = msg.id >> 8;
//    can_msg[3] = msg.id >> 16;
//    can_msg[4] = msg.id >> 24;
//    can_msg[5] = msg.buf[0];
//    can_msg[6] = msg.buf[1];
//    can_msg[7] = msg.buf[2];
//    can_msg[8] = msg.buf[3];
//    can_msg[9] = msg.buf[4];
//    can_msg[10] = msg.buf[5];
//    can_msg[11] = msg.buf[6];
//    can_msg[12] = msg.buf[7];
//    can_msg[13] = cur_time & 0xFF;
//    can_msg[14] = hour(); //(cur_time >> 8) & 0xFF;
//    can_msg[15] = minute(); //(cur_time >> 16) & 0xFF;
//    can_msg[16] = second (); //(cur_time >> 24) & 0xFF;
//    cur_millis = (unsigned int)(millis()%1000);
//    can_msg[17] = (cur_millis >> 8) & 0xFF;
//    can_msg[18] = cur_millis& 0xFF;
    //can_msg[18] = '\n';

    sprintf(databuf, "0 %4X %X/%X/%X/%X/%X/%X/%X/%X %d-%d-%d-%d", msg.id, msg.buf[0],msg.buf[1],
      msg.buf[2],msg.buf[3],msg.buf[4],msg.buf[5],msg.buf[6],msg.buf[7], hour(), minute(), second(), millis()%1000);
    Serial.println(databuf);

    /*
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
    Serial.print("  EXT: "); Serial.print(msg.flags.extended );
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for ( uint8_t i = 0; i < 8; i++ ) {
      Serial.print(msg.buf[i]); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(msg.timestamp);
    */
  }
}
