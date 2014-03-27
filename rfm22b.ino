// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Reciever with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny/2G RX, Orange/OpenLRS Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenTinyRX
// **********************************************************
 
#define NOP() __asm__ __volatile__("nop") 

// Режимы RFM для 7-го регистра
#define RF22B_PWRSTATE_READY    01 
#define RF22B_PWRSTATE_TX       0x09 
#define RF22B_PWRSTATE_RX       05 

// Режимы прерывания для 5-го регистра
#define RF22B_Rx_packet_received_interrupt   0x02 
#define RF22B_PACKET_SENT_INTERRUPT  04 
#define RF22B_PWRSTATE_POWERDOWN  00 

 
unsigned char ItStatus1, ItStatus2; 

unsigned char read_8bit_data(void); 
// void to_tx_mode(void); 
void to_ready_mode(void); 
void send_8bit_data(unsigned char i); 
void send_read_address(unsigned char i); 
void _spi_write(unsigned char address, unsigned char data); 
void RF22B_init_parameter(void); 

void port_init(void);   
unsigned char _spi_read(unsigned char address); 
void Write0( void ); 
void Write1( void ); 
void timer2_init(void); 
void Write8bitcommand(unsigned char command); 
void to_sleep_mode(void); 
 
 
//***************************************************************************** 

//-------------------------------------------------------------- 
void Write0( void ) 
{ 
    SCK_off;  
    NOP(); 
     
    SDI_off; 
    NOP(); 
     
    SCK_on;  
    NOP(); 
} 
//-------------------------------------------------------------- 
void Write1( void ) 
{ 
    SCK_off;
    NOP(); 
     
    SDI_on;
    NOP(); 
     
    SCK_on; 
    NOP(); 
} 
//-------------------------------------------------------------- 
void Write8bitcommand(unsigned char command)    // keep sel to low 
{ 
   unsigned char n=8; 
    nSEL_on;
    SCK_off;
    nSEL_off; 
    while(n--) 
    { 
         if(command&0x80) 
          Write1(); 
         else 
          Write0();    
              command = command << 1; 
    } 
    SCK_off;
}  


//-------------------------------------------------------------- 
unsigned char _spi_read(unsigned char address) 
{ 
 unsigned char result; 
 send_read_address(address); 
 result = read_8bit_data();  
 nSEL_on; 
 return(result); 
}  

//-------------------------------------------------------------- 
void _spi_write(unsigned char address, unsigned char data) 
{ 
 address |= 0x80; 
 Write8bitcommand(address); 
 send_8bit_data(data);  
 nSEL_on;
}  


//-------Defaults 7400 baud---------------------------------------------- 
void RF22B_init_parameter(void) 
{ 
  ItStatus1 = _spi_read(0x03); // read status, clear interrupt   
  ItStatus2 = _spi_read(0x04); 
  
  _spi_write(0x06, 0x00);    // no interrupt: no wakeup up, lbd, 
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode 
  if(Regs4[2]!=0)  _spi_write(0x09, Regs4[2]);     // точная подстройка частоты 
  else _spi_write(0x09, 199);     // если сброшена, используем умолчание   
  _spi_write(0x0a, 0x05);    // выход CLK 2 МГц ?
  _spi_write(0x0b, 0x12);    // gpio0 TX State
  _spi_write(0x0c, 0x15);    // gpio1 RX State 

#if(RX_BOARD_TYPE == 2)
  _spi_write(0x0d, 0x0a);    // GPIO2, для управления SAW фильтром
#endif
#if(RX_BOARD_TYPE == 1)
  _spi_write(0x0d, 0x1b);    // GPIO2 как индикатор принимаемыемх пакетов
#endif

  _spi_write(0x0e, 0x00);    // gpio 0, 1,2 NO OTHER FUNCTION. 
  
// From Expert
  _spi_write(0x1F, 0x03);    //  Clock recovery
  _spi_write(0x1E, 0x0A);    //  AFC timing
  _spi_write(0x12, 0x60);    //  Режим измерения температуры -40..+85 C
  _spi_write(0x13, 0xF8);    //  Смещение температуры ?
  _spi_write(0x0F, 0x80);    //  АЦП для измерения температуры
  _spi_write(0x1D, 0x40);    //  AFC enable

//--------------------------
   
  _spi_write(0x1c, 0x26);    // IF filter bandwidth
  _spi_write(0x20, 0x44);    // clock recovery oversampling rate  
  _spi_write(0x21, 0x00);    // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0; 
  _spi_write(0x22, 0xF2);    // 0x22   ncoff =5033 = 0x13a9 
  _spi_write(0x23, 0x7C);    // 0x23 
  _spi_write(0x24, 0x06);    // 0x24 
  _spi_write(0x25, 0x7D);    // 0x25 clock recovery ...
  _spi_write(0x2a, 0x1e);    // AFC limiter

  _spi_write(0x30, 0xAC);    // Mode: enable packet handler, msb first, enable crc CCITT, 
  _spi_write(0x32, 0x8C);    // Header: 0x32address enable for headere byte 0, 1,2,3, receive header check for byte 0, 1,2,3 
  _spi_write(0x33, 0x0A);    // header 3, 2, 1,0 used for head length, fixed packet length, synchronize word length 3, 2, 
  _spi_write(0x34, 0x04);    // 2 bytes preamble 
  _spi_write(0x35, 0x22);    // expect full preamble 
  _spi_write(0x36, 0x2d);    // synchronize word 3
  _spi_write(0x37, 0xd4);    // 2
  _spi_write(0x38, 0x00);    // 1
  _spi_write(0x39, 0x00);    // 0

  _spi_write(0x3e, RF_PACK_SIZE);    // total tx 16 byte 
   
  _spi_write(0x6d, 0x0f);    // 7 set power max TX power 
// 7400 bps data rate
  _spi_write(0x6e, 0x3C); //  RATE_7400 
  _spi_write(0x6f, 0x9F); //   

  _spi_write(0x69, 0x60);    //  AGC enable
  _spi_write(0x70, 0x2E);    //  manchester enable!!!
  _spi_write(0x79, 0x00);    // no hopping (1-st channel)
  _spi_write(0x7a, HOPPING_STEP_SIZE);    // 60khz step size (10khz x value) // no hopping 

  _spi_write(0x71, 0x23);  // Gfsk,  fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio 
  _spi_write(0x72, 0x0E);  // frequency deviation setting to 8750

  //band 434.075
 _spi_write(0x75, 0x53);   // 433075 кГц  
 _spi_write(0x76, 0x4C);    
 _spi_write(0x77, 0xE0); 
}


//-------------------------------------------------------------- 
void send_read_address(unsigned char i) 
{ 
 i &= 0x7f; 
  
 Write8bitcommand(i); 
}  
//-------------------------------------------------------------- 
void send_8bit_data(unsigned char i) 
{ 
  unsigned char n = 8; 
  SCK_off;
    while(n--) 
    { 
         if(i&0x80) 
          Write1(); 
         else 
          Write0();    
         i = i << 1; 
    } 
   SCK_off;
}  
//-------------------------------------------------------------- 

unsigned char read_8bit_data(void) 
{ 
  unsigned char Result, i; 
  
 SCK_off;
 Result=0; 
    for(i=0;i<8;i++) 
    {                    //read fifo data byte 
       Result=Result<<1; 
       SCK_on;
       NOP(); 
       if(SDO_1) 
       { 
         Result|=1; 
       } 
       SCK_off;
       NOP(); 
     } 
    return(Result); 
}  
//-------------------------------------------------------------- 

//----------------------------------------------------------------------- 
void rx_reset(void) 
{ 
  _spi_write(0x07, RF22B_PWRSTATE_READY); 
  _spi_write(0x7e, 36);    // threshold for rx almost full, interrupt when 1 byte received 
  _spi_write(0x08, 0x03);    //clear fifo disable multi packet 
  _spi_write(0x08, 0x00);    // clear fifo, disable multi packet 
  _spi_write(0x07,RF22B_PWRSTATE_RX );  // to rx mode 
  _spi_write(0x05, RF22B_Rx_packet_received_interrupt); 
  ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04);  
}  
//-----------------------------------------------------------------------    

void to_rx_mode(void) 
{  
 to_ready_mode(); 
 delay(50); 
 rx_reset(); 
 NOP(); 
}  


//-------------------------------------------------------------- 
void to_ready_mode(void) 
{ 
  ItStatus1 = _spi_read(0x03);   
  ItStatus2 = _spi_read(0x04); 
  _spi_write(0x07, RF22B_PWRSTATE_READY); 
}  
//-------------------------------------------------------------- 
void to_sleep_mode(void) 
{ 
  _spi_write(0x07, RF22B_PWRSTATE_READY);  
   
  ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04);    
  _spi_write(0x07, RF22B_PWRSTATE_POWERDOWN); 

} 
//--------------------------------------------------------------   
  
void sleepMks(word mks)   // аналог delayMicroseconds, но с отдачей квантов SBUS
{
  unsigned long t=micros()+mks;
  
  while(micros() < t)
    sendSbus();  
}  

// Маяк из проекта KHA

void beacon_tone(int16_t hz, int16_t len, byte pow) //duration is now in half seconds.
{
  int16_t d = 500000 / hz; // better resolution

  if (d < 5) d = 5;

  _spi_write(0x6d, (pow&7)|0x8);   // устанавливаем мощность

#if(RX_BOARD_TYPE == 1)
  _spi_write(0x0e, 0x04);     // зажигаем индикатор
#else
  Green_LED_ON
#endif  

  delay(1);

#if(RX_BOARD_TYPE == 1)
  _spi_write(0x0e, 0x00);     // гасим индикатор
#else
  Green_LED_OFF
#endif  

  wdt_reset();               //  поддержка сторожевого таймера

  word cycles = (len * 250000 / d);

  for(word i = 0; i < cycles; i++) {
    SDI_on;
    sleepMks(d);
    SDI_off;
    sleepMks(d);
  }
}

void beacon_send(void)
{
  ItStatus1 = _spi_read(0x03);   // read status, clear interrupt
  ItStatus2 = _spi_read(0x04);
  _spi_write(0x06, 0x00);    // no wakeup up, lbd,
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  _spi_write(0x09, beaconFcorr);  // подстройка частоты
  _spi_write(0x0a, 0x05);
  _spi_write(0x0b, 0x12);    // gpio0 TX State
  _spi_write(0x0c, 0x15);    // gpio1 RX State
  _spi_write(0x0d, 0x0a);    // gpio2 - управление SAW фильтром/или лампочкой
  _spi_write(0x0e, 0x00);    // gpio2=0

  _spi_write(0x70, 0x2C);    // disable manchester

  _spi_write(0x30, 0x00);    //disable packet handling

  _spi_write(0x79, BeaconReg[0]);    // start channel

  _spi_write(0x7a, 0x06);   // 50khz step size (10khz x value) // no hopping

  _spi_write(0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
  _spi_write(0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

  _spi_write(0x73, 0x00);
  _spi_write(0x74, 0x00);    // no offset

   SAW_FILT_OFF              // SAW disable

  _spi_write(0x6d, (BeaconReg[1]&7)|0x8);   // 5 set mid power 25mW
  delay(10);
  _spi_write(0x07, RF22B_PWRSTATE_TX);    // to tx mode
  beacon_tone(440, 1 ,BeaconReg[1]);
  beacon_tone(349, 1, BeaconReg[2]);
  beacon_tone(261, 1, BeaconReg[3]);
  beacon_tone(175, 1, BeaconReg[4]);

  _spi_write(0x07, RF22B_PWRSTATE_READY);
}
 

// Процедура автоматической привязки к передатчику
//
#if (__AVR_ATmega328P__ == 1) 
static byte afcCntr;           // для вычисления поправки частоты
static int afcAvr;      
static byte futMode;           // для определения режима 

bool findHop(byte cnl, word maxTime, byte bind=0, byte afc=0)  // отлов пакета за заданное время     
{
  byte i,crc;
  
  wdt_reset();               //  поддержка сторожевого таймера
  _spi_write(0x79, cnl);
  if(afc) _spi_write(0x1D, 0x40);    //  AFC enable
  to_rx_mode(); 
  unsigned long t=millis()+maxTime;
  RF_Mode = Receive;              // Start next packet wait

  while(RF_Mode == Receive) {
     if(millis() > t) break;  
  }

  if(RF_Mode == Received) {   // если дождались
     send_read_address(0x7f); // Send the package read command
     for(i=0; i<RF_PACK_SIZE; i++) { //read all buffer 
       RF_Rx_Buffer[i] = read_8bit_data(); 
     }  
     rx_reset();

     if(futMode) calcCRC(RF_Rx_Buffer);  // проверяем принятый пакет
     else{                             // если режим не определен 
       crc=CRC8(RF_Rx_Buffer+2,RF_PACK_SIZE-3);  // пробуем классику
       if(crc == 0) Regs4[5] &= 1;     // убираем 2-ку
       else {
         crc=CRC8(RF_Rx_Buffer+1,RF_PACK_SIZE-1);  // или пробуем режим Futaba                             
         if(crc == 0) Regs4[5]=2;     // запоминаем режим  
       }
     }
     if(crc == 0) {
       futMode=1;                      // режим опеределен 
       if(bind && bind != RF_Rx_Buffer[0]) return false;  // если надо, сверяем бинд
       if(afc) { 
         i=_spi_read(0x2B);   // читаем отклонение частоты  
         if(i>128) afcAvr -= (256-i);
         else afcAvr += i;
         afcCntr++;
       }
       return true;
     }
  }

  return false;
}

#define MAX_BIND_TIME 19999         // максимальное время поиска бинда

void makeBind(void)                         // собственно поиск передатчика
{
  byte *buf=new byte[256];
  byte hops[HOPE_NUM];
  byte i,j,k,l,n,hCnt;
  byte bind;
  unsigned long t,maxT;
  byte ue=check_modes(REBIND_JUMPER)==0;    // флаг, разрешающий UART
  
  sei();

repeatAll:
  futMode=hCnt=bind=0;                 // 0 - нет привязки, режим работы не определен 
  afcAvr=afcCntr=0;                    // для вычисления откл. частоты
  if(Regs4[2] < 170 || Regs4[2] > 230) Regs4[2]=199;     // на всякий случай проверим поправку
  
  RF22B_init_parameter();      // подготовим RFMку 
  to_rx_mode(); 
  SAW_FILT_OFF                 
  _spi_write(0x1D, 0x00);    //  AFC disable

  rx_reset();

  if(ue) Serial.print("\r\nBind find start: ");
  for(i=0; i<255; i++) buf[i]=0;
  t=millis()+MAX_BIND_TIME;         // засечем момент начала поиска

//     
// Постоянно сканируем эфир, на предмет выявления возможных частот 

repeatFind:  
  wdt_reset();               //  поддержка сторожевого таймера
  j=k=n=l=0;
  for(i=0; i<255; i++) {
      if(buf[i] < 250) {          // для еще не проверенных частот 
        _spi_write(0x79, i);      // ставим канал
        delayMicroseconds(649);   // очень жаль, что меньше нельзя
        j=_spi_read(0x26);        // читаем уровень сигнала
        if(j>buf[i]) buf[i]=j;    // накапливаем максиммум
        if(j > l) { l=j; k=i; }   // заодно фиксируем самый большой пик
      }
  }
  Green_LED_OFF;

  if(hCnt==0) {
     if(ue) { Serial.print(" Maxlevel["); Serial.print(k); Serial.print("]="); Serial.println(l); }
     if(l < 150) goto repeatAll;
  }
  if(l < 150) goto repeatFind;
  
 //
 // Елси есть кандидаты на отлов

  if(findHop(k,259,bind)) {               // проверяем канал 
    bind=RF_Rx_Buffer[0];                 // фиксируем бинд 
    if(ue) { Serial.print(k); Serial.write(' '); }
    hops[hCnt++]=k;  // берем до 8-ми каналов, лежащих выше порога
    Green_LED_ON;                         // мигаем диодом в честь найденного пакета   
  } 
  buf[k]=255;                              // признак того, что данная частота уже проверенна
  
  if(hCnt == 0) goto repeatAll;               // пока никого не нашли
  if(millis() > t) {
    if(hCnt != 1 && hCnt != 2 && hCnt != 4 && hCnt != HOPE_NUM) goto repeatAll;   // если время поиска истекло и не найдено даже вырожденных случаев
  } else  {  
    if(hCnt < HOPE_NUM) goto repeatFind;         // не набрали минимального числа кандидатов
  }
  
  //
  // Ищем последовательность прыжков
  maxT=millis()+MAX_BIND_TIME/3;                // максимальное время на поиск последовательности
repTimes:
  if(ue) Serial.print("\r\nTimes: ");   
  n=0;
  for(i=1; i<hCnt; i++) {                     
    do {
      if(millis() > maxT) goto repeatAll;         // но не бесконечно
    } while(!findHop(hops[0],259,bind,1));        // ждем первый канал

    t=millis();
    if(findHop(hops[i],599,bind,1)) {            // проверяем, сколько времени надо ждать относительно первого
       t=millis()-t; 
       j=(t-8)/32; 
       if(j >= HOPE_NUM) j-=HOPE_NUM;
       for(k=0; k<i; k++) {                    // повторяющихся времен быть не должно
         if(buf[k] == j) goto repTimes;
       }
       buf[i] = j; 
       if(ue) { Serial.print(j); Serial.write(' '); }
       n++;
    }
  }
  if(ue) Serial.println();
  hop_list[0]=hops[0];                        // первый канал всегда известен

  if(hCnt == 1) {                             // банальный случай всех одинаковых каналов
    for(i=1; i<HOPE_NUM; i++)  hop_list[i]=hops[0];      
  } else if(hCnt == 2) {                      // банальный случай чередования 2-х каналов
    for(i=0; i<HOPE_NUM; i+=2)  { hop_list[i]=hops[0]; hop_list[i+1]=hops[1];   }
  } else if(hCnt == 4) {
     for(i=1; i<4; i++) {                     // расставляем каналы по местам
       j=buf[i]+1;
       if(j < 4)
         hop_list[j+4]=hop_list[j]=hops[i];
     }
  } else {                                    // классический вариант из 8-ми независимых каналов
     if(n < HOPE_NUM-1) goto repeatAll;        
     for(i=1; i<HOPE_NUM; i++) {              // расставляем каналы по местам
       j=buf[i]+1;
       hop_list[j]=hops[i];
     }
  }  

  // Завершаем биндинг 

  afcAvr/=afcCntr;
  Regs4[1]=bind;                              // формируем бинд
   
  if(abs(afcAvr) < 35)  Regs4[2] -= afcAvr;   // и поправка кварца
  if(ue) {
    Serial.print("Bind=");   Serial.print(bind);
    Serial.print(" Fcorr=");   Serial.print(Regs4[2]);
    Serial.print(" Sequence: ");
    for(i=0; i<HOPE_NUM; i++) { Serial.print(hop_list[i]); Serial.write(' '); }
    Serial.println(" End");
  }

  // проверяем остальные настройки на корректность
  Regs4[3]=Regs4[6]=0;                        // сбрасываем удлинитель хода и маски дискр каналов
  if(Regs4[4] > 1) Regs4[4]=0;                 
  
  delete buf;
  write_eeprom();              // записываем новую привязку в EEPROM 
  Red_LED_ON;

#if(RX_BOARD_TYPE == 1)       // с Тини особый случай
  _spi_write(0x0d, 0x0a);     // gpio2 - управление  лампочкой
  _spi_write(0x0e, 0x04);     // зажигаем индикатор
#else
  Red_LED_ON;
#endif
}
#else
void makeBind(void)                         // собственно поиск передатчика
{
}
#endif
