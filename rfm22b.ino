// **********************************************************
// **      RFM22B/Si4432 control functions for OpenLRS     **
// **       This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2011-09-26
// Supported Hardware : OpenLRS Tx/Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************
 
#define NOP() __asm__ __volatile__("nop") 

 
// Режимы RFM для 7-го регистра
#define RF22B_PWRSTATE_READY    01 
#define RF22B_PWRSTATE_TX        0x09 
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

//  _spi_write(0x3a, Regs4[0]);    // tx header 
//  _spi_write(0x3b, Regs4[1]);    // реально не используются
//  _spi_write(0x3c, Regs4[2]); 
//  _spi_write(0x3d, Regs4[3]); 
  _spi_write(0x3e, RF_PACK_SIZE);    // total tx 16 byte 
   
    //RX HEADER
// _spi_write(0x3f, Regs4[0]);   // check hearder не используется !!!!
// _spi_write(0x40, Regs4[1]); 
// _spi_write(0x41, Regs4[2]); 
// _spi_write(0x42, Regs4[3]); 
// _spi_write(0x43, 0xff);    // all the bit to be checked 
// _spi_write(0x44, 0xff);    // all the bit to be checked 
// _spi_write(0x45, 0xff);    // all the bit to be checked 
// _spi_write(0x46, 0xff);    // all the bit to be checked 
  
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
//  _spi_write(0x73, 0x00);  // frquensy offset 
//  _spi_write(0x74, 0x00);    // no offset 
 

  //band 434.075
 _spi_write(0x75, 0x53);   // 433075 кГц  
 _spi_write(0x76, 0x4C);    
 _spi_write(0x77, 0xE0); 
// _spi_write(0x06, 0x10);    // Enable RSSI interrupt

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

/*****************************************************************
void to_tx_mode(void) 
{ 
 unsigned char i;
 to_ready_mode();
 
  _spi_write(0x08, 0x03);    // disABLE AUTO TX MODE, enable multi packet clear fifo 
  _spi_write(0x08, 0x00);    // disABLE AUTO TX MODE, enable multi packet, clear fifo 
  
    // ph +fifo mode 
  _spi_write(0x34, 4);      //  4 nibble = 2byte preamble 
  _spi_write(0x3e, RF_PACK_SIZE);    // total tx 17 byte 

 for (i = 0; i<RF_PACK_SIZE; i++) { 
  _spi_write(0x7f, RF_Tx_Buffer[i]); 
 } 
 
 _spi_write(0x05, RF22B_PACKET_SENT_INTERRUPT);  
 ItStatus1 = _spi_read(0x03);      //read the Interrupt Status1 register 
 ItStatus2 = _spi_read(0x04); 
  _spi_write(0x07, RF22B_PWRSTATE_TX);    // to tx mode 

  while(nIRQ_1);

  to_ready_mode();
}  
*****************************************************/

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
  
  
void frequency_configurator(long frequency)
{
/******  
  // frequency formulation from Si4432 chip's datasheet
  // original formulation is working with mHz values and floating numbers, I replaced them with kHz values.
  
  unsigned int frequency_constant = (frequency/10000) - 24; // result is 19 for 430-439.990 Mhz

  frequency = frequency / 10;
  frequency = frequency - 24000;
  
  frequency =  frequency - (frequency_constant*1000);  
  
  frequency = frequency * 64; // this is the Nominal Carrier Frequency (fc) value for register setting
  
   
  byte byte0 = (byte) frequency;
  byte byte1 = (byte) (frequency >> 8);
  
  _spi_write(0x75, 0x40 + frequency_constant);
  _spi_write(0x76, byte1);    
  _spi_write(0x77, byte0); 
**************************/
  // no variantrs
 _spi_write(0x75, 0x53);   // 433075 кГц  
 _spi_write(0x76, 0x4C);    
 _spi_write(0x77, 0xE0); 

}

// Маяк из проекта KHA

void beacon_tone(int16_t hz, int16_t len) //duration is now in half seconds.
{
  int16_t d = 500000 / hz; // better resolution

  if (d < 5) {
    d = 5;
  }

  int16_t cycles = (len * 250000 / d);

  for (int16_t i = 0; i < cycles; i++) {
    SDI_on;
    delayMicroseconds(d);
    SDI_off;
    delayMicroseconds(d);
  }
}

void beacon_send(void)
{
  ItStatus1 = _spi_read(0x03);   // read status, clear interrupt
  ItStatus2 = _spi_read(0x04);
  _spi_write(0x06, 0x00);    // no wakeup up, lbd,
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  _spi_write(0x09, Regs4[2]);  // подстройка частоты
  _spi_write(0x0a, 0x05);
  _spi_write(0x0b, 0x12);    // gpio0 TX State
  _spi_write(0x0c, 0x15);    // gpio1 RX State
  _spi_write(0x0d, 0x0a);    // gpio2 - управление SAW фильтром
  _spi_write(0x0e, 0x00);    // gpio2=0

  _spi_write(0x70, 0x2C);    // disable manchester

  _spi_write(0x30, 0x00);    //disable packet handling

  _spi_write(0x79, BeaconReg[0]);    // start channel

  _spi_write(0x7a, 0x06);   // 50khz step size (10khz x value) // no hopping

  _spi_write(0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
  _spi_write(0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

  _spi_write(0x73, 0x00);
  _spi_write(0x74, 0x00);    // no offset

  frequency_configurator(433);  // фиктивный вызов

  _spi_write(0x6d, 0x0f);   // 7 set max power 100mW

//  delay(10);
//  _spi_write(0x07, RF22B_PWRSTATE_TX);    // to tx mode
//  delay(10);

  //close encounters tune
  //  G, A, F, F(lower octave), C
  //octave 3:  392  440  349  175   261

//  beacon_tone(392, 1);

  Green_LED_ON
  _spi_write(0x6d, (BeaconReg[1]&7)|0x8);   // 5 set mid power 25mW
  delay(10);
  _spi_write(0x07, RF22B_PWRSTATE_TX);    // to tx mode
  delay(10);
  Green_LED_OFF
  beacon_tone(440,1);

  Green_LED_ON
  _spi_write(0x6d, (BeaconReg[2]&7)|0x8);   // 4 set mid power 13mW
  delay(10);
  Green_LED_OFF
  beacon_tone(349, 1);

  Green_LED_ON
  _spi_write(0x6d, (BeaconReg[3]&7)|0x8);   // 2 set min power 3mW
  delay(10);
  Green_LED_OFF
  beacon_tone(175,1);

  Green_LED_ON
  _spi_write(0x6d, (BeaconReg[4]&7)|0x8);   // 0 set min power 1.3mW
  delay(10);
  Green_LED_OFF
  beacon_tone(261, 1);

  _spi_write(0x07, RF22B_PWRSTATE_READY);
}
 
