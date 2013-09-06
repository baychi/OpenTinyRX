// **********************************************************
// **                   OpenLRS Functions                  **
// **        Developed by Melih Karakelle on 2010-2011     **
// **          This Source code licensed under GPL         **
// **********************************************************
// Latest Code Update : 2013-08-17
// Supported Hardware : Open Tiny LRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/


void INIT_SERVO_DRIVER(void)
{
   TCCR1B   =   0x00;   //stop timer
   TCNT1H   =   0x00;   //setup
   TCNT1L   =   0x00;
   ICR1   =     40000;   // used for TOP, makes for 50 hz
   
   TCCR1A   =   0x02;   
   TCCR1B   =   0x1A; //start timer with 1/8 prescaler for 0.5us PPM resolution
   
   TIMSK1 = _BV (TOIE1);   
} 

void RFM22B_Int()
{
 if (RF_Mode == Transmit) 
    {
     RF_Mode = Transmitted; 
    } 
 if (RF_Mode == Receive) 
    {
     RF_Mode = Received; 
    }  
}

void Red_LED_Blink(unsigned short blink_count)
{
  unsigned char i;
  for (i=0;i<blink_count;i++)     {
     delay(125);
     Red_LED_ON;
     delay(125);
     Red_LED_OFF;
  }
}


static unsigned pairs[10] = { // пары для 5-х возможных перемычек
    Servo1_OUT,Servo2_OUT,
    Servo3_OUT,Servo4_OUT,
    Servo5_OUT,Servo6_OUT, 
    Servo7_OUT,Servo8_OUT, 
    Servo9_OUT,Servo10_OUT
}; 

// Проверка перемычки на паре каналов (1-2...9-10)
//
unsigned char check_modes(int n)   // 0 - режим PWM/PPM; 1-анализатор спектра?; 2 - сброс регистров в дефолт, 4 -сателлит.
{
  if(n>4) n=0;
  else n+=n;
  
  pinMode(pairs[n+1], INPUT);     // input
  digitalWrite(pairs[n+1], HIGH); // pull up
  digitalWrite(pairs[n], HIGH);   // CH1,3,5 is HIGH
  delayMicroseconds(2);
  if (digitalRead(pairs[n+1]) == HIGH) 	{
	digitalWrite(pairs[n], LOW); // CH1,3,5 is LOW
	delayMicroseconds(2);
	if (digitalRead(pairs[n+1]) == LOW) { // OK jumper plugged
//             pinMode(pairs[n+1], OUTPUT);   // не восстанавливаем выход, что-бы не было конфликта
	     return  1; // Jumper is set
	}
  }
  pinMode(pairs[n+1], OUTPUT);                // перемычка не найдена - восстанавливаем выход

  return  0; // Jumper not set
}

//############# FREQUENCY HOPPING FUNCTIONS #################
void Hopping(void)
{
    unsigned char hn;
    hopping_channel++;
    if (hopping_channel>=HOPE_NUM) hopping_channel = 0;
    hn=hop_list[hopping_channel];
    _spi_write(0x79, hn);

// поддержака SAW фильтра:
   if(hn >= SAWreg[0] && hn <= SAWreg[1]) SAW_FILT_ON // если мы внутри частот фильтра
   else SAW_FILT_OFF

    Rx_RSSI=0; N_RSSI=0;  Pause_RSSI=0; N_pause=0;
}

// Преобразование данных входного буфера buf в длительности PWM
void Buf_To_Servo(unsigned char buf[])
{
     unsigned int i;
     int temp_int;

     for(i = 0; i<RC_CHANNEL_COUNT; i++) { // Write into the Servo Buffer
        temp_int = 4 * buf[i+2];
        if(i<8) {                   // 2-й байт пакета содержит старшие биты 8-ми первых каналов 
           if(buf[1]&(1<<i)) temp_int +=1024; 
         } else temp_int +=temp_int;
         Servo_Buffer[i] = temp_int+1980; // кодируем PWM в мкс*2
      }
}  

void Direct_Servo_Drive(void)         // перекидываем ширины вых. импульсов из временного буфера в рабочий
{
    Servo_Position[AILERON] = Servo_Buffer[AILERON];  
    Servo_Position[ELEVATOR] = Servo_Buffer[ELEVATOR];  
    Servo_Position[THROTTLE] = Servo_Buffer[THROTTLE];  
    Servo_Position[RUDDER] = Servo_Buffer[RUDDER];  
    Servo_Position[RETRACTS] = Servo_Buffer[RETRACTS];  
    Servo_Position[FLAPS] = Servo_Buffer[FLAPS];  
    Servo_Position[AUX1] = Servo_Buffer[AUX1];  
    Servo_Position[AUX2] = Servo_Buffer[AUX2];  
    Servo_Position[AUX3] = Servo_Buffer[AUX3];  
    Servo_Position[AUX4] = Servo_Buffer[AUX4];  
    Servo_Position[AUX5] = Servo_Buffer[AUX5];  
    Servo_Position[AUX6] = Servo_Buffer[AUX6];  
  
    if(RSSIreg[2]) 
       Servo_Position[RSSIreg[2]-1]=(lastRSSI*8)+2000;    // выводим RSSI вместо одно из каналов 
    PWM_enable=1;                      // включаем генерацию PWM
}
 
// Вычисление CRC8 по массиву данных
//
unsigned char CRC8(unsigned char buf[], unsigned char len)
{
   unsigned char i,j,crc=0;
    
   for(i=0; i<len; i++) {
     crc = crc ^ buf[i];
     for(j=0; j<8; j++) {
       if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
       else crc >>= 1;
     }
   }

   return crc;
}  
