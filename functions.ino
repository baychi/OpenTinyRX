// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Reciever with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny/2G RX, Orange/OpenLRS Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenTinyRX
// **********************************************************


void INIT_SERVO_DRIVER(void)
{
   TCCR1B   =   0x00;   //stop timer
   TCNT1H   =   0x00;   //setup
   TCNT1L   =   0x00;
   ICR1   =     ppmPwmCycleTime;   // used for TOP, makes for 50 hz
   
   TCCR1A   =   0x02;   
   TCCR1B   =   0x1A; //start timer with 1/8 prescaler for 0.5us PPM resolution
   
   TIMSK1 = _BV (TOIE1);   
} 

unsigned long rTime;

void RFM22B_Int()
{
  if (RF_Mode == Transmit)    {
      RF_Mode = Transmitted; 
  } 
  if (RF_Mode == Receive)    {
      RF_Mode = Received; 
      rTime=millis();
  }  
}

void Red_LED_Blink(unsigned short blink_count)
{
  unsigned char i;

#if(RX_BOARD_TYPE == 1)         // с Тини особый случай
    _spi_write(0x0d, 0x0a);     // gpio2 - управление SAW фильтром/или лампочкой
#endif
    
  for (i=0; i<blink_count; i++)     {
     wdt_reset();               //  поддержка сторожевого таймера
     delay(125);
     if(blink_count > 10 && checkMenu()) {            // в долгих ситуациях даем возможность входа в меню
       doMenu();
       return; 
     }

#if(RX_BOARD_TYPE == 1)       // с Тини особый случай
  _spi_write(0x0e, 0x04);     // зажигаем индикатор
#else
     Red_LED_ON;
#endif
     delay(125);

#if(RX_BOARD_TYPE == 1)       // с Тини особый случай
  _spi_write(0x0e, 0x00);     // гасим индикатор
#else
   Red_LED_OFF;
#endif
  }
}


static unsigned pairs[14] = { // пары для 5-х возможных перемычек
    Servo1_OUT,Servo2_OUT,
    Servo3_OUT,Servo4_OUT,
    Servo5_OUT,Servo6_OUT, 
    Servo7_OUT,Servo8_OUT, 
    Servo9_OUT,Servo10_OUT,
    0,1,                       // а это UART  
    Servo2_OUT,Servo3_OUT      // для включения SBUS
}; 

// Проверка перемычки на паре каналов (1-2...9-10)
//
unsigned char check_modes(byte n)   // 0 - режим PWM/PPM; 1-анализатор спектра?; 2 - сброс регистров в дефолт, 4 -сателлит. 5-rebind
{
  if(n>6) return 0;
  else n+=n;

  pinMode(pairs[n], INPUT);     // input
  digitalWrite(pairs[n], HIGH); // pull up

  #if(RX_BOARD_TYPE == 1)           // у Тини нет 10-го канала, проверяем на перемычку между 9 и GND
    if(n == 8) {
      delayMicroseconds(199);
      if(digitalRead(pairs[n]) == LOW) return 1; // если притянут к земле, значит перемычка есть

      pinMode(pairs[n], OUTPUT);                // перемычка не найдена - восстанавливаем выход
      return  0; // Jumper not set
    }
  #endif

  digitalWrite(pairs[n+1], HIGH);   // CH1,3,5 is HIGH
  delayMicroseconds(199);
  if (digitalRead(pairs[n]) == HIGH) 	{
	digitalWrite(pairs[n+1], LOW); // CH1,3,5 is LOW
	delayMicroseconds(199);
	if (digitalRead(pairs[n]) == LOW) { // OK jumper plugged
//             pinMode(pairs[n], OUTPUT);   // не восстанавливаем выход, что-бы не было конфликта
	     return  1; // Jumper is set
	}
  }
  pinMode(pairs[n], OUTPUT);                // перемычка не найдена - восстанавливаем выход

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
}

byte futabaDirect[14];         // копия прямого кодирования 10-ти 11 битных каналов для передачи в sbus

/**********************************/
void setFDch(byte n, word pwm) // вставить канал n в буфер futabaDirect
{
  byte bn=n*11;              // абсолютный номер первого бита
  byte m,b=bn>>3;              // номер байта
  bn-= b<<3;                 // номер бита в байте

  m=(1<<(8-bn))-1;             // маска первого байта
  futabaDirect[b++] |= ((pwm&m) << bn);
  pwm >>= 8-bn;
  futabaDirect[b] |= pwm;      // второй байт
  if(bn > 5) {
    futabaDirect[++b] |= pwm>>8; // третий байт, если надо
  }
}

/**********************************/

// Преобразование данных входного буфера buf в длительности PWM
byte Buf_To_Servo(unsigned char buf[])
{
     byte ServoStrechNum=(Regs4[3]%10)-1;   // Номер сервы с расширенном диапазоном
     byte ServoStrechNum2=(Regs4[3]/10)-1;  // Номер сервы с расширенном диапазоном 2
     byte i,m,fsFlag=0;
     int pwm;

     if(Regs4[5] < 2) {                    // Стандартный пакет Экспертовского представления
       byte lowBit=buf[RC_CHANNEL_COUNT+3];
       byte bit11=buf[RC_CHANNEL_COUNT+1];    // 12-й канал, может быть использовани как хранилище 11-го бита
       for(i=0; i<RC_CHANNEL_COUNT; i++) { // Write into the Servo Buffer
         pwm = buf[i+2] << 2;        // основные 8 бит значения
         if(i<8) {                        // 2-й байт пакета содержит старшие биты 8-ми первых каналов 
           if(buf[1]&(1<<i)) pwm |= 0x400;          // это старший бит первых 8-ми каналов
           if(i<7 && (lowBit&(2<<i))) pwm |= 2;     // делаем 10 бит для 7 первых каналов из упр. байта
           if(Regs4[5] && (bit11&(1<<i))) pwm |= 1; // а здесь добавляем еще и 11-й бит, если разрешено 
         } else pwm <<= 1;          // для каналов 9-12 точность 8 бит

         if(i >= 8 && Regs4[5]) {                  // Дополнительный режим 11 бит
           if(i == 8) pwm |= (buf[12]&7);          // доводим 9-й канал до 11 бит, за счет 11-го
           else if( i == 9) {
             pwm |= ((buf[12] >> 3)&7);            // доводим 10-й канал до 11 бит
             Servo_Buffer[7] |= (buf[12]>>5)&2;    // и еще один 2-й бит в 8-й канал 
           } else pwm=0; 
         }
         if(i == ServoStrechNum || i == ServoStrechNum2) {     // реализуем расширение диапазона сервы на 150%
           pwm-=1024;
           pwm+=pwm>>1;       // умножаем на 1.5
           if(pwm > 1200) pwm=1200;
           else if(pwm < -1200) pwm=-1200;
           pwm+=1024;
         }
         Servo_Buffer[i] = pwm+1976;               // кодируем PWM в мкс*2
       } 
       fsFlag=(buf[RF_PACK_SIZE-1]&1);             //  возвращаем признак установки FS
     } else {                           // В режме Futaba, 14 байт данных содержат десять 11-ти битных каналов
       byte j,k;
       word wm;
       
       if(receiver_mode == 2) { // когда включен sbus
         if(Regs4[5] == 3) {             // для прямой пересылки в sbus
           for(i=0; i<sizeof(futabaDirect); i++) futabaDirect[i]=0; // подготовим буфер обнулением
         } else {
           for(i=0; i<sizeof(futabaDirect); i++) futabaDirect[i]=buf[i+1]; // или скопируем непреобразованный пакет, 
         }
       }
       k=m=1;                           // счетчик байт в пакете и маска бита
       for(i=0; i<10; i++) {            // цикл по кналам 
         pwm=0; wm=1;
         for(j=0; j<11; j++) {          // счетчик бит в представлении
           if(buf[k] & m) pwm |= wm;
           wm += wm;
           if(m == 0x80) { m=1; k++; } // обнуляем
           else m = m + m;             // или двигаем маску
         }

         if(Regs4[5] == 3) {           // в режиме 3 обратный порядок каналов 
            j=9-i;     
            if(receiver_mode == 2) setFDch(j,pwm);  // кладем в sbus буфер
         } else j=i;

         if(j == ServoStrechNum || j == ServoStrechNum2) {     // реализуем расширение диапазона сервы на 150%
           pwm-=1024;
           pwm+=pwm>>1;               // умножаем на 1.5
           if(pwm > 1200) pwm=1200;
           else if(pwm < -1200) pwm=-1200;
           pwm+=1024;
         }
         Servo_Buffer[j]=((pwm+pwm+pwm+pwm+pwm)>>2) + 1760;   // формируем значение в PPM буфере
       }
       fsFlag=buf[k]&0x80;            // признак установки FS в пакете          
     }

//
//  Отрабатываем дискретные выхода, если есть

      if(Regs4[6]) {
        m=1;
        for(i=0; i<8; i++) {
           if(Regs4[6] & m) { 
              if(Servo_Buffer[i] > 3000) *portAddr[i] |= diskrMask[i];    // включаем
              else *portAddr[i] &= ~diskrMask[i];                         // или отключаем
           }
           m += m;
        }
      }
    
      return fsFlag;
}  

void Direct_Servo_Drive(void)         // перекидываем ширины вых. импульсов из временного буфера в рабочий
{
    for(byte i=0; i<RC_CHANNEL_COUNT; i++) {
      cli();
      Servo_Position[i] = Servo_Buffer[i];  
      sei();
    }
    if(RSSIreg[2]) {
      cli();
      Servo_Position[RSSIreg[2]-1]=(lastRSSI*8)+2000;    // выводим RSSI вместо одно из каналов 
      sei();
    }
    PWM_enable=1;                     // включаем генерацию PWM
    prepSbusPkt();                    // готовим новый SBUS пакет
}
 
// Вычисление CRC8 по массиву данных
//
byte CRC8(byte buf[], byte len)
{
   byte i,j,crc=0;
    
   for(i=0; i<len; i++) {
     crc = crc ^ buf[i];
     for(j=0; j<8; j++) {
       if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
       else crc >>= 1;
     }
   }

   return crc;
}  

byte calcCRC(byte buf[])
{
   if(Regs4[5] >= 2) return CRC8(buf+1,RF_PACK_SIZE-1);  // проверяем принятый пакет в режиме Futaba
   else return CRC8(buf+2,RF_PACK_SIZE-3);               // проверяем принятый пакет в режиме Эксперта
}       
