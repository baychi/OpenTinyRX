// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Reciever with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-29
// Supported Hardware : Expert Tiny/2G RX, Orange/OpenLRS Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenTinyRX
// **********************************************************

#define REGS_EERPON_ADR 17     /* first byte of eeprom */
#define FS_EEPROM_ADR   64     /* address of FS settings   */
#define STAT_EPROM_ADR  88     /* начальный адрес статистики в EEPROM */
unsigned int LAST_EEPROM_ADR=1024;   /* последний адрес статистики в EEPROM для ATMEGA328 = 35*26 байт*/
#if (__AVR_ATmega328P__ == 1)
  #define FLASH_SIZE 16384         /* размер контроллируемой памяти программ */
#else
  #define FLASH_SIZE 32768         /* размер контроллируемой памяти программ */
#endif

#define FLASH_SIGN_ADR 6         /* адрес сигнатуры прошивки в EEPROM */
#define FLASH_KS_ADR 8           /* адрес контрольной суммы прошивки в EEPROM */
#define EEPROM_KS_ADR 10         /* адрес контрольной суммы настроек в EEPROM */
#define STAT_PTR_ADR 12          /* адрес указателя на очередную запись статистики в EEPROM */
#define STAT_FLIGHT_ADR 14       /* адрес номера полета в EEPROM */ 


unsigned int read_eeprom_uint(int address)
{
 return (EEPROM.read(address) * 256) + EEPROM.read(address+1); 
}

unsigned char read_eeprom_uchar(int address)
{
 return  EEPROM.read(address+REGS_EERPON_ADR); 
}


void write_eeprom_uint(int address,unsigned int value)
{
 EEPROM.write(address,value / 256);  
 EEPROM.write(address+1,value % 256);  
}

void write_eeprom_uchar(int address,unsigned char value)
{
  return  EEPROM.write(REGS_EERPON_ADR+address,value); 
}


// Проверка целостности прошивки
//

byte flash_check(void)
{
  unsigned int i,sign,ks=0;
  
  if(version[1] == 0) return 0;           // при отладке программы, механизм можно отключить
  
  for(i=0; i<FLASH_SIZE; i++)             // считаем сумму 
      ks+=pgm_read_byte(i);
  
   sign=version[0] + (version[1]<<8);     // сигнатура из номера версии
   i=read_eeprom_uint(FLASH_SIGN_ADR);    // прежняя сигнатура
   if(sign != i) {                        // при несовпадении, пропишем новые значения
     write_eeprom_uint(FLASH_SIGN_ADR,sign); 
     write_eeprom_uint(FLASH_KS_ADR,ks);
     return 1;                            // признак 1-го запуска 
   } else {                               // в противном случае проверяем КС
     i=read_eeprom_uint(FLASH_KS_ADR);
     if(i != ks) return 255;                // признак разрушенной прошивки
   }
   
   return 0;                               // все в порядке
}    


// сохранение настроек FS
void save_failsafe_values(void)
{
  for (byte i=0;i<RC_CHANNEL_COUNT;i++)   {
     EEPROM.write(FS_EEPROM_ADR+(2*i),Servo_Buffer[i] / 256); 
     EEPROM.write(FS_EEPROM_ADR+1+(2*i),Servo_Buffer[i] & 0xFF);
   } 
}

// Чтение настроек файлсейва
//
void load_failsafe_values()
{
   for(byte i=0; i<RC_CHANNEL_COUNT; i++)   {
     Servo_Buffer[i] = (EEPROM.read(FS_EEPROM_ADR+(2*i)) * 256) + EEPROM.read(FS_EEPROM_ADR+(2*i)+1);
     if(Servo_Buffer[i] < 1900 || Servo_Buffer[i] > 4100) Servo_Buffer[i]=3000;             // защита от некорретных данных
  }  
}

// Чтение всех настроек
byte read_eeprom(void)
{
   unsigned int ks=0;
   byte i,j;

   for(i=0; i<sizeof(Regs4); i++)      // S/N, номер линка, поправка частоты, разрешение статистики, servostrech
     ks+= Regs4[i] = read_eeprom_uchar(i); 

   for(i=0; i<sizeof(confReg); i++)    // регистры конфигурацииb 7-10
     ks+= confReg[i] = read_eeprom_uchar(i+7); 
     
   // hopping channels
   for(i=0; i<HOPE_NUM; i++)  ks+=hop_list[i] = read_eeprom_uchar(i+11);
  
   // Регистры маяка (19-23): частота, мошность1 - мощность 4.
   for(i=0; i<sizeof(BeaconReg); i++)    ks+=BeaconReg[i] = read_eeprom_uchar(i+19);

// Сформируем ряд убывающих уровней маяка, не больших BeaconReg[1]
   i=BeaconReg[1];   if(i > 7) i=7; 
   if(i >= 6) j=2; else j=1;
   if(i > j) i-=j;    BeaconReg[2]=i;
   if(i > j) i-=j;    BeaconReg[3]=i;
   BeaconReg[4]=0;
   
   // Регистры поддержки SAW фильтра (25,26) 
   for(i=0; i<sizeof(SAWreg); i++)
     ks+= SAWreg[i] = read_eeprom_uchar(25+i);  

   i=read_eeprom_uchar(28);           // номер первого PWM в PPM режиме
   if(i>0 && i<=8) { pwm1chnl=i; ks+=i; }
   
// Регистры RSSI (40-41). Задают тип (биби/Вольты) и режим (уровень сигнала или отношение сигнал/шум)
   for(i=0; i<sizeof(RSSIreg); i++) {
     ks+= RSSIreg[i] = read_eeprom_uchar(40+i);  
   }
   if(RSSIreg[2] < 1 || RSSIreg[2] > RC_CHANNEL_COUNT) RSSIreg[2]=0;             // 1 - RC_CHANNEL_COUNT, другое - не использовать

   if(read_eeprom_uint(EEPROM_KS_ADR) != ks) return 0;            // Checksum error

   return 1;                                            // OK
} 

// Запись всех настроек
void write_eeprom(void)
{
   unsigned int ks=0;
   byte i;
   
   for(i=0; i<sizeof(Regs4); i++) {      // S/N, номер линка, поправка частоты, разрешение статистики, servostrech
     write_eeprom_uchar(i,Regs4[i]);
     ks+=Regs4[i];      
   }     

   for(i=0; i<sizeof(confReg); i++) {    // регистры конфигурацииb 7-10
     write_eeprom_uchar(i+7,confReg[i]);
     ks+=confReg[i];
   }
   
   // hopping channels
   for(i=0; i<HOPE_NUM; i++) {
     write_eeprom_uchar(i+11,hop_list[i]);   
     ks+=hop_list[i];
   }
  
   // Регистры маяка (19-24): частота, мошность1 - мощность 4.
   for(i=0; i<sizeof(BeaconReg); i++) {
     write_eeprom_uchar(i+19,BeaconReg[i]);  
     ks+=BeaconReg[i];  
   }
   // Регистры поддержки SAW фильтра (25,26) 
   for(i=0; i<sizeof(SAWreg); i++) { 
     write_eeprom_uchar(25+i,SAWreg[i]);     ks+=SAWreg[i];  
   }
   write_eeprom_uchar(28,pwm1chnl);      ks+=pwm1chnl;  // номер первого PWM в PPM режиме

// Регистры RSSI (40-41). Задают тип (биби/Вольты) и режим (уровень сигнала или отношение сигнал/шум)
   for(i=0; i<sizeof(RSSIreg); i++) { 
     write_eeprom_uchar(40+i,RSSIreg[i]);     ks+=RSSIreg[i];  
   }
  
   write_eeprom_uint(EEPROM_KS_ADR,ks);        // Write checksum
} 

char etxt1[] PROGMEM = "FLASH ERROR!!! Can't work!";
char etxt2[] PROGMEM = "Error read settings!";
char etxt3[] PROGMEM = "Settings reset to defaults!";


void eeprom_check(void)              // читаем и проверяем настройки из EEPROM, а также целостность программы
{
  byte b=flash_check();
  unsigned long t;
  if(b == 255) {
      if(!satFlag)  printlnPGM(etxt1);
      Red_LED_Blink(29999);          // очень долго мигаем красным, если КС не сошлась
  }    
  
  if(check_modes(2)) {               //  Джампер на каналах 5-6 - означает сброс настроек к дефолтным
     Red_LED_Blink(4);
     write_eeprom(); 
     if(!satFlag)  printlnPGM(etxt3);
     Red_LED_Blink(4);
  } else {
     if(!read_eeprom()) {
       if(!satFlag)  printlnPGM(etxt2);

       if(b == 1) {                  // при первом запуске программы, это ничего страшного
          t=millis()+MENU_WAIT_TIME;
          while(millis() < t) {
            Red_LED_Blink(1);        // мигаем красным, если КС не сошлась
            if(checkMenu()) {       // реализуем возможность входа в меню для ручной коррекции настроек
              doMenu(); 
              return;
            }
          }
          // сбрасываем настройки в значения по умолчанию
          Regs4[1]=72; Regs4[2]=199; Regs4[4]=1;  Regs4[3]=Regs4[5]=Regs4[6]=0;
          SAWreg[0]=75; SAWreg[1]=210;
          BeaconReg[0]=100;  BeaconReg[1]=4;  BeaconReg[2]=2;  BeaconReg[3]=1;  BeaconReg[4]=0;  BeaconReg[5]=30;
          RSSIreg[0]=7; RSSIreg[1]=1; RSSIreg[2]=0;
          hop_list[0]=77; hop_list[1]=147; hop_list[2]=89; hop_list[3]=167; 
          hop_list[4]=109; hop_list[5]=189; hop_list[6]=127; hop_list[7]=209;

          write_eeprom(); 
          return;
       } 
       Red_LED_Blink(120);  // мигаем красным, если КС не сошлась
    }
  }
}  

