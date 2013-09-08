// **********************************************************
// **                OpenLRS EEPROM Functions              **
// **        Developed by Melih Karakelle on 2010-2012     **
// **          This Source code licensed under GPL         **
// **********************************************************
// Latest Code Update : 2012-03-09
// Supported Hardware : OpenLRS Rx-Tx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/

#define REGS_EERPON_ADR 17     /* first byte of eeprom */
#define FS_EEPROM_ADR   64     /* address of FS settings   */
#define STAT_EPROM_ADR  88     /* начальный адрес статистики в EEPROM */
unsigned int LAST_EEPROM_ADR=1024;   /* последний адрес статистики в EEPROM для ATMEGA328 = 35*26 байт*/

#define FLASH_SIZE 16384         /* размер контроллируемой памяти программ */
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
  
  for(i=0; i<FLASH_SIZE; i++)        // считаем сумму 
      ks+=pgm_read_byte(i);
  
   sign=version[0] + (version[1]<<8);     // сигнатура из номера версии
   i=read_eeprom_uint(FLASH_SIGN_ADR);    // прежняя сигнатура
   if(sign != i) {                        // при несовпадении, пропишем новые значения
     write_eeprom_uint(FLASH_SIGN_ADR,sign); 
     write_eeprom_uint(FLASH_KS_ADR,ks);
   } else {                               // в противном случае проверяем КС
     i=read_eeprom_uint(FLASH_KS_ADR);
     if(i != ks) return 1;                // признак разрушенной прошивки
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
   ks+= SAWreg[0] = read_eeprom_uchar(25);  
   ks+= SAWreg[1] = read_eeprom_uchar(26);  

   i=read_eeprom_uchar(28);           // номер первого PWM в PPM режиме
   if(i>0 && i<=8) { pwm1chnl=i; ks+=i; }
   
// Регистры RSSI (40-41). Задают тип (биби/Вольты) и режим (уровень сигнала или отношение сигнал/шум)
   ks+= RSSIreg[0] = read_eeprom_uchar(40);  
   ks+= RSSIreg[1] = read_eeprom_uchar(41);  

   RSSIreg[2] = read_eeprom_uchar(42);  
   if(RSSIreg[2] < 1 || RSSIreg[2] > RC_CHANNEL_COUNT) RSSIreg[2]=0;             // 1 - RC_CHANNEL_COUNT, другое - не использовать
   ks+=RSSIreg[2];

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
   write_eeprom_uchar(25,SAWreg[0]);     ks+=SAWreg[0];  
   write_eeprom_uchar(26,SAWreg[1]);     ks+=SAWreg[1];  

   write_eeprom_uchar(28,pwm1chnl);      ks+=pwm1chnl;  // номер первого PWM в PPM режиме

// Регистры RSSI (40-41). Задают тип (биби/Вольты) и режим (уровень сигнала или отношение сигнал/шум)
   write_eeprom_uchar(40,RSSIreg[0]);     ks+=RSSIreg[0];  
   write_eeprom_uchar(41,RSSIreg[1]);     ks+=RSSIreg[1];  
   write_eeprom_uchar(42,RSSIreg[2]);     ks+=RSSIreg[2];  

   write_eeprom_uint(EEPROM_KS_ADR,ks);        // Write checksum
} 


void eeprom_check(void)              // читаем и проверяем настройки из EEPROM, а также целостность программы
{
  if(flash_check()) {
      if(!satFlag) Serial.println("FLASH ERROR!!! Working unpredictable!");
      Red_LED_Blink(120);  // долго мигаем красным, если КС не сошлась
  }    
  
  if(check_modes(2)) {               //  Джампер на каналах 5-6 - означает сброс настроек к дефолтным
     Red_LED_Blink(4);
     write_eeprom(); 
     if(!satFlag) Serial.println("Settings reset to defaults!");
     Red_LED_Blink(4);
  } else {
    if(!read_eeprom()) {
        if(!satFlag) Serial.println("Error read settings!");
        Red_LED_Blink(120);  // мигаем красным, если КС не сошлась
    }
  }
}  


