// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Reciever with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-12-24
// Supported Hardware : Expert Tiny/2G RX, Orange/OpenLRS Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenTinyRX
// **********************************************************

#include <avr/boot.h>
#define SIGRD 5                     // бит регистров идентификации для boot_signature_byte_get

struct saveStatRec {                // текущая статистика и сохраняемая во FLASH (26 байт)
   byte flightNum;                  // номер полета (инкрементируется при каждом включении)
   byte FS;                         // количество провалов связи и FS в момент сохранения (старший бит)
   byte lost[HOPE_NUM];             // количество потерянных пакетов за интервал (в миинуту макс 238) FF, FE - все - нет потерь (что-бы не насиловать FLASH)
   byte rssi[HOPE_NUM];             // средний уровень RSSI за интервал
   byte noise[HOPE_NUM];            // средний уровень шума за интервал
} saveStat;        

byte statByte,flightCntr=0;         // номер полета
unsigned long statTime;             // время последней записи статистики
unsigned int  statAdr=STAT_EPROM_ADR;      // адрес в EEPROM

void resCurStat(void)               // подготовить новую статистику
{
  for(byte i=0; i<HOPE_NUM; i++) {   
    curStat.rssi[i]=curStat.noise[i]=0;
    curStat.lost[i]=curStat.nc[i]=curStat.rc[i]=0;
  }
  curStat.FS=0;
}

void statSave(void)    // записать очередную запись во FLASH 
{
  byte i; 
  saveStat.flightNum=flightCntr;
  saveStat.FS=curStat.FS;
  if(failsafe_mode || (PWM_enable == 0)) saveStat.FS |= 0x80;       // признак, что находимся в состоянии FS
  
  for(i=0; i<HOPE_NUM; i++) {    // подготовим новую статистику и сохраненние старой
    saveStat.lost[i]=curStat.lost[i];
    saveStat.rssi[i]=saveStat.noise[i]=0;
    if(curStat.rc[i]) saveStat.rssi[i]=curStat.rssi[i]/curStat.rc[i];  // средний сигнал
    if(curStat.nc[i]) saveStat.noise[i]=curStat.noise[i]/curStat.nc[i]; // средний шум
  } 
  resCurStat();                  // обнулим текущую статистику
  statByte=0;                    // запускаем фоновую запись
}

void statLoop(void)                                 // фоновой цикл записи статистики (максиммум 3 байта за раз
{
  byte *ptr=(byte *)&saveStat;
  
  if(Regs4[4] == 0 || sbusDis()) return;                             // if disabled
    
  if(statByte < sizeof(saveStat)) {                 
     EEPROM.write(statAdr+statByte,ptr[statByte]);  // пишем байт
     if(++statByte == sizeof(saveStat)) {           // если закончили, сформируем указатель на след. запись
        statAdr+=sizeof(saveStat);                  
        if(statAdr >= LAST_EEPROM_ADR) statAdr=STAT_EPROM_ADR;
     }
  } else if(statByte == sizeof(saveStat)) {
        EEPROM.write(STAT_PTR_ADR+1,statAdr/256);    // сначала старший бай, так как он важнее
        statByte++;
  } else if(statByte == sizeof(saveStat)+1) {
        EEPROM.write(STAT_PTR_ADR,statAdr&0xff);
        statByte++;
  } else if(statByte == sizeof(saveStat)+2){
       statByte++;
       if(curStat.min == 0)  EEPROM.write(STAT_FLIGHT_ADR,flightCntr);  // номер полета пишем после первой заиси, что-бы не суетится
       curStat.min++;
  }

  if((time-statTime)/1000 >= STAT_INTERVAL) {      // интервал истек 
      statTime=time;
      statSave();                                  // запустим сохранение очередной записи
  }
}  

void statInit(void)                            // инициализация статистики в начале работы
{
   byte i;

   i=boot_signature_byte_get(0x02);            // отличаем Мегу 168 от 328-й
   if(i == 0x94) LAST_EEPROM_ADR=504;          // 16*26 + 88

   statAdr=EEPROM.read(STAT_PTR_ADR);          // читаем указатель на очередную запись
   statAdr += EEPROM.read(STAT_PTR_ADR+1)*256; 
   if(statAdr < STAT_EPROM_ADR || statAdr >=LAST_EEPROM_ADR) { // некорректный указатель
      statAdr=STAT_EPROM_ADR;  
      EEPROM.write(STAT_PTR_ADR,statAdr&0xff);
      EEPROM.write(STAT_PTR_ADR+1,statAdr/256);
      flightCntr=0;      
   } else {
     i=(statAdr-STAT_EPROM_ADR + sizeof(saveStat))%sizeof(saveStat);   // защищаемся от непопадания на границу записи
     if(i != 0)  statAdr-=i;
     flightCntr=EEPROM.read(STAT_FLIGHT_ADR);    // читаем номер полета
   }
   flightCntr++;                                 // ставим новый полет 
   if(flightCntr >= 100) flightCntr=0;           // меняется в интервале 1-100

  resCurStat();                   // готовим текущую статистику
  curStat.min=0;                  // время пойдет с началом связи
  statByte=sizeof(saveStat);      // не будем писать, пока не накопим первую запись
  statTime=millis();
}

void printSpace(byte n)         // печать n пробелов
{
   for(; n>0; n--) Serial.write(' ');
}  


void print3(byte val)  // печать 3-цифр с выравниваем пробелами
{
  if(val < 10) printSpace(2);
  else if(val <100) printSpace(1);
  Serial.print(val);
  printSpace(1);
}  

void print2(byte val)  // печать 2-цифр с выравниваем пробелами
{
  int v=val*100;
  val=v/(238*STAT_INTERVAL/60);
  if(val > 99) val=99;
  if(val < 10)  printSpace(1);
  Serial.print(val);
  printSpace(1);
}  

static byte dFlag;      // признак перевода в дБ  
void print3d(byte val)  // печать 3-цифр в децибелах или тугриках
{
  if(dFlag) val >>=1;
  print3(val);
}  

char stxt1[] PROGMEM = "FSn InFS  Drops:1   2   3   4   5   6   7   8  ";
char stxt2[] PROGMEM = "RSSI:1   2   3   4   5   6   7   8 Noise:1   2   3   4   5   6   7   8";
char stxt3[] PROGMEM = "FSn InFS Dr%:1  2  3  4  5  6  7  8  S/N:1   2   3   4   5   6   7   8";

void statShow(unsigned char mode)                  // вывести статистику на экран
{
   int i;
   byte j,k,tmp;
   byte n=0,fn=0;
   byte fs= (mode == 's') || (mode ==  'S');   // признак вывода только статистики пакетов
   byte fl= (mode == 'l') || (mode ==  'L');   // признак вывода только уровней
   byte fr= (mode == 'r') || (mode ==  'R');   // признак вывода разности C/Ш
   byte fd= (mode == 'd') || (mode ==  'D');   // признак вывода битой статистики
   
   dFlag=0;
   if(mode < 'Z') dFlag=1;                     // прописная буква означает вывод в дБ (тугрика делятся на 2). 
   
   if(!fs && !fl && !fr) fs=fl=1;                     // полный вывод
   Serial.print("Last statisics:("); Serial.print(STAT_EPROM_ADR); Serial.print("-"); 
   Serial.print(LAST_EEPROM_ADR);  Serial.print(") form ");  Serial.println(statAdr);
   Serial.print("FN  cnt ");
   if(fs) printlnPGM(stxt1,0);
   if(fr) printlnPGM(stxt3);
   else if(fl) printlnPGM(stxt2);
   else Serial.println();

   i=statAdr;
   while(1) {
     j=EEPROM.read(i++);               // номер полета
     if(j > 100 && fd == 0) {          // стертая запись или бракованная
       i+=(sizeof(saveStat)-1);        // пропускаем стертые записи
     } else {
       if(j != fn) { n=0; fn=j; }         // счетчик записей внутри полета
       print3(fn);                        // печатаем номер полета
       print3(++n);                       // печатаем счетчик записей
    
       if(fs || fr) {            
         tmp=EEPROM.read(i++);            // кол-во FS
         print3(tmp&0x7f);                // перчатаем FS
         printSpace(1);                   // выравнивание  
         print3((tmp&0x80) == 0x80);      // печатаем признак FS в конце записи
     
         printSpace(3+2*fs);                   // выравнивание  
         if(fs) for(j=0; j<HOPE_NUM; j++) print3(EEPROM.read(i++));  // печатаем потери пакетов
         else for(j=0; j<HOPE_NUM; j++) print2(EEPROM.read(i++));  // печатаем потери пакетов
       } else i+=9;

       if(fl || fr) {
         printSpace(3);                   // выравнивание  
         if(fs) printSpace(1);            // маленькая хитрость, чтобы вписаться в 80 символов
         if(fr) {                         // выводим соотношение сигнал/шум 
           for(j=0; j<HOPE_NUM; j++) { 
               tmp=EEPROM.read(i);  k=EEPROM.read(i+8);  i++;
               if(tmp >= k) tmp-=k;       // если сигнала нет, RSSI=0
               else tmp=0;                // значит и C/Ш=0;  
               print3d(tmp);               // печатаем RSSI-шум
           }
           i+=8;                          
         } else {                        // отдельно сигнал, отдельно шум
           for(j=0; j<HOPE_NUM; j++) print3d(EEPROM.read(i++));  // печатаем RSSI

           printSpace(4);                 // выравнивание  
           for(j=0; j<HOPE_NUM; j++) print3d(EEPROM.read(i++));  // печатаем шум
         }
       } else i+=16;
       
       Serial.println();
       wdt_reset();               //  поддержка сторожевого таймера
     }
     if(i >= LAST_EEPROM_ADR) i=STAT_EPROM_ADR;
     if(i == statAdr) break;           // круг замкнулся, значит мы достигли конца
   }   
}  

void statErase(void)                  // стирание статистики
{
  Serial.print("Statistics in EEPROM erase ");
  for(int i=STAT_EPROM_ADR; i<LAST_EEPROM_ADR; i++) {
       EEPROM.write(i,0xff);
       if((i&0x7f) == 0) Serial.write('.');
       wdt_reset();               //  поддержка сторожевого таймера
  }
  statAdr=STAT_EPROM_ADR;
  flightCntr=0;
  EEPROM.write(STAT_PTR_ADR,statAdr&0xff);
  EEPROM.write(STAT_PTR_ADR+1,statAdr/256);
  EEPROM.write(STAT_FLIGHT_ADR,flightCntr);
  
  statInit();
  Serial.println(" done");
}  
