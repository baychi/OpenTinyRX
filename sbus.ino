// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Reciever with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny/2G RX, Orange/OpenLRS Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenTinyRX
// **********************************************************

#if (__AVR_ATmega328P__ == 1) 

// Реализация SBUS выхода с приемника
//

#define MIN_DIS_TIME 33           // время, на которое можно открыть прерывания, без риска не успеть (20 = 10 мкс)
#define NEED_SEND_TIME 6399       // время по 0.5 мкс, требуемое для гарантированной отправки пакета 
#define MIN_INTERVAL_TIME 6499    // минимальное время в мкс интарвалов между SBUS пакетами


static word lastFront;        // время последнего сформированного фронта

void outBits(byte n, byte v)  // физическое изменение ножки SBUS выхода в заданное время
{
  lastFront+=n;               // формируем время нового фронта

  if((lastFront-TCNT1) > MIN_DIS_TIME) { // если интервал большой
    sei();                                 
    while((lastFront-TCNT1) > MIN_DIS_TIME);    // ждем с разрешенными прерываниями
    cli();
  }  

  while(lastFront != TCNT1);    // теперь точно ждем наступления момента, закрыв прерывания

  if(v) SBUS_OUT_HIGH;          // меняем состояние лапки 
  else  SBUS_OUT_LOW;
}


static byte lastBit=0,timCntr=0;

void putBit(byte b)               // Вывод очередного бита
{
  if(b == lastBit) {               // пока бит не меняется 
    timCntr += 20;                 // просто наращиваем время  
    return;
  }
  
  outBits(timCntr,b);              // выводим накопленное
  lastBit=b;                       // и начинаем новое накопление 
  timCntr=20;
}

void putByte(byte b) 
{
   byte i,j,m=1;

   putBit(0);                      // вывод стартового бита 
   for(i=j=0; i<8; i++) {
     if(b&m) { putBit(1); j++; }   // j - счетчик единиц
     else putBit(0);

     m=m<<1;                       // двигаем маску
   }  
   if(j&1) putBit(1);              // выводим бит четности       
   else putBit(0);

   putBit(1);  putBit(1);          // и 2 стоповых бита
}  

static byte sbusPkt[25];           // пакет SBUS протокола

void prepSbusPkt()                 // подготовка пакета SBUS из текущих данных
{
  word pwm;
  byte i,j,b=0,n=1,m=1;
  
  if(receiver_mode!=2) return;    // когда нет SBUS режима
  
  for(i=0; i<RC_CHANNEL_COUNT; i++) {
     pwm=Servo_Position[i];
     pwm=((pwm-1760)<<2)/5;       // переходим от мкс*2 к битовому представления

     for(j=0; j<11; j++) {        // закидываем 11 бит очередного канала
       if(pwm & 1) b |= m;        // накапливаем биты в байте
       pwm >>= 1;
       if(m == 0x80) {            // если накопили 8 бит
          sbusPkt[n++]=b;         // кладем в пакет 
          b=0;  m=1;              // и начинаем новый байт
       } else m += m;             // или просто двигаем маску  
     }
  }    

  sbusPkt[n++]=b;                 // кладем остаток последнего 12-го канала
  for(; n<23; n++) sbusPkt[n]=0;  // остаток заполняем нулями
  
  sbusPkt[0]=0xf;
  b=0; if(failsafe_mode) b=8;     // формируем признаки FS и потерянных пакетов
  sbusPkt[23]=b;
  sbusPkt[24]=0;
}


unsigned long tSbus=0;            // время последней отправки sbus пакета

void sendSbus()
{
  unsigned long t;

  if(receiver_mode!=2 || !PWM_enable) return;

  t=micros();
  if((t-tSbus) < MIN_INTERVAL_TIME ||     // мв не должны слать пакеты чаще, чем нужно
     TCNT1 > (ppmPwmCycleTime-NEED_SEND_TIME)) return;    // нам потребуется 3 мс, поэтому не всегда можно выдать пакет SBUS
  tSbus=t;

  if(lastPackBad) sbusPkt[23] |= 4;   // добавим признак битого пакета    

  cli();
  lastBit=0; timCntr=0;           // готовим его контекст
  lastFront=TCNT1;
  SBUS_OUT_LOW;                   // начинаем первый стратовый бит

  for(byte i=0; i<sizeof(sbusPkt); i++)  // выводим байты пакета
     putByte(sbusPkt[i]);
 
//  SBUS_OUT_HIGH;                 // в конце всегда стоповые биты
  sei();                         // окончаельно открываем прерывания
}

bool sbusDis()                    // проверка на запрет по времени для тормозный функций типа статистики
{
  if(receiver_mode!=2) return false; // не sbus

  if(micros()-tSbus < MIN_INTERVAL_TIME) return false;  // недавно отправлялись
  
  return true;
}  

#endif
