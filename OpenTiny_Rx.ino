// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Reciever with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-11-06
// Supported Hardware : Expert Tiny/2G RX, Orange/OpenLRS Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenTinyRX
// **********************************************************
// # PROJECT DEVELOPERS # 
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne) 
// thUndead (forum nick name: thUndead) 
// Modifyed Baychi to compatable with Expert 2G/Tiny LRS


#include "config.h"
#include <EEPROM.h>

//---------------------------------------------------------------------------
//
// Функции для приемников саттелитов
//
void tryRecvSat(void)
{
  unsigned char in;  
  signed char i,n;
  
  if(time-lastSatTime < SAT_AFTER_TIME) return;    // даем возможность обработать прежний пакет

  n=Serial.available();
  for(i=0; i<n; i++) {              // обрабатываем накопленные в буфере UART байты
    in=Serial.read();             
    if(!satCntr) {                
       if(in == SAT_PACK_HEADER) {  // ищем и принимаем 1-й байт
          satIn[0]=SAT_PACK_HEADER;
          satCntr++;
          satRecFlag=0;            // пока пакет формируется, им неля пользоваться 
          menuFlag=0;              // запрещаем меню
       }
    } else {                      // принимаем очередной байт
      satIn[satCntr++]=in;
      if(satCntr >= SAT_PACK_LEN) {  // достигли конца пакета
         satCntr=0;             
         if(CRC8(satIn+2,SAT_PACK_LEN-3) == 0) {  // проверяем на целостность
             lastSatTime=time;     // фиксируем время пакета
             satRecFlag = 1;       // и флаг его наличия
             break;
         }
       }
    }
  }
}  

//-----------------------------------------------------------------------------
// #include <Wire.h>

void setup() 
{ 
        //LEDs
        pinMode(GREEN_LED_pin, OUTPUT);  
        pinMode(RED_LED_pin, OUTPUT);
        
        //RF module pins
        pinMode(SDO_pin, INPUT); //SDO
        pinMode(SDI_pin, OUTPUT); //SDI        
	pinMode(SCLK_pin, OUTPUT); //SCLK
        pinMode(IRQ_pin, INPUT); //IRQ
        pinMode(nSel_pin, OUTPUT); //nSEL
     
        
        pinMode(0, INPUT); // Serial Rx
        pinMode(1, OUTPUT);// Serial Tx
        digitalWrite(0, HIGH); // pull up

        pinMode(RSSI_OUT, OUTPUT); //RSSI pinout
        
        pinMode(Servo1_OUT, OUTPUT); //Servo1
        pinMode(Servo2_OUT, OUTPUT); //Servo2
        pinMode(Servo3_OUT, OUTPUT); //Servo3
        pinMode(Servo4_OUT, OUTPUT); //Servo4
        pinMode(Servo5_OUT, OUTPUT); //Servo5
        pinMode(Servo6_OUT, OUTPUT); //Servo6
        pinMode(Servo7_OUT, OUTPUT); //Servo7
        pinMode(Servo8_OUT, OUTPUT); //Servo8
        pinMode(Servo9_OUT, OUTPUT); //Servo9
        pinMode(Servo10_OUT, OUTPUT); //Servo10
        
       INIT_SERVO_DRIVER();
       
       attachInterrupt(IRQ_interrupt,RFM22B_Int,FALLING);
}


//############ SERVO INTERRUPT ##############
// We configured the ICR1 value for 40.000 into the "init_servo_driver" function. 
// It's mean this interrupt works when the Timer1 value equal 40.000
// we are configuring it for 40.000 - servo_signal_time for each servo channel, and The interrupt generating perfect servo timings for us.
// Timer1 configured for 1/8 CPU clock. with this configuration, each clock time is equal 0.5us and we are driving the servo with 2048 step resolution.
ISR(TIMER1_OVF_vect)
{
  unsigned int us; // this value is not real microseconds, we are using 0.5us resolution (2048 step), this is why the all values 2 times more than real microseconds.
  
  while (TCNT1<32);  // Убираем неопределенность входа в прерывание (лишний джиттер)

   PORTB &= offOutsMask[0];      // стави все выходы PPM/PWM в 0
   PORTC &= offOutsMask[1];
   PORTD &= offOutsMask[2];

   if(PWM_enable) {  // генерим любые импульсы только после разрешения
     Servo_Number++;  // jump to next servo
     if(Servo_Number > reciever_outs) { // back to the first servo 
       total_ppm_time = 0; // clear the total servo ppm time
       Servo_Number=0;
     }
 
     if(Servo_Number == reciever_outs) {    // После последнего, выводим только межканальную паузу
        //Servos accepting 50hz ppm signal, this is why we are waiting for 20ms before second signal brust. 
        if(total_ppm_time < ppmPwmCycleTime-6000) us = ppmPwmCycleTime - total_ppm_time; //wait for total 20ms loop.  waiting time = 20.000us - total servo times
        else us=6000;                      // если сумма импульсов болше 20 мс, обеспечиваем 3 мс паузу, растягивя цикл                                
     } else {
       char i=Servo_Number;                // i  - номер воспроизводимого канала, он не всегда равен порядковому номеру импульса
       if(receiver_mode == 2) i += pwm1chnl-1;   // это номер воспроизводимого канала в режиме SBUS
       us = Servo_Position[i];            // берем ширину импульса из нужного канала
       total_ppm_time += us;              // добавляем ее к общей сумма
 
       // а теперб вычислим куда его выводить
       if (receiver_mode==0) {             // Parallel PWM, индекс выхода совпадает с номером импульса
         i=Servo_Number; 
       } else if(receiver_mode == 1) {     // Serial PPM, до 10 импульсов
          i=Servo_Number+4-pwm1chnl;       // остальные PWM каналы, определяются смещением
          if(i<3) i=16;                    // пока не дошли, до 4-го канала ставим невоспроизводимый индекс 
       } else {                            // режим SBUS, 
          i=Servo_Number+3;                // PWM каналы выводятся на 4,5,6,7 выходы, определяются смещением
       }
       if(i < sizeof(portMask)) {          // выводим текущий канал на заданный смещением вывод
         *portAddr[i] |= portMask[i];     
       }
     }
     if(receiver_mode==1 && Servo_Number <= MAX_PPM_OUT) { // формируем начало PPM импулmcа
       Serial_PPM_OUT_HIGH;           // ставим 1-ку на PPM выходе
       while(TCNT1 <500);             // !!!!! Не очень хорошая идея, паузы но пока так
       us-=503;                       // вычитаемм сделанную паузу, с небольшим запасом
       Serial_PPM_OUT_LOW;            // ставим 1-ку на PPM выходе
     }
  } else us=ppmPwmCycleTime;    // обеспечиваем холостой цикл
  
  TCNT1 = ppmPwmCycleTime - us; // configure the timer interrupt for X micro seconds     
}

// Вывод RSSI с усреденнием
//
#if defined(Analog_RSSI) 

static word curAvr=0;

void OutRSSI(byte val, byte weight)
{
  byte avr,navr=RSSIreg[0];     // тип вывода или степень усреднения
  
  if(navr == 0) {               // режим би-би
    if(val == 0) avr=127;       // пищим, когда пакет потерян
    else avr=0; 
  } else {                      // режим усреденеия
    curAvr=curAvr-curAvr/navr + val*weight;
    avr=curAvr/navr;
  }
   analogWrite(RSSI_OUT,avr);
   lastRSSI=avr;
}
#endif

void dOutsInit()               // инициализация дискретных выходов
{
  word pi;
  
  if(Regs4[6]) {
     for(byte i=0; i<8; i++) {   // до 8-им дискретных выходов
       if(Regs4[6] & (1<<i)) {   // если они есть...
         pi=0;
         if(portAddr[i] == &PORTC) pi=1;  // вычислим индекс порта (как оказалос они не попорядку)
         if(portAddr[i] == &PORTD) pi=2;
         
         offOutsMask[pi] |= diskrMask[i]; // запрещаем данную ногу уходить в 0
         portMask[i] = 0;                                // запрещаем данную ногу, как выход PWM
       } 
     } 
   }
}

unsigned char read_8bit_data(void); 
void to_ready_mode(void); 
void send_8bit_data(unsigned char i); 
void send_read_address(unsigned char i); 
void _spi_write(unsigned char address, unsigned char data); 
void RF22B_init_parameter(void); 

void port_init(void);   
unsigned char _spi_read(unsigned char address); 
void Write8bitcommand(unsigned char command); 
void to_sleep_mode(void); 

char htxt2[] PROGMEM = "Press 'm' to start MENU in 10 sec";

//############ MAIN LOOP ##############
void loop() 
{
  unsigned char prevPR=0;
  unsigned char i, j, afc_counter=0;
  unsigned char crc, first_data = 0;
  unsigned char sfsFlag=0;     // признак что последний пакет содержал признак записи FS 
  int temp_int, afc_avr=0;
  long tdif, btime;
  int next_time;               // время ожидания следующего пакета
  
  receiver_mode = check_modes(PPM_MODE_JUMPER); // режим PPM
  if(receiver_mode) {
     reciever_outs=MAX_PPM_OUT;                 // в режиме PPM 
  } else if(check_modes(SBUS_MODE_JUMPER)) {    // проверяем на SBUS
     receiver_mode=2;
     reciever_outs=MAX_SBUS_OUT;
     ppmPwmCycleTime=28000;
     ICR1 = ppmPwmCycleTime;                    // used for TOP, makes for 50 hz

     portMask[0] = 0;                           // закроем pin1 для SBUS оут
     offOutsMask[SBUS_OUT_PORT]  |= SBUS_OUT_BIT;   
     SBUS_OUT_HIGH;
  }
  
  satFlag=check_modes(SAT_MODE_JUMPER);       // проверим на режим саттелита
  
  Red_LED_Blink(1); // Red LED blink

  if(check_modes(REBIND_JUMPER)) makeBind();    // данный джампер, означает режим поиска и привязки к передатчику
  Serial.begin(SERIAL_BAUD_RATE); //Serial Transmission 
  if(!satFlag) {
    printHeader();
    Red_LED_Blink(1); // Red LED blink
  }
        
  eeprom_check(); 
  beaconFcorr=Regs4[2];
  statInit();                // инициализируем статистику
  dOutsInit();               // инициализируем дискреные выходы
  
  load_failsafe_values();   // загрузим дефолты каналов
  RF22B_init_parameter();   // инициализируем RFMку

  PWM_enable=0;
  wdt_enable(WDTO_1S);     // запускаем сторожевой таймер 

  if(!satFlag) {           // 

    Serial.print("S/N=");  Serial.println(Regs4[0]);
    showRegs();            // отобразим регистры (в режиме саттелита не перегружаем буффер)
  
    Serial.print("IRQ="); 
    Serial.println(_spi_read(0x04)&1); 

    Serial.println("START"); 
    Serial.println(_spi_read(0x00));   // тип RFMки
    Serial.println(_spi_read(0x01));   // версия RFMки
    Serial.println(_spi_read(0x02));   // и ее состояние
  
    Serial.print("T="); 
    Serial.println(_spi_read(0x11)-0x40);  // читаем температуру из АЦП
 
    printlnPGM(htxt2);  // реальное время задано константой
    
    if(receiver_mode == 1)  Serial.println("PPM out"); 
    if(receiver_mode == 2)  Serial.println("SBUS out"); 
 }
  
hotRest:
  afc_counter=afc_avr=0;
  search_mode=1;
  rx_reset();
  to_rx_mode(); 
  sei();
  RF_Mode = Receive;    // First packet wait
	
  start_time=time = millis();
  
  last_pack_time = time; // reset the last pack receiving time for first usage
  last_hopping_time = time; // reset hopping timer
 
  while(1) {    /* MAIN LOOP */

    wdt_reset();              // поддержка сторожевого таймера
 
    if (_spi_read(0x0C)==0) {  // detect the locked module and reinit
       RF22B_init_parameter(); 
       to_rx_mode(); 			 
       if(!satFlag) Serial.println("FiErr!");
    }

// Секция приема и обработки нового пакета                        
    if(RF_Mode == Received) {  // Прошло рперывание принятого пакетоа
       Red_LED_OFF;  Green_LED_ON;
                                 
       send_read_address(0x7f); // Send the package read command

       for(i = 0; i<RF_PACK_SIZE; i++) { //read all buffer 
	  RF_Rx_Buffer[i] = read_8bit_data(); 
       }  
       rx_reset();            
                                
       crc=CRC8(RF_Rx_Buffer+2,RF_PACK_SIZE-3); // проверяем принятый пакет 
       if(crc != 0) {
         if(!satFlag) Serial.println("CRC!");    // Ошибка CRC пакета
       } else if(RF_Rx_Buffer[0] != Regs4[1]) {   // проверка номера линка 
         if(!satFlag) Serial.println("BIND!");   // ошибка номера линка
         crc=0xff;                               // crc=0 как флаг нормального пакета
       }  else {                                 
         Buf_To_Servo(RF_Rx_Buffer);             // преобразуем в PWM

	 if(RF_Rx_Buffer[RF_PACK_SIZE-1] == 0x1)  { //Set Failsafe
           if(!sfsFlag) {
             if(!satFlag) Serial.println("FS W");
                save_failsafe_values();          // при первом появлении признака, пишем в EEPROM
                sfsFlag=1;
             }
           } else sfsFlag=0;
        }					 

        if(crc == 0) {                           // если пакет наш и целый
// Выдача статистики через UART
//				 				 
          if(N_RSSI) Rx_RSSI /= N_RSSI;           // вычислим средний RSSI 
          N_RSSI=Rx_RSSI;                         // копия для телеметрии
          if(N_pause) prevPR=(Pause_RSSI /= N_pause);      // вычислим средний шум в паузе
          else Pause_RSSI=prevPR;                 // ингода не успеваем 
          
          if(Pause_RSSI > Rx_RSSI) Rx_RSSI=0;     // и вычисляем отношение C/Ш
          else Rx_RSSI-=Pause_RSSI;               // вместо простого РСИИ
                                 
          if(RSSIreg[1]) lastRSSI=Rx_RSSI;        // RSSI level
          else lastRSSI=N_RSSI;                   // or s/n ratio
          OutRSSI(lastRSSI,1);                    // выводим RSSI
				 
          temp_int=_spi_read(0x2B);              // читаем отклонение частоты

          #if defined(Serial_RSSI)
          if(!satFlag) {
            Serial.print("R=");   Serial.print(N_RSSI);
            Serial.print(" S=");   Serial.print(statMin);  // номер записи сохраняемой статистикм
            Serial.print(" C=");   Serial.print(hopping_channel+1);
            Serial.print(" A=");   Serial.print(temp_int);
            Serial.print(" Rn=");  Serial.println(Pause_RSSI);  // уровень шума
/***************************************
            if(hopping_channel == 0) {                     // отладка !!!!
              for(i=0; i<RC_CHANNEL_COUNT; i++) {
                  Serial.print(Servo_Position[i]);
                  Serial.write(' ');
             }
             Serial.println();
            }
*****************************************/            
          }
          #endif
          curStat.rssi[hopping_channel] += N_RSSI;     // для статистики
          curStat.noise[hopping_channel] += Pause_RSSI;
          statCntr[hopping_channel]++;                 // считаем циклы
       
// 
// Подстройка частоты
         if(Regs4[2] != 0) {                          // в ручном режиме не работает  
           if(temp_int > 127) temp_int=temp_int-256;
           if(temp_int < -25 || temp_int > 25) temp_int=0;     // нереальные значения игнорируем
           else temp_int = temp_int * 16;
           afc_counter++;
        
           afc_avr = afc_avr-afc_avr/16 + temp_int/16;         // усредняем методом скользящего среднего
           if(abs(afc_avr) > (AFC_POROG*16) && afc_counter > 32) {
             afc_avr = -(afc_avr/16);
             Regs4[2]+=afc_avr;
             _spi_write(0x09,Regs4[2]);                       // меняем подстройку частоты   
             if(!satFlag) { Serial.print("Fcorr=");   Serial.println(afc_avr); }
             afc_counter=afc_avr=0;
           }
         } else lastPackBad=1;        // флажок битого пакета для SBUS

       	 beacon_flag=failsafe_mode = 0; // deactivate failsafe mode
         self_pack_time=last_beacon_time=last_pack_time=time=millis(); // record last package time
         search_mode=0;      // отменяем режим поиска
         rl_counter=0;

         lastPackBad=0;        // хороший пакет 
         Direct_Servo_Drive(); // use stick commands directly for standard rc plane flights
         if(satFlag) {         // поддержка режима сателита
           RF_Rx_Buffer[0]=SAT_PACK_HEADER;     // используем входной буфер, для отправки
           Serial.write(RF_Rx_Buffer,SAT_PACK_LEN);  // Отсылаем пакет
         }
       }
                                 
extern unsigned long rTime;
       Hopping(); //Hop to the next frequency
       last_hopping_time = rTime;      // берем реальное время приема пакета
                               
       RF_Mode = Receive;              // запускаем новый прием
       Green_LED_OFF;
       sendSbus();                     // поддержка цикла отправки SBUS
       delayMicroseconds(499);         // что-бы гарантировать RSSI
       continue;
     }

     time = millis();            // текущее время
     tdif=time - last_hopping_time;  // время с момента последнего ереключения частоты (приема пакета)
     sendSbus();               // поддержка цикла отправки SBUS

     if(doFrecHandCorr()) goto hotRest;  // поддержка ручной подстройки частоты
     if(menuFlag && time-start_time < MENU_WAIT_TIME) {   // даем 10 сек на вход в меню
       if(checkMenu()) {        // реализуем возможность входа в меню
         doMenu(); 
         RF22B_init_parameter(); 
         goto hotRest; 
       } 
     } else tryRecvSat();        // в остальное время принимаем пакеты от саттелитов
       
//
// Если данные сателита более актуальны чем свои
     if(satRecFlag && lastSatTime > self_pack_time+33) {
          Buf_To_Servo(satIn);        // обрабатываем пакет
          Direct_Servo_Drive();       // выводим на сервы
          satRecFlag=0;
    	  beacon_flag=failsafe_mode = 0; // deactivate failsafe mode
          last_beacon_time=last_pack_time = time;
          if(!satFlag) { Serial.println("$SAT");  }
          else Serial.write(satIn,SAT_PACK_LEN);  // Отсылаем пакет следующему в цепочке
     }

     // Обработка ситуации FS
     if ((time-last_pack_time > TIME_TO_FS) && (failsafe_mode == 0) && PWM_enable)  {
          if(!satFlag) Serial.println("to fs");
  	  failsafe_mode = 1; // Activate failsafe mode
          last_beacon_time=time;
          load_failsafe_values(); // Load Failsafe positions from EEPROM
          Direct_Servo_Drive(); // Set directly the channels form Servo buffer
          Red_LED_OFF;
          OutRSSI(0,8);        // нет связи, нет RSSI
          curStat.FS++;        // для статистики 
      } 

        // в зависимости от того сколько прошло ждем пакеты синхроннно или асинхронно
      if(time-self_pack_time > TIME_TO_SEARCH) search_mode=1;
      if(tdif > 0 && tdif < 5) {              // первые 5 мс меряем уровень шума
         delayMicroseconds(99);
	 Pause_RSSI += _spi_read(0x26);       // Read the RSSI value
         N_pause++;                           // для усреднения
      }
      if(search_mode) next_time = 255;         // включаем режим поиска
      else {                                   // или продолжнаем ловить в заданное время
        next_time = 35;	
// Операции во время ожидания пакета
        if(tdif > 10 && tdif < 21) {          // где-то в середине пакета читаем RSSI 
          delayMicroseconds(99);
          Rx_RSSI += _spi_read(0x26);         // Read the RSSI value
          N_RSSI++;                           // для усреднения
        } else if(tdif < 25) {
          statLoop();                         // в безопасное время работаем со статиcтикой, которая может сожрать до 4 мс
        }
      }
                        
// Переход к новому каналу, если пакет не получен в заданное время
       if(tdif >= next_time) {  //automatic hopping for clear channel when rf link down for 32ms.	
         if(search_mode == 0) {
           if(!failsafe_mode)  Red_LED_ON;   // зажигаем карсный для индикации потреи

            if(hopping_channel&1) last_hopping_time += 32;  // Что-бы не терять синхронизацию  
            else last_hopping_time += 31;                  // добавляем 31.5 мс в среднем
         } else last_hopping_time=time;
                               
         OutRSSI(0,search_mode*7+1);                       // выводим 0-й RSSI
          
         if(!satFlag) {
             Serial.print("$RL");
             Serial.print(++rl_counter);
          #if defined(Serial_RSSI)
             if(N_pause) prevPR= (Pause_RSSI/=N_pause);      // вычислим средний шум в паузе
             else Pause_RSSI=prevPR;
             Serial.print(" S=");   Serial.print(statMin); 
             Serial.print(" C=");   Serial.print(hopping_channel+1);
             Serial.print(" Rn=");  Serial.print(Pause_RSSI);  // уровень шума
          #endif
             Serial.println();
         } 
         curStat.lost[hopping_channel]++;            // для статистики
         lastPackBad=1;                              // флажок битого пакета для SBUS              
         Hopping(); //Hop to the next frequency
      }  
                              

//
//  Маяк при отсутствии связи
                    
    if(failsafe_mode) {
       statLoop();   
       tdif=time-last_beacon_time; 
       btime=BeaconReg[5]; btime *= 1000; 
       if((beacon_flag && tdif > BEACON_INTERVAL) || tdif > btime) {  // С интервалом в 7 секунд после первой паузы
          beacon_flag=1; 
          if(BeaconReg[5] != 0xff) {                                 // если маяк не запрещен
            if(!satFlag) Serial.println("SOS"); 
            beacon_send();                                           // Маяк посылает 2-х секундные посылки по 4 тона
          }
          if(!satFlag) Serial.println("Init"); 
          RF22B_init_parameter();   // go back to normal RX
          rx_reset();
          last_beacon_time=time;
       }
    } 

  }  // --------------------------------------- конец основного цикла ----------------------------------------
	 
}

