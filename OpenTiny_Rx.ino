// **********************************************************
// ******************   Open Tiny Rx Code   *******************
// *** Based on original code from Melih Karakelle on 2010-2011  ***
// **  an Arudino based RC Rx/Tx system with extra futures **
// **       This Source code licensed under GPL            **
// **********************************************************
// Version Number     : 1.12
// Latest Code Update : 2012-03-21
// Supported Hardware : OpenLRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************
// # PROJECT DEVELOPERS # 
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne) 
// thUndead (forum nick name: thUndead) 
// Modifyed Baychi to compatable with Expert 2G/Tiny LRS


#include "config.h"

#include <EEPROM.h>

// Функции меню терминала
//
static unsigned char menuFlag=1;              // флаг, разрешающий меню
static unsigned char regs[] = {1, 2, 3, 11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,28,40,41,42 } ;
static char *help[] = {
  "Bind N",
  "Freq Corr",
  "Statistics enable",
  "Hope F1",
  "Hope F2",
  "Hope F3",
  "Hope F4",
  "Hope F5",
  "Hope F6",
  "Hope F7",
  "Hope F8",
  "Beacon F (FF=disable)",
  "Beacon P1",  
  "Beacon P2",
  "Beacon P3",
  "Beacon P4",
  "Beacon start time (sec)",
  "SAW Fmin",
  "SAW Fmax",
  "PPM mode 1st PWM chnl (1-8) [4]", 
  "RSSI type: sound(0)/level(1)",
  "RSSI mode: level(0)/SN ratio(1)",
  "RSSI over PWM (ch num:1-12) 0- not use"
};  
  

void showRegs(void)         // показать значения регистров
{
  unsigned char i,j=0;
  for(int i=1; i<=REGS_NUM; i++) {
    if(regs[j] == i) {
      Serial.print(i);
      Serial.print("=");
      Serial.print(read_eeprom_uchar(i));
      Serial.print("\t");
      Serial.println(help[j]);
      j++;
    }
  }
}


int checkMenu(void)   // проверка на вход в меню
{
   int in; 
   
   if (Serial.available() > 0) {
      in= Serial.read();             // все, что пришло, отображаем
      if(in == 'm' || in == 'M') return 1; // есть вход в меню
   } 
   return 0;                        // не дождались 
}


void getStr(char str[])             // получение строки, завершающейся Enter от пользователя
{
  int in,sn=0;
  str[0]=0;
  while(1) {
    if (Serial.available() > 0) {
      if(Serial.peek() == SAT_PACK_HEADER) {  // если обнаружили заголовок от саттелита
        str[0]='q'; str[1]=0;      // иммитируем Quit
        menuFlag=0;              // запрещаем меню
        return;
      }
       in= Serial.read();             // все, что пришло, отображаем
       if(in > 0) {
          Serial.write(in);
          if(in == 0xd || in == 0xa) {
            Serial.println("");
            return;                     // нажали Enter
          }
          if(in == 8) {                 // backspace, удаляем последний символ
            if(sn) sn--;
            continue;
          } 
          str[sn]=in; str[sn+1]=0;
          if(sn < 6) sn++;              // не более 6 символов
        }
     } else delay(1);
  }
}

void doMenu()                       // работаем с меню
{
  char str[8];
  int reg,val;
  Serial.println("To Enter MENU Press ENTER");
  getStr(str);
  if(str[0] == 'q' || str[0] == 'Q') return;     // Q - то quit
  
  while(1) {
    Serial.println("Rg=Val \tComments -----------------");
    showRegs();
    Serial.println("Type Reg and press ENTER, type Value and press ENTER (q=Quit, ss/sl/sa=Stat)");

rep:  
    getStr(str);
    if(str[0] == 's' || str[0] == 'S') {
      if(str[1] == 'e') statErase();
      else statShow(str[1]);  // Печать статистики
      goto rep;
    }
    
    if(str[0] == 'q' || str[0] == 'Q') return;     // Q - то quit
    reg=atoi(str);
    if(reg<0 || reg>REGS_NUM) continue; 

    getStr(str);
    if(str[0] == 'q' || str[0] == 'Q') return;     // Q - то quit
    val=atoi(str);
    if(val<0 || val>255) continue; 
    if(reg == 0 && val ==0) continue;              // избегаем потери s/n

    Serial.print(reg); Serial.print("=");   Serial.println(val);  // Отобразим полученное
    
     write_eeprom_uchar(reg,val);  // пишем регистр
     read_eeprom();                // читаем из EEPROM    
     write_eeprom();               // и тут-же пишем, что-бы сформировать КС 
  }    
}  
//---------------------------------------------------------------------------
//
// Функции для приемников саттелитов
//
void tryRecvSat(void)
{
  unsigned char in;  
  signed char i,n;
  
  if(time-lastSatTime < SAT_AFTER_TIME) return;    // даем возможность обратать прежний пакет

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
        
        Serial.begin(SERIAL_BAUD_RATE); //Serial Transmission 
//        Wire.begin(); //I2C Transmission
        
        
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
  
  while (TCNT1<32);  // Убираем неоперделенность входаа в прерывание (лишний джиттер)

  Servo_Ports_LOW;
 
  if(PWM_enable) {  // генерим любые импульсы только после разрешения
    Servo_Number++; // jump to next servo
    if (Servo_Number > reciever_outs) { // back to the first servo 
      total_ppm_time = 0; // clear the total servo ppm time
      Servo_Number=0;
    }
 
    if(Servo_Number == reciever_outs) { // Check the last servo number. 
        //Servos accepting 50hz ppm signal, this is why we are waiting for 20ms before second signal brust. 
        if(total_ppm_time < 34000) us = 40000 - total_ppm_time; //wait for total 20ms loop.  waiting time = 20.000us - total servo times
        else us=6000;                      // если сумма импульсов болше 20 мс, обеспечиваем 3 мс паузу, растягивя цикл                                
    }  else {
      us = Servo_Position[Servo_Number]; // read the servo timing from buffer
      total_ppm_time += us; // calculate total servo signal times.
    }
 
    if (receiver_mode==0) { // Parallel PPM
      switch (Servo_Number) {
      case 0:
        Servo1_OUT_HIGH;
        break;
      case 1:
        Servo2_OUT_HIGH;
        break;
      case 2:
        Servo3_OUT_HIGH;
        break;
      case 3:
        Servo4_OUT_HIGH;
        break;
      case 4:
       Servo5_OUT_HIGH;
        break;
      case 5:
        Servo6_OUT_HIGH;
        break;
      case 6:
        Servo7_OUT_HIGH;
        break;
      case 7:
        Servo8_OUT_HIGH;
        break;
      case 8:
        Servo9_OUT_HIGH;
        break;  
      case 9:
        Servo10_OUT_HIGH;
        break;  
        }     
     } else { // Serial PPM over 3&4  channel and PWM at 5-10 ch
        switch (Servo_Number+4-pwm1chnl) {
         case 3:
          Servo4_OUT_HIGH;
         break;
         case 4:
          Servo5_OUT_HIGH;
         break;
         case 5:
          Servo6_OUT_HIGH;
         break;
         case 6:
          Servo7_OUT_HIGH;
         break;
        case 7:
          Servo8_OUT_HIGH;
          break;
        case 8:
          Servo9_OUT_HIGH;
          break;  
        case 9:
          Servo10_OUT_HIGH;
          break;  
        }     
        delayMicroseconds(250);         // !!!!! Не очень хорошая идея, но пока так
        us-=500;
        Serial_PPM_OUT_HIGH;
    }
  } else us=40000;    // обеспечиваем холостой цикл
  
  TCNT1 = 40000 - us; // configure the timer interrupt for X micro seconds     
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

//############ MAIN LOOP ##############
void loop() 
{
  unsigned char i, j, afc_counter=0;
  unsigned char crc, first_data = 0;
  unsigned char sfsFlag=0;     // признак что последний пакет содержал признак записи FS 
  int temp_int, afc_avr=0;
  unsigned char tempAfc=199;     // временная поправка частоты 

  int tdif, btime, next_time = 35;    // время ожидания следующего пакета
  
  receiver_mode = check_modes(0); // Check the possible jumper positions for changing the receiver mode.
  if(receiver_mode) {
     reciever_outs=10;        // RC_CHANNEL_COUNT;  // в режиме PPM 12 каналов
//     pinMode(Servo2_OUT, INPUT); // что-бы не было конфликта 
  }
  satFlag=check_modes(4);       // проверим на режим саттелита
  
  Red_LED_Blink(1); // 3x Red LED blinks for serial PPM mode.

  if(!satFlag) {
     Serial.println("Baychi soft 2013");
     Serial.print("RX Open Tiny V2 F"); Serial.println(version[0]);
  }
  eeprom_check(); 
  statInit();         // инициализируем статистику
  
  load_failsafe_values();   // Load failsafe values on startup
  RF22B_init_parameter(); // Configure the RFM22B's registers

  frequency_configurator(CARRIER_FREQUENCY); // Calibrate the RFM22B to this frequency, frequency hopping starts from here.
  PWM_enable=0;

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
 
    Serial.println("If need menu - press 'm' in 5 sec");  // реальное время задано константой
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

        if (_spi_read(0x0C)==0) {  // detect the locked module and reinit
           RF22B_init_parameter(); 
           to_rx_mode(); 			 
           if(!satFlag) Serial.println("FiErr!");
        }

	time = millis();  // текущее время
	tdif=time - last_hopping_time;  // время с момента последнего приема

        if(menuFlag && time-start_time < MENU_WAIT_TIME) {   // даем 5 сек на вход в меню
           if(checkMenu()) {        // реализуем возможность входа в меню
              doMenu(); 
              RF22B_init_parameter(); 
              goto hotRest; 
           } 
        } else tryRecvSat();        // в остальное время принимаем пакеты от саттелитов
       
        if(Regs4[2] == 0 && Serial.available() > 0) {   // ручная подстройка частоты при REG2=0
           i=Serial.read();
           if(i == 0xd) {            //
              Regs4[2]=tempAfc;
              write_eeprom();
              Serial.println("Fixed!");
              RF22B_init_parameter(); 
              goto hotRest; 			 
           }
           if(i == '>' || i == '.' || i==238) tempAfc++;
           else if(i == '<' || i == ',' || i==161) tempAfc--;
           else continue;
           
           if(tempAfc > 255) tempAfc=1;
           if(tempAfc < 1) tempAfc=255;
           _spi_write(0x09, tempAfc);  // подстройка частоты
           Serial.println(tempAfc);
        }

	//Detect the broken RF link and switch it to failsafe mode after 1 seconds  
        // Обработка ситуации FS
	if ((time-last_pack_time > TIME_TO_FS) && (failsafe_mode == 0) && PWM_enable)  {
          if(!satFlag) Serial.println("to fs");
  	  failsafe_mode = 1; // Activate failsafe mode
          last_beacon_time=time;
          load_failsafe_values(); // Load Failsafe positions from EEPROM
          Direct_Servo_Drive(); // Set directly the channels form Servo buffer
          Red_LED_OFF;
          #if defined(Analog_RSSI) 
              analogWrite(RSSI_OUT,0); 
 	  #endif
          curStat.FS++;        // для статистики 
        } 

        // в зависимости от того сколько прошло ждем пакеты синхроннно или асинхронно
        if(time-self_pack_time > TIME_TO_SEARCH) search_mode=1;
	if(search_mode) next_time = 255;         // включаем режим поиска
        else {                                   // или продолжнаем ловить в заданное время
           next_time = 35;	
// Операции во время ожидания пакета
           if(tdif  < 5) {                  // первые 5 мс меряем уровень шума
             delayMicroseconds(299);
	     Pause_RSSI += _spi_read(0x26);       // Read the RSSI value
             N_pause++;                           // для усреднения
          } else if(tdif > 9 && tdif < 19) {            // где-то в середине пакета читаем RSSI 
             delay(1);
             Rx_RSSI += _spi_read(0x26);         // Read the RSSI value
             N_RSSI++;                           // для усреднения
           } else if(tdif < 23) statLoop();      // работаем со статичтикой, которая может сожрать до 4 мс
       }
                        
// Переход к новому каналу, если пакет не получен в заданное время
       if (tdif >= next_time) {//automatic hopping for clear channel when rf link down for 32ms.	
          if(search_mode == 0) {
            if(!failsafe_mode)  Red_LED_ON;   // зажигаем карсный для индикации потреи

            if(hopping_channel&1) last_hopping_time += 32;  // Что-бы не терять синхронизацию  
            else last_hopping_time += 31;                  // добавляем 31.5 мс в среднем
          } else last_hopping_time=time;
                               
          #if defined(Analog_RSSI) 
           if(RSSIreg[0]) analogWrite(RSSI_OUT,0);         // режим RSSI
           else analogWrite(RSSI_OUT,127);                // режим би-би 
  	  #endif
                               
          if(!satFlag) {
             Serial.print("$RL");
             Serial.println(++rl_counter);
          } 
                      
          curStat.lost[hopping_channel]++;                // для статистики
          
          Hopping(); //Hop to the next frequency
      }  
                              
// Секция приема и обработки нового пакета                        
     if(RF_Mode == Received) {  // RFM22B INT pin Enabled by received Data
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
                  save_failsafe_values();     // при первом появлении признака, пишем в EEPROM
                  sfsFlag=1;
               }
            } else sfsFlag=0;
         }					 

        if(crc == 0) {                        // если пакет наш и целый
// Выдача статистики через UART
//				 				 
          if(N_RSSI) Rx_RSSI /= N_RSSI;         // вычислим средний RSSI 
          N_RSSI=Rx_RSSI;                       // копия для телеметрии
          if(N_pause) Pause_RSSI /= N_pause;      // вычислим средний шум в паузе
          if(Pause_RSSI > Rx_RSSI) Rx_RSSI=0;     // и вычисляем отношение C/Ш
          else Rx_RSSI-=Pause_RSSI;               // вместо простого РСИИ
                                 
          #if defined(Analog_RSSI) 
            if(RSSIreg[1]) lastRSSI=Rx_RSSI;   // RSSI level
            else lastRSSI=N_RSSI;               // or s/n ratio
            if(RSSIreg[0]) analogWrite(RSSI_OUT,lastRSSI);      // режим RSSI
            else analogWrite(RSSI_OUT,0);    // режим би-би
          #endif
				 
          temp_int=_spi_read(0x2B);   // читаем отклонение частоты

          #if defined(Serial_RSSI)
          if(!satFlag) {
            Serial.print("R=");   Serial.print(N_RSSI);
            Serial.print(" S=");   Serial.print(statMin);  // номер записи сохраняемой статистикм
            Serial.print(" C=");   Serial.print(hopping_channel+1);
            Serial.print(" A=");   Serial.print(temp_int);
            Serial.print(" Rn=");  Serial.println(Pause_RSSI);  // уровень шума
          }
          #endif
          curStat.rssi[hopping_channel] += N_RSSI;     // для статистики
          curStat.noise[hopping_channel] += Pause_RSSI;
          if(hopping_channel == 7) statCntr++;         // считаем циклы
        
// 
// Подстройка частоты
          if(Regs4[2] != 0) {                          // в ручном режиме не работает  
            if(temp_int > 127) temp_int=temp_int-256;
            if(temp_int < -20 || temp_int > 20) temp_int=0;     // нереальные значения игнорируем
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
          }

       	  beacon_flag=failsafe_mode = 0; // deactivate failsafe mode
          self_pack_time=last_beacon_time=last_pack_time = time; // record last package time
          search_mode=0;      // отменяем режим поиска
          rl_counter=0;

          Direct_Servo_Drive(); // use stick commands directly for standard rc plane flights
          if(satFlag) {         // поддержка режима сателита
            RF_Rx_Buffer[0]=SAT_PACK_HEADER;     // используем входной буфер, для отправки
            Serial.write(RF_Rx_Buffer,SAT_PACK_LEN);  // Отсылаем пакет
          }
        }
                                 
        Hopping(); //Hop to the next frequency
        delay(1);
                                
        RF_Mode = Receive;              // Start next packet wait
        last_hopping_time = time;    
        Green_LED_OFF;
     }
//
// Если данные сателита более актуальны 
      if(satRecFlag && lastSatTime > self_pack_time+33) {
          Buf_To_Servo(satIn);        // обрабатываем пакет
          Direct_Servo_Drive();       // выводим на сервы
          satRecFlag=0;
    	  beacon_flag=failsafe_mode = 0; // deactivate failsafe mode
          last_beacon_time=last_pack_time = time;
          if(!satFlag) { Serial.print("$SAT");   Serial.println(""); }
          else Serial.write(satIn,SAT_PACK_LEN);  // Отсылаем пакет следующему в цепочке
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
            beacon_send();                                           // Маяк посылает 2-х серкундные посылки по 4 тона
          }
          if(!satFlag) Serial.println("Init"); 
          RF22B_init_parameter();   // go back to normal RX
          rx_reset();
          last_beacon_time=time;
       }
    } 

  }  // --------------------------------------- конец основного цикла ----------------------------------------
	 
}

