// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Reciever with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny/2G RX, Orange/OpenLRS Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenTinyRX
// **********************************************************

// Версия и номер компиляции. Используется для проверки целостности программы
// При модификации программы необходимо изменить одно из этих чисел 
unsigned char version[] = { 6, 1 };

//####### RX BOARD TYPE #######
// 1 = Rx 2G/Tiny original Board
// 2 = Rx Open/orange v2 Board
#define RX_BOARD_TYPE 1

//######### DEBUG MODES ##########
// 0 = No Debug Output
// 1 = Servo Position values 
#define DEBUG_MODE 0

// Время удержания синхронизации прыжков после потери связи
#define TIME_TO_SEARCH 2999
// Время от потери связи, до ухода в FS (не делать меньше 250 мс)
#define TIME_TO_FS 599
// Интервал между сериями писков маяка
#define BEACON_INTERVAL 4999
// Время для входа в меню
#define MENU_WAIT_TIME 9999

//######### TRANSMISSION VARIABLES ##########
//!!! These values configurable over PC with OpenLRS Configurator Software
//If you are using older than v1.12 firmware, load the new firmware first, 
//then open the configurator software when FTDI cable connected. 
//And press Set Dafaults button for configuring your device EEPROM

#define CARRIER_FREQUENCY  433075  // 433Mhz startup frequency !!! не менять
#define HOPPING_STEP_SIZE  6 // 60kHz hopping steps
#define HOPE_NUM          8 /* number of hope frequensies */ 
#define AFC_POROG         4 /* предельное отклонение частоты, требующее коррекции */

//###### HOPPING CHANNELS #######
//Каналы прыжков (регистры 11-18) Select the hopping channels between 0-255
//Frequency = CARRIER_FREQUENCY + (StepSize(60khz)* Channel_Number) 
static unsigned char hop_list[HOPE_NUM] = {77,147,89,167,109,189,127,209};   // по умолчанию - мои частоты

// Четыре первых регистра настроек (S/N, номер Bind, поправка частоты, номер сервы расширения, разрешение статистики 
static unsigned char Regs4[5] = {99 ,72, 204, 0, 1 };  

// Регистры поддержки SAW фильтра (25,26) задают границы частот, внутри которых фильтр включен (GPIO2=1)
static unsigned char  SAWreg[2] =   {75, 210 };  
// Регистры маяка (19-24): частота, мошность1 - мощность 4, пауза перед первым писком (сек)
// При неотключаемом SAW, неообходимо ограничить мощность 10 мВт и внести частоту маяка в полосу фильтра
static unsigned char  BeaconReg[6] =   { 101, 4, 2, 1, 0, 30 };  
static unsigned char pwm1chnl = 4;     // номер первого PWM канала в комбинированном режимме.

// Регистры RSSI (40-42). Задают тип (биби/Вольты) и режим (уровень сигнала или отношение сигнал/шум).
// RSSIreg[2] - вывод RSSI через PWM выход (1-8)
static unsigned char  RSSIreg[3] =   { 1, 0, 0 };  

static unsigned char menuFlag=1;              // Флаг, разрешающий работу с меню

//###### SERIAL PORT SPEED #######
#define SERIAL_BAUD_RATE 38400    // как у Эксперта
#define REGS_NUM 42               // номер последнего используемого регистра настроек

//###### RSSI MODES ########
// Analog RSSI pin is Ch0 on v2 receivers. If you enable this function CH0 works as PWM RSSI output.
#define Analog_RSSI    

//Serial RSSI is transmitting the RSSI value over serial port. You can use this function for debugging.
#define Serial_RSSI //Serial RSSI value for analyzing

// Channel Names and Numbers
#define AILERON 0
#define ELEVATOR 1
#define THROTTLE 2
#define RUDDER 3
#define RETRACTS 4
#define FLAPS 5
#define AUX1 6
#define AUX2 7
#define AUX3 8
#define AUX4 9
#define AUX5 10
#define AUX6 11

#define RF_PACK_SIZE 16                 /* размер данных в пакете */
#define RC_CHANNEL_COUNT 12             /* количество каналов управления и импульсов на PPM OUT */
#define PWM_OUT_NUM 10                   /* количество PWM выходов */

unsigned char RF_Rx_Buffer[RF_PACK_SIZE];
unsigned int Servo_Buffer[RC_CHANNEL_COUNT] = {3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};	//servo position values from RF
unsigned int Servo_Position[RC_CHANNEL_COUNT] = {3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};	//real servo position values
static unsigned char Servo_Number = 0;
unsigned int total_ppm_time=0;
unsigned int rl_counter=0;
unsigned short Rx_RSSI, N_RSSI, Pause_RSSI, N_pause;       // переменные для вычисления RSSI
unsigned int lastRSSI=0;                                 

static unsigned char receiver_mode = 0, reciever_outs=PWM_OUT_NUM;  // режим работы (PWM/PPM) и количкство вых каналов 10/12
static unsigned char hopping_channel = 0;

unsigned char PWM_enable=0;      // запрет на генерацию импульсов до приема первого целого пакета 

unsigned long time,start_time;   // текущее время в мс и время старта
unsigned long last_pack_time ;   // время  последнего хорошего пакета
unsigned long last_hopping_time; // время последнего прыжка 
unsigned long last_beacon_time;  // время последнего писка маяка   
unsigned long self_pack_time;    // время последнего целого пакета от своей RFMки 
unsigned char beacon_flag = 0;   // признак первого писка
unsigned char failsafe_mode = 0; //Falsafe modes  0 = Deactive, 1 = Active
unsigned char search_mode =  1;  // Флаг медленного поиска первого пакета 

volatile unsigned char RF_Mode = 0;  /* для RFMки */
#define Available 0
#define Transmit 1
#define Transmitted 2
#define Receive 3
#define Received 4

//--------------------------------------------------------------------------
// Статистика полета за интервал (типично - минута)

#define STAT_INTERVAL 60            /* время накопления записи статистики 1-60 сек */
unsigned int  statCntr[HOPE_NUM],statMin=0;   // счетчик очередной записи статистики и минут 

struct curStatRec {                 // Текущая накапливаемая статистика
   unsigned char flightNum;         // номер полета (инкрементируется при каждом включении)
   unsigned char FS;                // количество провалов связи и FS в момент сохранения (старший бит)
   unsigned char lost[HOPE_NUM];    // количество потерянных пакетов за интервал (в миинуту макс 238) FF, FE - все - нет потерь (что-бы не насиловать FLASH)
   unsigned int rssi[HOPE_NUM];     // средний уровень RSSI за интервал
   unsigned int noise[HOPE_NUM];    // средний уровень шума за интервал
} curStat;                          // текущая статистика 

// Функции статистики
void statLoop(void);                  // цикл поддержки 
void statInit(void);                  // инициализация статистики в начале работы
void statShow(void);                  // вывести статистику на экран
void statErase(void);                 // стирание статистики

//-----------------------------------------------------------------
//
//  Поддержка приемников сателлитов
//  Обмен через UART на 38400 (задержка 3.5 мс) или 115200 (задержка 1 мс)
//  Формат посылки 16 байт похож на обычный пакет: 0x16 байт_страших_бит 12байт_каналов CRC8 управляющий_байт
//  Сателитом становится приемник, с перемычкой на каналах 9-10 (I2C). Он не выдает статистику через UART, вместо этого выдает полученные пакеты
//  Все приемники (в том числе и саттелиты) после первых 5 сек (для входа в меню) будут готовы принимать пакеты саттелита через UART. 
//  Если свой пакет не принят, но принят пакет от саттелита, он используется вместо совего. Саттелит также передаст его дальше, что делает возможным 
//  каскадное подключение до 4-х саттелитов к основному приемнику. 
//
#define SAT_PACK_LEN 16                 /* размер пакета от сателитов */
#define SAT_PACK_HEADER 0x16            /* признак посылки от саттелита */
#define SAT_AFTER_TIME  20              /* время в течении которого новые данные не извлекаются, давая возможность обработать принятое */

unsigned char satIn[SAT_PACK_LEN];      // буфер входного пакет от внешнего саттелита
unsigned char satCntr, satFlag=0;         // счетчик принимаемых байт и признак работы в режиме сателлита
unsigned char satRecFlag=0;             // признак актуального пакета в буфере
unsigned long lastSatTime=0;            // время приема последнего пакета от сателлита


#if (RX_BOARD_TYPE==1)           // Expert original reciever
    //## RFM22B Pinouts for Rx v1 Board
    #define SDO_pin 12
    #define SDI_pin 11        
    #define SCLK_pin 13 
    #define IRQ_pin 2
    #define nSel_pin A0
    #define IRQ_interrupt 0
        
    #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
    #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
    #define  nSEL_on PORTC |= 0x01  //A0
    #define  nSEL_off PORTC &= 0xFE //A0
    
    #define  SCK_on PORTB |= _BV(5) // D13
    #define  SCK_off PORTB &=~_BV(5) // D13
    
    #define  SDI_on PORTB |= _BV(3)  // B3
    #define  SDI_off PORTB &=~_BV(3) // B3
    
    #define  SDO_1 (PINB & 0x10) == 0x10 //B4
    #define  SDO_0 (PINB & 0x10) == 0x00 //B4
    
// SAW filtre support
//    #define SAW_FILT_ON _spi_write(0x0e, 0x04);    // GPIO2=1   
//    #define SAW_FILT_OFF  _spi_write(0x0e, 0x00);    // GPIO2=0
    #define SAW_FILT_ON  PORTC |= _BV(7);   
    #define SAW_FILT_OFF PORTC &= ~_BV(7);

    //#### Other interface pinouts ###
    #define GREEN_LED_pin A6
    #define RED_LED_pin A7
    
    #define Red_LED_ON   PORTC |= _BV(6);  
    #define Red_LED_OFF  PORTC &= ~_BV(6); 
    
    #define Green_LED_ON  PORTC = PORTC;  // фиктивно
    #define Green_LED_OFF PORTC = PORTC;    
//    #define Green_LED_ON  _spi_write(0x0e, 0x04);   // Оригинальный Tiny
//    #define Green_LED_OFF _spi_write(0x0e, 0x00);      
        
    #define Servo_Ports_LOW PORTB &= 0xF8; PORTC &= 0xC1; PORTD &= 0x0F; // pulling down the servo outputs
    
    #define RSSI_MODE 1 //0=disable  1=enable 
    #define RSSI_OUT 3 // D3
    
    #define Servo1_OUT 10 //Servo1
    #define Servo2_OUT 9 //Servo2
    #define Servo3_OUT 8 //Servo3
    #define Servo4_OUT 7 //Servo4
    #define Servo5_OUT 6 //Servo5
    #define Servo6_OUT 5 //Servo6
    #define Servo7_OUT 4 //Servo7
    #define Servo8_OUT A5 //Servo8
    #define Servo9_OUT A4 //Servo9 
    #define Servo10_OUT A3 //Servo10 // 2G only
    #define Servo11_OUT A2 //Servo11 // 2G only  
    #define Servo12_OUT A1 //Servo9  // 2G only
    
    #define Servo1_OUT_HIGH PORTB |= _BV(2) //Servo1
    #define Servo2_OUT_HIGH PORTB |= _BV(1) //Servo2
    #define Servo3_OUT_HIGH PORTB |= _BV(0) //Servo3
    #define Servo4_OUT_HIGH PORTD |= _BV(7) //Servo4
    #define Servo5_OUT_HIGH PORTD |= _BV(6) //Servo5
    #define Servo6_OUT_HIGH PORTD |= _BV(5) //Servo6
    #define Servo7_OUT_HIGH PORTD |= _BV(4) //Servo7
    #define Servo8_OUT_HIGH PORTC |= _BV(5) //Servo8
    #define Servo9_OUT_HIGH PORTC |= _BV(4) //Servo9
    #define Servo10_OUT_HIGH PORTC |= _BV(3) //Servo10
    #define Servo11_OUT_HIGH PORTC |= _BV(2) //Servo11
    #define Servo12_OUT_HIGH PORTC |= _BV(1) //Servo12
    
    #define Serial_PPM_OUT_HIGH PORTB |= _BV(0) //Serial PPM out on Servo 3
#endif


#if (RX_BOARD_TYPE==2)
      //### PINOUTS OF OpenLRS Rx V2 Board
      #define SDO_pin A0
      #define SDI_pin A1        
      #define SCLK_pin A2 
      #define IRQ_pin 2
      #define nSel_pin 4
      #define IRQ_interrupt 0
      
      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= 0x10 //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTC |= 0x04 //C2
      #define  SCK_off PORTC &= 0xFB //C2
      
      #define  SDI_on PORTC |= 0x02 //C1
      #define  SDI_off PORTC &= 0xFD //C1
      
      #define  SDO_1 (PINC & 0x01) == 0x01 //C0
      #define  SDO_0 (PINC & 0x01) == 0x00 //C0
      
// SAW filtre support
      #define SAW_FILT_ON  _spi_write(0x0e, 0x04);    // GPIO2=1   
      #define SAW_FILT_OFF  _spi_write(0x0e, 0x00);    // GPIO2=0

      //#### Other interface pinouts ###
      #define GREEN_LED_pin 13
      #define RED_LED_pin A3
    
      #define Red_LED_ON  PORTC |= _BV(3);
      #define Red_LED_OFF  PORTC &= ~_BV(3);
      
      #define Green_LED_ON  PORTB |= _BV(5);
      #define Green_LED_OFF  PORTB &= ~_BV(5);
      
      #define Servo_Ports_LOW PORTB &= 0xE0; PORTC &= 0xCF; PORTD &= 0x1F; // pulling down the servo outputs
      
      #define Servo1_OUT 5 //Servo1
      #define Servo2_OUT 6 //Servo2
      #define Servo3_OUT 7 //Servo3
      #define Servo4_OUT 8 //Servo4
      #define Servo5_OUT 9 //Servo5
      #define Servo6_OUT 10 //Servo6
      #define Servo7_OUT 11 //Servo7
      #define Servo8_OUT 12 //Servo8
      #define Servo9_OUT A4 //Servo9
      #define Servo10_OUT A5 //Servo10
      
      #define RSSI_MODE 0 // 0=disable  1=enable 
      #define RSSI_OUT 3  // PORTD.3      
      
      #define Servo1_OUT_HIGH PORTD |= _BV(5) //Servo1
      #define Servo2_OUT_HIGH PORTD |= _BV(6) //Servo2
      #define Servo3_OUT_HIGH PORTD |= _BV(7) //Servo3
      #define Servo4_OUT_HIGH PORTB |= _BV(0) //Servo4
      #define Servo5_OUT_HIGH PORTB |= _BV(1) //Servo5
      #define Servo6_OUT_HIGH PORTB |= _BV(2) //Servo6
      #define Servo7_OUT_HIGH PORTB |= _BV(3) //Servo7
      #define Servo8_OUT_HIGH PORTB |= _BV(4) //Servo8
      #define Servo9_OUT_HIGH PORTC |= _BV(4) //Servo9
      #define Servo10_OUT_HIGH PORTC |= _BV(5) // Servo10
    
     #define Serial_PPM_OUT_HIGH PORTD |= _BV(7) //Serial PPM out on Servo 3
//      #define Serial_PPM_OUT_HIGH PORTB = _BV(4) //Serial PPM out on Servo 8
#endif

void printlnPGM(char *adr, char ln=1);   // печать строки из памяти программы ln - перевод строки

