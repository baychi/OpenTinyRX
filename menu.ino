// Функции меню терминала
//
static unsigned char regs[] = {1, 2, 3, 4, 11,12,13,14,15,16,17,18,19,20,24,25,26,28,40,41,42 } ;
static char *help[] = {
  "Bind N",
  "Freq Corr",
  "Servo 150% strech num (1-12)", 
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
  "Beacon Pmax (mWt): 0-1.2; 1-2; 2-3; 3-6; 4-12; 5-25; 6-50; 7-100",  
  "Beacon start time (sec)",
  "SAW Fmin",
  "SAW Fmax",
  "PPM mode 1st PWM chnl (1-8) [4]", 
  "RSSI type: sound(0)/level(1-99: average factor)",
  "RSSI mode: level(0)/SN ratio(1)",
  "RSSI over PWM (ch num:1-12) 0- not use"
};  
  

void showRegs(void)         // показать значения регистров
{
  unsigned char i,j=0;
  for(i=1; i<=REGS_NUM; i++) {
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


byte checkMenu(void)   // проверка на вход в меню
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
       in=Serial.read();             // все, что пришло, отображаем
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
          if(sn < 10) sn++;             // не более 10 символов
        }
     } else delay(1);
      wdt_reset();               //  поддержка сторожевого таймера
   }
}

#define R_AVR 199               // усреднение RSSI

byte margin(byte v)
{
   if(v < 10) return 0; 
   else if(v>71) return 61;

   return  v-10;
}

void showNoise(char str[])             // отображаем уровень шумов по каналам
{
  byte fBeg=0, fMax=254;
  byte rMin, rMax;
  word rAvr;
  byte i,j,k;

  rAvr=atoi(str+1);           // считаем параметры, если есть в виде Nbeg-end
  if(rAvr > 0 && rAvr < 255) {
     fBeg=rAvr;
     for(i=2; i<10; i++) {
      if(str[i] == 0) break;
      if(str[i] == '-') {
        rAvr=atoi(str+i+1);
        if(rAvr > fBeg && rAvr < 255) fMax=rAvr;
        break;
      }
    }
  }
  
  RF22B_init_parameter();      // подготовим RFMку 
  to_rx_mode(); 
  SAW_FILT_OFF                 
 
  Serial.println("FHn: Min Avr Max");
  
  for(i=fBeg; i<=fMax; i++) {    // цикл по каналам
     _spi_write(0x79, i);       // ставим канал
     delayMicroseconds(749);
     rMin=255; rMax=0; rAvr=0;
     for(j=0; j<R_AVR; j++) {   // по каждому каналу 
       delayMicroseconds(99);
       k=_spi_read(0x26);         // Read the RSSI value
       rAvr+=k;
       if(k<rMin) rMin=k;         // min/max calc
       if(k>rMax) rMax=k;
     }
     if(i < 10) Serial.print("  ");
     else if(i <100) Serial.print(" ");
     Serial.print(i);
     k=':';
     for(j=0; j<HOPE_NUM; j++) {   // отметим свои частоты
        if(hop_list[j] == i) {
          k='#';
        }
     }
     Serial.write(k); Serial.print(" ");
     print3(rMin);   
     k=rAvr/R_AVR;  print3(k);
     print3(rMax);

     if(str[0] == 'N') {         // если надо, печатаем псевдографику 
       rMin=margin(rMin); 
       rMax=margin(rMax); 
       k=margin(k); 

       for(j=0; j<=rMax; j++) {                         // нарисуем псевдографик
         if(j == k) Serial.print("*");
         else if(j == rMin) Serial.print("<");
         else if(j == rMax) Serial.print(">");
         else if(j>rMin && j <rMax) Serial.print(".");
         else Serial.print(" ");
       }
     }
     
     Serial.println();
     wdt_reset();               //  поддержка сторожевого таймера
  }
}

void doMenu()                       // работаем с меню
{
  char str[12];
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
    if(str[0] == 'n' || str[0] == 'N') {  // отсканировать и отобразить уровень шума 
       showNoise(str);
       goto rep;
    }
    
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
