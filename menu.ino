// Функции меню терминала
//
static unsigned char regs[] = {1, 2, 3, 11,12,13,14,15,16,17,18,19,20,24,25,26,28,40,41,42 } ;
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
  "Beacon Pmax (mWt): 0-1.2; 1-2; 2-3; 3-6; 4-12; 5-25; 6-50; 7-100",  
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
      wdt_reset();               //  поддержка сторожевого таймера
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
