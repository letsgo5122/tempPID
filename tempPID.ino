#include<LiquidCrystal.h>
const int rs = 12, en = 11, d4 = 7, d5 = 6, d6 = 5, d7 = 4;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define seconds(x) (x * 1000) //x seconds
#define RELAY 13


//Temperature sensor
#define Hotend(x) uint8_t(x)  //x = Anolog 0 ~ 5
int Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

//LCD 
#define ecClk 2
#define ecData 3
#define ecBT A1
double lcdDisplayCounter = 0;// ms
bool RefreshLCD = 0;
int m_w;
int ecRotState=1;//encoder A,B pins stop at HIGH
int ecRotLastState;
float ecCounter;
bool ecBtPushed = 0;


//menu
String menu="Main";
String menuPID[]= { "Start", "Kp","Ki", "Kd" ,"WindowSize","SampleTime","BackMain","Back"};
float itemIndex;
String raw[4];

//PID
double Setpoint, Output ,currentTemp;
double Kp=0, Ki=0, Kd=0;
int WindowSize = 1000;
unsigned long windowStartTime;
int SampleTime = 100;
unsigned long lastTime;
double pErr, iErr, dErr, lastErr, temp_dState;
float ecVal;
//float sec = 1000;
float targetPoint, nextTargetTime, period, tempStep=0 ,elapse=0.0;
bool pidRun=0;

void setup() {
  Serial.begin(9600);
  lcd.begin(20, 4);
  pinMode(Hotend(0), INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(ecBT, INPUT_PULLUP);
  pinMode(ecClk, INPUT_PULLUP);
  pinMode(ecData, INPUT_PULLUP);

  //setPid(50,1.125,160);//45
  setPid(20,1.06,50);//45
  Setpoint = 25;
  ecVal = 50;
  menu = "Main";

  }

void loop(){
  menuSelect(menu);
  
 // isBtPush();//Reset some values
  double Input = getTemp(Hotend(0));//A0
  currentTemp = Input;
  
  if (pidRun){
      targetPoint = nextTemperature(currentTemp);
    
      Compute(targetPoint, Input);
    
      pidCtrl();
    }
  
  lcdDisplay();
  
  }

void menuSelect(String m){
  ecRotState = digitalRead(ecClk);
  if ( ecRotState!=ecRotLastState ){
    if( digitalRead(ecData)!= ecRotState  ){
      ecCounter = 0.5;// Run two times when rotary turned a tick      
    }
    else { 
      ecCounter = -0.5;
    }

    if ( m == "Main"){ecVal += ecCounter;}
    else if ( m == "menuPID"){itemIndex += ecCounter;}
    else if( m == "Start" ){ecVal += ecCounter;}
    else if ( m == "Kp" ){Kp += ecCounter;}
    else if ( m == "Ki" ){Ki += ecCounter * 0.01;}
    else if( m == "Kd" ){Kd += ecCounter;}
    else if ( m == "WindowSize" ){WindowSize += ecCounter * 100;}
    else if ( m == "SampleTime" ){SampleTime += ecCounter * 10;}
    ecRotLastState = ecRotState;
    if (ecCounter ==1 || ecCounter ==-1 ){ecCounter=0;}
  }
    
 

   if (digitalRead(ecBT)==0 && m=="Main"){
      menu = "menuPID";
      delay(500);//ecBT debounce
   }
   //submenu
  if ( m == "menuPID" ){
    uint8_t elements = sizeof(menuPID)/sizeof(menuPID[0]);
    itemIndex =  itemIndex > elements? 0:abs(itemIndex);
    String allItem[elements];
    uint8_t selectedItem;
    //add '>' or space at each item
    for (int i= 0;i<elements;i++){
      if ((int)itemIndex==i){
        allItem[i] = '>' + menuPID[i];
        selectedItem = i;       
      }
      else { allItem[i] = ' ' + menuPID[i]; }
      }
     //scroll item 
     if ( 0<=selectedItem && selectedItem<=3){
      for(int i = 0;i<4 ;i++){
        raw[i] = allItem[i];
      }
     }
     if ( 4<=selectedItem && selectedItem<=7){
      for(int i = 0;i<4 ;i++){
        raw[i] = allItem[i+4];
      }
     }
     
  }
   
   if (digitalRead(ecBT)==0 && m=="menuPID"){
    uint8_t elements = sizeof(menuPID)/sizeof(menuPID[0]);
    for (int i= 0;i<elements;i++){
        //Serial.println(raw[i].charAt(0));
        if ( raw[i][0] == '>'  ){ 
          menu = raw[i].substring(1); 
          Serial.println(menu);
        }   
    }
    delay(500);//ecBT debounce
   }
   if (digitalRead(ecBT)==0 && m=="BackMain"){
      menu = "Main";  
      delay(500);//ecBT debounce
   }
   if (digitalRead(ecBT)==0 && m=="Back"){
      menu = "menuPID";  
      delay(500);//ecBT debounce
   }
   if (digitalRead(ecBT)==0 && m=="Kp"){
      menu = "menuPID";  
      delay(500);//ecBT debounce
   }

   if (digitalRead(ecBT)==0 && m=="Ki"){
      menu = "menuPID";
      delay(500);//ecBT debounce
   }
   if (digitalRead(ecBT)==0 && m=="Kd"){
      menu = "menuPID";
      delay(500);//ecBT debounce
   }
  if (digitalRead(ecBT)==0 && m=="WindowSize"){
      menu = "menuPID";  
      delay(500);//ecBT debounce
   }
   if (digitalRead(ecBT)==0 && m=="SampleTime"){
      menu = "menuPID";  
      delay(500);//ecBT debounce
   }

   if (digitalRead(ecBT)==0 && m=="Start"){
      menu = "Main";
      delay(500);//ecBT debounce    elapse = 0;
      WindowSize = 500; SampleTime = 50;
      tempStep = 3;//3
      pErr=0; iErr=0; dErr=0;
      //getTemp(Hotend(A0))
      targetPoint = getTemp(Hotend(0)) + tempStep;// nextTemperature 
      Setpoint = ecVal;
      pidRun = 1;
   }
 
}


float nextTemperature(float currentTemp){ 
  if (currentTemp >= targetPoint && currentTemp < Setpoint ){
      
      targetPoint += tempStep;

      if ( currentTemp + 10 >= Setpoint && WindowSize <1600){
       int w_size =abs(Setpoint-currentTemp)*50;
        WindowSize+=w_size; 
        SampleTime+=w_size; 
        tempStep=1;
        if (WindowSize>1600)WindowSize = 1600;
        if (SampleTime>1000)SampleTime = 1000;
      }
    }
  if (targetPoint > Setpoint) targetPoint = Setpoint;
  return targetPoint;
}


void Compute(float targetPoint,float Input){
  unsigned long now = millis();
  double timeChange = now - lastTime;  
  if ( timeChange >= SampleTime ){ 
    pErr = (targetPoint - Input); 
    //iErr = constrain(iErr + pErr, -250, 250);
    iErr += pErr * timeChange/1000;//sec
    dErr =  (pErr - lastErr)/timeChange/1000;
    //dErr = Kd + (Kd*(temp_dState -Input)-Kd);
    Output = Kp * pErr + Ki * iErr + Kd * dErr;
    if (Output>WindowSize)Output = WindowSize;
    lastErr = pErr;
    lastTime = now;
    temp_dState = Input;
  }  
}

void pidCtrl(){
   if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  m_w = millis() - windowStartTime;
  if (Output > m_w) {
    digitalWrite(RELAY, HIGH);
    //RefreshLCD = 1;
  }
  else digitalWrite(RELAY, LOW);

}


void setPid(double p, double i, double d){
  Kp = p; Ki = i; Kd = d;
}

double getTemp(uint8_t pin){
  Vo = analogRead(pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  //Tf = (Tc * 9.0)/ 5.0 + 32.0;
  return Tc;
  }


void lcdDisplay(){
  double now = millis();
    if (now-lcdDisplayCounter >= seconds(1) || RefreshLCD){
      if(menu=="Main" || menu=="BackMain"){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(currentTemp);
        lcd.print("C");
        lcd.print(" Set:");
        lcd.print(int(Setpoint));
        lcd.print(" *:");
        lcd.print(int(ecVal));
        lcd.setCursor(0, 1);     
        lcd.print("Out:");
        lcd.print(Output,0);
        lcd.print(" Menu:");
        lcd.print(menu);
        lcd.setCursor(0, 2);
        lcd.print("WSize:");
        lcd.print(WindowSize);
        lcd.setCursor(0, 3);
        lcd.print("Tg:");
        lcd.print((int)targetPoint);
        lcd.print(" elapse:");
        lcd.print(elapse,0);
      
        //Serial.print("\t");
        //Serial.print(" Output: ");
        //Serial.println(Output);  
      }
      if(menu== "menuPID"){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(raw[0]);
        lcd.setCursor(0, 1);
        lcd.print(raw[1]);
        //lcd.print(":");
        //lcd.print(Kp);
        lcd.setCursor(0,2);
        lcd.print(raw[2]);
        //lcd.print(":");
        //lcd.print(Ki);
        lcd.setCursor(0,3);
        lcd.print(raw[3]);
        //lcd.print(":");
        //lcd.print(Kd);
      }
      if(menu== "Back"){
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("BK");
        lcd.print("        Back");  
      }
      if(menu== "Kp"){
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Kp:");
        lcd.print(Kp);
        lcd.setCursor(0, 3);
        lcd.print("        Back");  
      }
      if(menu== "Ki"){
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Ki:");
        lcd.print(Ki);
        lcd.setCursor(0, 3);
        lcd.print("        Back");  
      }
      if(menu== "Kd"){
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Kd:");
        lcd.print(Kd);
        lcd.setCursor(0, 3);
        lcd.print("        Back");  
      }
      if(menu== "WindowSize"){
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("WindowSize:");
        lcd.print(WindowSize);
        lcd.setCursor(0, 3);
        lcd.print("        Back");  
      }
      if(menu== "SampleTime"){
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("SampleTime:");
        lcd.print(SampleTime);
        lcd.setCursor(0, 3);
        lcd.print("        Back");  
      }
      if(menu== "Start"){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(currentTemp);
        lcd.print("C");
        lcd.print(" Set:");
        lcd.print(int(Setpoint));
        lcd.print(" *:");
        lcd.print(int(ecVal));
        lcd.setCursor(0, 1);
        lcd.print("Kp:");
        lcd.print(Kp,0); 
        lcd.print(" Kd:");
        lcd.print(Kd,0);   
        lcd.setCursor(0, 2);     
        lcd.print("Ki:");
        lcd.print(Ki,3);                   
      }

      Serial.print("Tc: ");
      Serial.println(currentTemp);
      
      if (!RefreshLCD)elapse++;
        RefreshLCD = 0;
        lcdDisplayCounter = now;        

    }
    
  }
