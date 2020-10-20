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
int ecRotState;
int ecRotLastState;
float ecCounter;
bool ecBtPushed = 0;
String menu="main";

//PID
double Setpoint, Output ,currentTemp;
double Kp=0, Ki=0, Kd=0;
int WindowSize = 1000;
unsigned long windowStartTime;
int sampleTime = 100;
unsigned long lastTime;
double pErr, iErr, dErr, lastErr;
float ecVal;
float sec = 1000;
float targetPoint, nextTargetTime, tempStep=0 ,elapse=0.0;


void setup() {
  Serial.begin(9600);
  lcd.begin(20, 4);
  pinMode(Hotend(0), INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(ecBT, INPUT_PULLUP);
  pinMode(ecClk, INPUT_PULLUP);
  pinMode(ecData, INPUT_PULLUP);

  //setPid(20,0.05,5);//45
  setPid(50,1.125,160);//45
  Setpoint = 25;
  ecVal = 45;
  
  }

void loop(){
  menuSelect(menu);
  
 // isBtPush();//Reset some values
  double Input = getTemp(Hotend(0));//A0
  currentTemp = Input;

  targetPoint = nextTemperature(currentTemp);

  Compute(targetPoint, Input);

  pidCtrl();
  
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
  
    if ( m == "main"){ecVal += ecCounter;}
    else if ( m == "Kp" ){Kp += ecCounter;}
    else if ( m == "Ki" ){Ki += ecCounter * 0.01;}
    else if( m == "Kd" ){Kd += ecCounter;}
    else if( m == "start" ){ecVal += ecCounter;}
    else ;
    
    ecRotLastState = ecRotState;
    //ecVal += ecCounter;
    if (ecCounter ==1 || ecCounter ==-1 ){ecCounter=0;}
  }
if (digitalRead(ecBT)==0 && m=="main"){
    menu = "pid";
    delay(500);//ecBT debounce
    return;
 }
 if (digitalRead(ecBT)==0 && m=="pid"){
    menu = "Kp";
    delay(500);//ecBT debounce
    return;
 }
  if (digitalRead(ecBT)==0 && m=="Kp"){
    menu = "Ki";
    delay(500);//ecBT debounce
    return;
 }
   if (digitalRead(ecBT)==0 && m=="Ki"){
    menu = "Kd";
    delay(500);//ecBT debounce
    return;
 }
 
 if (digitalRead(ecBT)==0 && m=="Kd"){//ecBT pushed
    menu = "start";
    delay(500);//ecBT debounce
    return;
 }
 if (digitalRead(ecBT)==0 && m=="start"){
    menu = "main";
    delay(500);//ecBT debounce    elapse = 0;
    WindowSize = 1000; sampleTime = 100;
    tempStep = 3;
    pErr=0; iErr=0; dErr=0;
    //getTemp(Hotend(A0))
    targetPoint = getTemp(Hotend(0)) + tempStep;// + 1 nextTemperature 
    Setpoint = ecVal;
    return;
 }
 
}


float nextTemperature(float currentTemp){
  if ( currentTemp + 10 >= Setpoint ){
   WindowSize=2000; sampleTime=1000, tempStep=1;
  }
  else {WindowSize = 1000; sampleTime = 100;}
  
  if (currentTemp >= targetPoint){
  //if (millis()-nextTargetTime >= sec && currentTemp >= targetPoint){
    targetPoint += tempStep;
    if (targetPoint > Setpoint) targetPoint = Setpoint;
    nextTargetTime = millis();
  }
  return targetPoint;
}


void Compute(float targetPoint,float Input){
  unsigned long now = millis();
  double timeChange = now - lastTime;
  if ( timeChange >= sampleTime){
    pErr = (targetPoint - Input);
    iErr += pErr * timeChange/1000;//sec
    dErr =  (pErr - lastErr)/timeChange/1000;
    Output = Kp * pErr + Ki * iErr + Kd * dErr;
    lastErr = pErr;
    lastTime = now;
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
      //lcd.begin(20, 4);
      if(menu=="main"){
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
        lcd.print("iEr:");
        lcd.print(iErr);
        lcd.setCursor(0, 3);
        lcd.print("Tg:");
        lcd.print(targetPoint,0);
        lcd.print(" elapse:");
        lcd.print(elapse,0);
      
        //Serial.print("\t");
        //Serial.print(" Output: ");
        //Serial.println(Output);  
      }
      if(menu== "pid"){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(" Kp:");
        lcd.print(Kp);
        lcd.setCursor(0, 1);
        lcd.print(" Ki:");
        lcd.print(Ki);
        lcd.setCursor(0, 2);
        lcd.print(" Kd:");
        lcd.print(Kd); 
        lcd.setCursor(0, 3);
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
      if(menu== "start"){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(currentTemp);
        lcd.print("C");
        lcd.print(" Set:");
        lcd.print(int(Setpoint));
        lcd.print(" *:");
        lcd.print(int(ecVal));
        lcd.setCursor(0, 1);     
        lcd.print("Output");
        lcd.print(Output,0);
        lcd.print(" MI:");
        lcd.print(menu);
        lcd.print("        Back");                          
      }

      Serial.print("Tc: ");
      Serial.println(currentTemp);
      
      if (!RefreshLCD)elapse++;
        RefreshLCD = 0;
        lcdDisplayCounter = now;        

    }
    
  }
