/*
 * Tugas Sistem Mikroprosesor
 * Naufalino Fadel Hutomo (13214138)
 * Adi Trisna Nurwijaya (13214122)
 */

#include <TimerOne.h>

//Pin
#define modeBtn A0 //PC0
#define incBtn A1 //PC1
#define seg_a 0 //PD0
#define seg_b 1
#define seg_c 2
#define seg_d 3 //PD3
#define seg_e 4 //PD4
#define seg_f 5 //PD5
#define seg_g 6 //PD6
#define seg_dot 7 //PD7
#define pinA 8  //seven segment detik //PB0
#define pinB 9  //seven segment detik puluhan
#define pinC 10  //seven segment menit
#define pinD 11  ////seven segment menit puluhan  //PB3
#define ledPin 13 // state indicator

//Global Variable
int s=0; //second
int m=27; //minute
int h=11; //hour
const int holdTime =2000;

int state = 0; //state 0 for MM:SS, state=1 for HH:MM
bool button0 = HIGH, button0Last = HIGH, button1 = HIGH;
long btnDnTime; //time the button hold down
long btnUpTime; //time the button hold high
boolean ignoreUp =false; //whether ignore the button release or not
unsigned long firstTime; //how long since the first button was pressed
int setM,setS,setH; //for setting clock
int swM,swS, timStart; //for stopwatch
boolean swMode=LOW, swModeLast=LOW;
void setup() {
  // put your setup code here, to run once:
  pinMode(seg_a,OUTPUT);
  pinMode(seg_b,OUTPUT);
  pinMode(seg_c,OUTPUT);
  pinMode(seg_d,OUTPUT);
  pinMode(seg_e,OUTPUT);
  pinMode(seg_f,OUTPUT);
  pinMode(seg_g,OUTPUT);
  pinMode(seg_dot,OUTPUT);
  
  pinMode(pinA,OUTPUT);   //last digit
  digitalWrite(pinA,HIGH);
  pinMode(pinB,OUTPUT);
  digitalWrite(pinB,LOW);
  pinMode(pinC,OUTPUT);
  digitalWrite(pinC,LOW);
  pinMode(pinD,OUTPUT); //first digit
  digitalWrite(pinD,LOW);
  pinMode (ledPin, OUTPUT);
  digitalWrite(ledPin,LOW);
    
  pinMode(modeBtn,INPUT);
  pinMode(incBtn,INPUT);

  //inisialisasi timer
  Timer1.initialize(1000000);
  Timer1.attachInterrupt(timerIsr);
}

void showClock(int number1, int number2);
void state2(void);
void state3 (void);
void state4(void);
void state5 (void);
void state6(void);

void loop() {
  
  button0 = digitalRead(modeBtn);
  
  if(button0==LOW && button0Last==HIGH &&(millis()-btnUpTime)>500)
  {
    btnDnTime=millis();
    }

  //button released , store up the time
  if(button0 == HIGH && button0Last==LOW && (millis()-btnDnTime) > 50 ) //50 ms is debounce time
  {
    if(ignoreUp== false)
      {
        if (state>=5)
        {
          state = (state)%2+5;
        }else if (state >= 2)   //when in adjustment mode, toggle the adjustment mode
        {
          state = (state-1)%3 +2;
        }else     
        {
          state = (state+1)%2;    //toggle display mode
        }
      }else
      {
        ignoreUp=false;
        btnUpTime=millis();  
      } 
  }

  //button held , store up the time
  if(button0 == LOW  && (millis()-btnDnTime) > 2000 ) //2000 ms is held time
  {
    if(state>=5)
    {
      state=0;
    }else if (state > 2)
    {
      s=setS; m = setM; h= setH;    //time adjusted
      state = state%2;
      digitalWrite(ledPin, LOW);   // indicate that back to display mode
    }else if (state==2)
    {
      state=5;
    }else
    {
      setS=s; setM=m; setH=h;     //enter adjustment mode
      state= state+2;
      digitalWrite(ledPin, HIGH);   // indicate that in adjustment mode      
    }
    ignoreUp = true;
    btnDnTime = millis();
  }
  
  button0Last=button0;  
  
  // state machine
  switch (state)
  {
    case 1: 
      showClock(m,h); 
      break;
    case 2: 
      state2();   //minute adjustment
      break;
    case 3:
      state3();   //second adjustment
      break;
    case 4:
      state4();   //second adjustment
      break;  
    case 5:
      state5();
      break;
    case 6:
      state6();
      break;
    case 0: showClock(s,m);
      break; 
  }
}

void timerIsr()
{
    //counting
    s++;
    
    if (s==60)
    {
      s=0;
      m++;
      if(m==60)
      {
        m=0;
        h++;
       }
      if(h==24)
      {
        h=0;
        } 
    }

    if(state==6 && (swMode==HIGH))
    {
      swS++;
      if (swS==60)
      {
        swS=0;
        swM++;
        if(swM==60)
        {
          swM=0;
        }
      }
    }
}

//Hour adjustment
void state2()
{
  button1 = digitalRead (incBtn);
  if (button1 == LOW)
  {
    setH++;
  }
  if (setH==24)
  {
    setH = 0;       //overflow
   }
   showClock(setM,setH);
}

//minute adjustment
void state3()
{
   button1 = digitalRead (incBtn);
  if (button1 == LOW)
  {
    setM++;
  }
  if (setM==60)
  {
    setM = 0;       //overflow
   }
  showClock(setM,setH); 
}

//second adjustment
void state4()
{
   button1 = digitalRead (incBtn);
  if (button1 == LOW)
  {
    setS++;
  }
  if (setS==60)
  {
    setS = 0;       //overflow
   }
  showClock(setS,setM); 
}

//stopwatch reset mode
void state5()
{
  swS=0; swM=0; swMode=LOW;
  showClock(swS, swM); 
  digitalWrite(ledPin,HIGH);
}

//stopwatch counting mode
void state6()
{
  button1=digitalRead(incBtn);
  swMode= !(swModeLast ^ button1);
  showClock(swS,swM);
  digitalWrite(ledPin,HIGH);
  swModeLast=swMode;
}

 
void displayNumber (int number){
 switch (number){
    case 0:
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,LOW);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,HIGH);
      digitalWrite(seg_dot,HIGH);
      break;
    case 1:
      digitalWrite(seg_a,HIGH);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,HIGH);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,HIGH);
      digitalWrite(seg_dot,HIGH);
      break;
    case 2:
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,HIGH);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,LOW);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      break;
    case 3:
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      break;
    case 4:
      digitalWrite(seg_a,HIGH);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,HIGH);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      break;
    case 5:
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,HIGH);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      break;
    case 6:
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,HIGH);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,LOW);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      break;
    case 7:
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,HIGH);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,HIGH);
      digitalWrite(seg_dot,HIGH);
      break;
    case 8:
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,LOW);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      break;
    case 9:
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      break;  
    default:
      digitalWrite(seg_a,HIGH);
      digitalWrite(seg_b,HIGH);
      digitalWrite(seg_c,HIGH);
      digitalWrite(seg_d,HIGH);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
   }
 }

 //showing number in 7 segment
void showClock(int number0,int number1)
{
  unsigned int a,b,c,d;
  a= number0%10;
  b= number0/10;
  c= number1%10;
  d=number1/10; 

  //display each digit
  displayNumber(a);
  digitalWrite(pinA,HIGH);
  if (state >= 3)
  {
    digitalWrite(seg_dot,LOW);
  }
  delay(3);
  digitalWrite(pinA,LOW);
  displayNumber(b);
  digitalWrite(pinB,HIGH);
  delay(3);
  digitalWrite(pinB,LOW);
  displayNumber(c);
  digitalWrite(pinC,HIGH);
  if (state < 3 || state ==6)
  {
    digitalWrite(seg_dot,LOW);
  }
  delay(3);
  digitalWrite(pinC,LOW);
  displayNumber(d);
  digitalWrite(pinD,HIGH);
  delay(3);
  digitalWrite(pinD,LOW);
 }

