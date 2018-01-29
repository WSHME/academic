/* Cited from instructables.com
Edited by : & Rizqi Cahyo Yuwono (13214090) & Naufalino Fadel Hutomo (13214138)
Instrumentation System - Final Project 
*/
#include <LiquidCrystal.h>

LiquidCrystal lcd(1,0,14,15,16,17); //LCD pins

volatile byte REV;       

unsigned long int rpm, maxRPM=20000000;  //  DEFINE RPM AND MAXIMUM RPM
 
unsigned long time;      
 
int ledPin = 13;           //   STATUS LED
 
int led = 0,RPMlen , prevRPM;  
 
int flag = 0;             

long prevtime = 0;       
 void setup()
 {
     lcd.begin(16, 2);     // INITIATE LCD 
     attachInterrupt(digitalPinToInterrupt(2), RPMCount, RISING);     
     
     REV = 0;      //  START ALL THE VARIABLES FROM 0 
     rpm = 0; 
     time = 0;
     pinMode(ledPin, OUTPUT);   
     
     lcd.print("TACHOMETER");           //   STARTUP TEXT
     lcd.setCursor(0, 1);
     lcd.print("090 - 138 ");          //  THAT'S ME
     delay(2000);
     lcd.clear();
     
 }
 
 void loop()
 {
  long currtime = micros();                 // GET CURRENT TIME
  
  long idletime = currtime - prevtime;        //  CALCULATE IDLE TIME
    
    if(REV >= 5 )           //  IT WILL UPDATE AFETR EVERY 5 READINGS
   {
     
             
     if(flag==0)            
     {
       lcd.clear();
       lcd.print("MOHON BERSABAR");
       flag=1;              
     }
     
     rpm = 60*1000000/(micros() - time)*(REV-1);       
     
     time = micros();                            
     
     REV = 0;
     
     int x= rpm;
     while(x!=0)
     {
       x = x/10;
       RPMlen++;
     }         
     
     if(RPMlen!=prevRPM)                             
     {
       lcd.clear();
       prevRPM = RPMlen;
       flag=0;
       lcd.print("SENSOR MEASURING");
     }
     
     lcd.setCursor(0, 1);
     lcd.print(rpm,DEC);
     
     lcd.setCursor(6,1);
     lcd.print("RPM");
     delay(500);

     prevtime = currtime; 
   }
   
   if(idletime > 5000000 )                      
   {
     
     if(flag==1)                            
     {
       lcd.clear();
       flag=0;
     }
     lcd.clear();
     lcd.print("MAXIMUM RPM");
     lcd.setCursor(0, 1);
     lcd.print(maxRPM,DEC);                     
     lcd.print("   RPM");
     delay(2000);
     lcd.clear();
     lcd.print("IDLE STATE");
     lcd.setCursor(0, 1);
     lcd.print("READY TO MEASURE");
     delay(2000);
     
     rpm=0;
     prevtime = currtime;
   }
     
     
 }
 
 void RPMCount()                                
 {
   REV++;                                       
}
