/*
 * Tugas Sistem Mikroprosesor - Jam Digital
 * Naufalino Fadel Hutomo (13214138)
 * Adi Trisna Nurwijaya (13214122)
*/

//library
#include<avr/io.h>
#include<avr/interrupt.h>
#include <util/delay.h>

#define seg_dot 7 // PD7

#define pinA 0  //seven segment detik //PB0
#define pinB 1  //seven segment detik puluhan PB1
#define pinC 2  //seven segment menit  PB2
#define pinD 3  ////seven segment menit puluhan  //PB3
#define ledPin 4 // state indicator PB5
	
#define modeBtn 0 //PC0
#define incBtn 1 //PC1

//Global Variable
int s=0; //second
int m=27; //minute
int h=11; //hour
int setM,setS,setH; //for setting clock
const int holdTime =2000;
int swM,swS, timStart; //for stopwatch
int swMode=LOW, swModeLast=LOW;

int state = 0; //state 0 for MM:SS, state=1 for HH:MM
unsigned int button0 = 1, button0Last = 1, button1 = 1;
long btnDnTime; //time the button hold down
long btnUpTime; //time the button hold high
int ignoreUp = 0; //whether ignore the button release or not
	//millis putpose
unsigned long millis;	// value of milliseconds elapsed since the device turn on, reset when change mode
unsigned long millis_frac; //fraction off millisecond. Because timer interrupt every 1,024 ms, the variable contain the fraction value

// function declaration 


//timer interrupt
ISR(TIMER1_COMPA_vect)
{
	s=s++;
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
	
	if(state==6 && (swMode!=0))
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

//Timer for counting millis
ISR(TIMER0_OVF_vect)
{
	millis++;
	millis_frac +=24;	//interrupt period %1000 us=1024%1000
	if (millis_frac >=1000)
	{
		millis_frac -=1000;
		millis++;
	}
	
}

int main()
{
	port_config(); //port configuration
	init_int();  //timer 0 & 1 interrupt initialization
		
	//main loop
	while(1)
	{
		//read button
		button0 = PINC & (1<<modeBtn);
		if(button0 == 0 && button0Last != 0 && (millis -btnDnTime)>500 )
		{
			btnDnTime = millis;
		}
		
		//button released , store up the time
		  if(button0 != 0 && button0Last==0 && (millis-btnDnTime) > 50 ) //50 ms is debounce time
		  {
			if(ignoreUp == 0)
			  {
				if(state>=5)
				{
					state= (state%2 )+5;
				}
				else if (state >= 2)   //when in adjustment mode, toggle the adjustment mode
				{
				  state = (state-1)%3 +2;
				}else     
				{
				  state = (state+1)%2;    //toggle display mode
				}
			  }else
			  {
				ignoreUp=0;
				btnUpTime=millis;  
			  } 
		  }

		  //button held , store up the time
		  if(button0 == 0  && (millis-btnDnTime) > 2000 ) //2000 ms is held time
		  {
			if(state>=5)
			{
				state=0;
			}
			else if (state > 2)
			{
			  s=setS; m = setM; h= setH;    //time adjusted
			  state = state%2;
			  PORTB &= ~(1 << ledPin);   // indicate that back to display mode, led LOW
			} 
			else if(state=2)
			{
				state=5;
			}else
			{
			  setS=s; setM=m; setH=h;     //enter adjustment mode
			  state= 2;
			  PORTB |= (1 << ledPin);   // indicate that in adjustment mode      
			}
			ignoreUp = 1;
			btnDnTime = millis;
		  }
		  
		  button0Last=button0;
	
		// state machine
		  switch (state)
		  {
			case 1: showClock(m,h); break;
			case 2: 
			  state2();   //hour adjustment
			  break;
			case 3:
			  state3();   //minute adjustment
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
	return 0;
}

void port_config (void)
{
	DDRD = 0xFF;
	PORTD = 0xFF;  //PD0-PD7 for 7segment
	DDRB = 0x1F;	//PB0-PB5 for 4 digit anode and LED pin
	PORTB = 0x00;
	DDRC = 0x00;	//PC0 for modeBtn, PC1 for incBtn. Input
	PORTC= 0x03;	// internal pull up
}

void init_int (void)
{
	//Timer 1 for counting time
	OCR1A = 0x3D08; //1 s /(1 / 16MHz*1024) -1

    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC with top on OCR1A, register TCCR1A default, no nedd to change

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS12) | (1 << CS10);
    // set prescaler to 1024 and start the timer
	
	//Timer 0 for milllis
	TCCR0B |= (1<<CS01) | (1<<CS00); //prescaler 64. Interrupt when overflow, at 1/16MHz*64*2^8=1024us
	TIMSK0 |= (1<<TOIE0); //overflow interrupt enable
	
    sei();
    // enable interrupts, global interrupt
	
}

 //showing number in 7 segment
void showClock(int number0,int number1)
{
  int a,b,c,d;
  a= number0%10;
  b= number0/10;
  c= number1%10;
  d=number1/10; 

  //display each digit
  displayNumber(a);
  PORTB |= (1<<pinA); // digitalWrite(pinA,HIGH); 
  if (state >= 3)
  {
    PORTD &= ~(1<<seg_dot); //digitalWrite(seg_dot,LOW);
  }
  _delay_ms(4);
  PORTB &= ~(1<<pinA); //digitalWrite(pinA,LOW); 
  displayNumber(b);
  PORTB |= (1<<pinB); //digitalWrite(pinB,HIGH); 
  _delay_ms(4);
  PORTB &= ~(1<<pinB); //digitalWrite(pinB,LOW); 
  displayNumber(c);
  PORTB |= (1<<pinC); //digitalWrite(pinC,HIGH); 
  if (state < 3)
  {
    PORTD &= ~(1<<seg_dot); //digitalWrite(seg_dot,LOW);  
  }
  _delay_ms(4);
  PORTB &= ~(1<<pinC); //digitalWrite(pinC,LOW);  
  displayNumber(d);
  PORTB |= (1<<pinD); //digitalWrite(pinD,HIGH);  
  _delay_ms(4);
  PORTB &= ~(1<<pinD); //digitalWrite(pinD,LOW);  
 }
 
 //displaying digit to 7 segment
 void displayNumber (int number){
  switch (number){
    case 0:
      PORTD = 0xC0;
	  /*
	  digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,LOW);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,HIGH);
      digitalWrite(seg_dot,HIGH);
      */
	  break;
    case 1:
      PORTD = 0xF9;
	  /*digitalWrite(seg_a,HIGH);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,HIGH);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,HIGH);
      digitalWrite(seg_dot,HIGH);
      */
	  break;
    case 2:
	  PORTD = 0xA4;
	  /*
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,HIGH);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,LOW);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      */
	  break;
    case 3:
	PORTD = 0xB0;
	/*
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      */
	  break;
    case 4:
    PORTD = 0x99;
	/* 
	  digitalWrite(seg_a,HIGH);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,HIGH);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
     */ 
	  break;
    case 5:
	PORTD = 0x92;
	/*
	  digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,HIGH);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      */
	  break;
    case 6:
	PORTD = 0x82;
	/*
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,HIGH);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,LOW);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
	*/
	 break;
    case 7:
     PORTD = 0xF8;
	 /* 
	  digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,HIGH);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,HIGH);
      digitalWrite(seg_dot,HIGH);
    */ 
	 break;
    case 8:
	PORTD = 0x80;
	/*
      digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,LOW);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      */
	  break;
    case 9:
      PORTD = 0x90;
	  /*
	  digitalWrite(seg_a,LOW);
      digitalWrite(seg_b,LOW);
      digitalWrite(seg_c,LOW);
      digitalWrite(seg_d,LOW);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,LOW);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
      */
	  break;  
    default:
      PORTD = 0xBF;
	  /*
	  digitalWrite(seg_a,HIGH);
      digitalWrite(seg_b,HIGH);
      digitalWrite(seg_c,HIGH);
      digitalWrite(seg_d,HIGH);
      digitalWrite(seg_e,HIGH);
      digitalWrite(seg_f,HIGH);
      digitalWrite(seg_g,LOW);
      digitalWrite(seg_dot,HIGH);
	*/
   }
 }
 
 //Hour adjustment
void state2()
{
  button1 = PINC & (1<<incBtn);
  
  if (button1 == 0)
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
  button1 = PINC & (1<<incBtn);
  
  if (button1 == 0)
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
  button1 = PINC & (1<<incBtn);
  
  if (button1 == 0)
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
  swS=0; swM=0; swMode=0;
  showClock(swS, swM); 
}

//stopwatch counting mode
void state6()
{
  button1 = PINC & (1<<incBtn);
  if (button1 == 0)
  {
	swMode= !swModeLast;
  }
  showClock(swS,swM);
  swModeLast=swMode;
}
