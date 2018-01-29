/*MUFC-Multicopter UAV Flight Controller --Auto Levelling
 Modified by: Naufalino Fadel Hutomo and Adi Trisna
 Inspired By: J. Brokking, Oct 2016
 
*/
#define BUFFER_LENGTH 32		//for Wire function use

//library AVR
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//library C
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
 #include "twi.h"

///PID constant and limit///
float kp_roll	= 1.3;		//Proportional constant for roll
float ki_roll	= 0.04;		//Integrative constant for roll
float kd_roll	= 18.0;		//Diferentiative constant for roll
int pid_max_roll= 400;		//maximum output of PID for roll

float kp_pitch	= 1.3;	//Pitch
float ki_pitch	= 0.04;
float kd_pitch	= 18.0;
int pid_max_pitch	= 400;

float kp_yaw	= 4.00;	//yaw
float ki_yaw	= 0.02;
float kd_yaw	= 0.0;
int pid_max_yaw	= 400;

int autoLevel	= 1;			//Auto level 0 (false) ~0 (true)

///Variable for wire
uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;
uint8_t txBuffer[BUFFER_LENGTH];
uint8_t transmitting =0;

uint8_t dumm;
int dummInt;
/// variable for micros
unsigned long micros=0;

///Global variable///
uint8_t last_ch1, last_ch2, last_ch3, last_ch4;	
uint8_t eeprom_data[36]; 		
uint8_t highByte, lowByte; 	
volatile int rec_input_ch1, rec_input_ch2, rec_input_ch3, rec_input_ch4;
int counter_ch1, counter_ch2, counter_ch3, counter_ch4;
int esc1, esc2, esc3, esc4;
int throttle, batteryVolt;
int cal_int, start, gyroAdrr;
int rec_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust; 

long accX, accY, accZ, accTotalVector;
unsigned long timerCh1, timerCh2, timerCh3, timerCh4, escTimer, escLoopTimer;
unsigned long timer1, timer2, timer3, timer4, currentTime;
unsigned long loopTimer;
double gyroPitch, gyroRoll, gyroYaw;
double gyro_axis_call[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int gyro_angle_set;	//true or false


///Interrupt Routine to get micros
ISR(TIMER0_OVF_vect)
{
	micros+= 16;	
}

///Interrupt whenever get signal from receiver
ISR(PCINT0_vect){
  currentTime = micros;
  //Channel 1=========================================
  if(PINB & 0b00000001){                                                     //Is input 8 high?
    if(last_ch1 == 0)
	{                                                //Input 8 changed from 0 to 1.
      last_ch1 = 1;                                                   //Remember current input state.
      timer1 = currentTime;                                               //Set timer_1 to currentTime.
	
	}
  }
  else if(last_ch1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_ch1 = 0;                                                     //Remember current input state.
    rec_input[1] = currentTime - timer1;                             //Channel 1 is currentTime - timer_1.
  }
  //Channel 2=========================================
  if(PINB & 0b00000010 ){                                                    //Is input 9 high?
    if(last_ch2 == 0){                                                //Input 9 changed from 0 to 1.
      last_ch2 = 1;                                                   //Remember current input state.
      timer2 = currentTime;                                               //Set timer_2 to currentTime.
    }
  }
  else if(last_ch2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_ch2 = 0;                                                     //Remember current input state.
    rec_input[2] = currentTime - timer2;                             //Channel 2 is currentTime - timer_2.
  }
  //Channel 3=========================================
  if(PINB & 0b00000100 ){                                                    //Is input 10 high?
    if(last_ch3 == 0){                                                //Input 10 changed from 0 to 1.
      last_ch3 = 1;                                                   //Remember current input state.
      timer3 = currentTime;                                               //Set timer_3 to currentTime.
    }
  }
  else if(last_ch3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_ch3 = 0;                                                     //Remember current input state.
    rec_input[3] = currentTime - timer3;                             //Channel 3 is currentTime - timer_3.

  }
  //Channel 4=========================================
  if(PINB & 0b00001000 ){                                                    //Is input 11 high?
    if(last_ch4 == 0){                                                //Input 11 changed from 0 to 1.
      last_ch4 = 1;                                                   //Remember current input state.
      timer4 = currentTime;                                               //Set timer_4 to currentTime.
    }
  }
  else if(last_ch4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_ch4 = 0;                                                     //Remember current input state.
    rec_input[4] = currentTime - timer4;                             //Channel 4 is currentTime - timer_4.
  }
}

///Setup routine///
void setup( void)
{
	intSetup();
	//copy EEPROM data
	for(start = 0; start<=35; start++)
	{
		eeprom_data[start] = eepromRead(start);
	}
	
	start=0;			//set start back to zero
	gyroAdrr= eeprom_data[32];		//store gyro address from previous setup
	
	wireBegin();			//I2c as master
	
	TWBR = 12;		//I2C clock to 400kHz
	
	///IO config
	DDRD 	|= 0b11110000;			//PD4, PD5, PD6, PD7, as output to ESC
	DDRB	|= 0b00110000;			// pin 8,9,10,11 or PB0-PB3 as input from receiver, Pin 12 and 13 (PB4, PB5) as output
		
	//use LED in Arduino for indication
	PORTB	|= (1 << 5);	//arduino pin 13
		
	//check EEPROM signature to make sure the setup is executed
	while(eeprom_data[33]!='J' || eeprom_data[34] !='M' ||eeprom_data[35]!= 'B' ) _delay_ms(10);
	
	//if setup is completed without MPU, stop flight controller
	if (eeprom_data[31]==2 || eeprom_data[31]==3) _delay_ms(10);
	

	set_gyro_registers();		//Set the specific gyro registers
	
	for (cal_int=0; cal_int < 1250; cal_int ++)	//wait 5 seconds before continue
	{
		PORTD |= 0b11110000;
		_delay_ms(1);
		PORTD &= 0b00001111;
		_delay_ms(3);
	}
	
	///Determine gyro offset
	for (cal_int = 0; cal_int <2000; cal_int ++)
	{
		if (cal_int % 15 ==0) 
		{
			PORTB ^= (0b00010000);	//blink LED
		}
		gyro_signalen();					//read gyro output
		gyro_axis_call[1]	+= gyro_axis[1];		//add roll value
		gyro_axis_call[2]	+= gyro_axis[2];		//add pitch value
		gyro_axis_call[3]	+= gyro_axis[3];		// add yaw value
		// give pulse to esc, biar gak bunyi
		PORTD |= 0b11110000;
		_delay_ms(1);
		PORTD &= 0b00001111;
		_delay_ms(3);
	}
	//divide by 2000, from 2000 measurement to have gyro offset
	gyro_axis_call[1] /= 2000;			//roll value
	gyro_axis_call[2] /= 2000;			//pitch value
	gyro_axis_call[3] /= 2000;			//yaw value
	
	///Interrupt setup
	PCICR 	|= (1 << PCIE0);		//enable PCMSK0 scan
	PCMSK0 	|= (1 << PCINT0);		//set PCINT0 / PD8 in Uno to trigger interrupt
	PCMSK0 	|= (1 << PCINT1);		//set PCINT1 / PD9 in Uno to trigger interrupt
	PCMSK0 	|= (1 << PCINT2);		//set PCINT2 / PD10 in Uno to trigger interrupt
	PCMSK0 	|= (1 << PCINT3);		//set PCINT3 / P11 in Uno to trigger interrupt
	
	//Arming Mode. wait until receiver active and throttle in lowest value
	while (rec_input_ch3 < 990 || rec_input_ch3 > 1020 || rec_input_ch4 < 1400)
	{
		rec_input_ch3 = convert_receiver_ch(3);		//convert the ch3/ throttle to normalized value
		rec_input_ch4 = convert_receiver_ch(4);		// idem, ch4/yaw
		start++;		// while waiting, increment
		
		//ngilangin bunyi esc, kasih pulse
		PORTD |= 0b11110000;
		_delay_ms(1);
		PORTD &= 0b00001111;
		_delay_ms(3);
		if(start ==125)
		{
			PORTB ^= 0b00010000;		//blink LED every 500 ms or 125 loop
			start=0;		//start again at 0
		}
	}
	start=0;			//set start to 0
	
	loopTimer = micros;		//set timer for next loop
	PORTB &= (0 << 5);		//set led to LOW when finished setup	
}

int main ()
{	
	setup();
	
	///Main Loop
	while(1)
	{
		//65.5 = 1 deg/s, we get it from MPU 6050 datasheet
		gyro_roll_input = (gyro_roll_input * 0.7) + ((gyroRoll/65.5) *0.3);		//convert the value, so gyro pid input is in deg/s
		gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyroPitch/65.5) *0.3);		//convert the value, so gyro pid input is in deg/s
		gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyroYaw/65.5) *0.3);		//convert the value, so gyro pid input is in deg/s
		
		//Calculate angle in gyro. 	0.0000611 = 1 /(250 Hz / 65.5)
		angle_pitch += gyroPitch * 	0.0000611;
		angle_roll	+= gyroRoll *	0.0000611;
		
		//because sin is in radian, convert to radian. 0.000001066 = 0.0000611 * 3.142 <pi> / 180 deg. Calculate angle if IMU has yaw effect, so it give side effect in other angle. Untuk lebih jelas, harus melihat geometri dari Roll, yaw, dan pitch
		angle_pitch	-= angle_roll * sin(gyroYaw * 0.000001066);
		angle_roll	+= angle_pitch * sin(gyroYaw * 0.000001066);
		
		//accel angle calculation
		accTotalVector =  sqrt((accX*accX) + (accY*accY) + (accZ*accZ));
		
		if (abs(accY) < accTotalVector)		//pitch angle. absolute to prevent NaN in asin function
		{
			angle_pitch_acc = asin( (float) accY/accTotalVector) * 57.296;
		}
		
		if (abs(accX) < accTotalVector)		//roll angle
		{
			angle_roll_acc = asin( (float) accX/accTotalVector) * -57.296;
		}
		
		// place MPU spirit level
		angle_pitch_acc -= 0.0;		//accel clibration angle pitch
		angle_roll_acc -= 0.0;
		
		angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;		//correct the drift of gyro angle with accel angle
		angle_roll	= angle_roll  * 0.9996 + angle_roll_acc  * 0.0004;

		pitch_level_adjust = angle_pitch * 15;			//calculate the pitch angle correction
		roll_level_adjust = angle_roll * 15;			//calculate the pitch angle correction
		
		if(!autoLevel)
		{
			pitch_level_adjust = 0;
			roll_level_adjust = 0;
		}
		
		///Arming. Throttle low, yaw left
		if (rec_input_ch3 < 1050 && rec_input_ch4 <1050) start=1;
		//when yaw is back to center, motor start
		if (start==1 && rec_input_ch3 < 1050 && rec_input_ch4 > 1450 )
		{
			start =2;
			
			angle_pitch = angle_pitch_acc;		//gyro angle eq to accel angle when the quad is started
			angle_roll = angle_roll_acc;
			gyro_angle_set = 1;
			
			//Resete PID for bumpless
			pid_i_mem_pitch = 0;
			pid_i_mem_roll = 0;
			pid_i_mem_yaw = 0;
			pid_last_pitch_d_error = 0;
			pid_last_roll_d_error = 0;
			pid_last_yaw_d_error = 0;
		}
		
		/// DISARMING. Throttle low, yaw right
		if (start==2 && rec_input_ch3 < 1050 && rec_input_ch4 > 1950 ) start=0;
		
		//PID setpoint in deg/s.
		pid_roll_setpoint=0;
		//dead band of 16us
		if (rec_input_ch1 > 1508) pid_roll_setpoint=rec_input_ch1-1508;
		else if (rec_input_ch1 < 1492) pid_roll_setpoint = rec_input_ch1 -1492;
		
		pid_roll_setpoint -= roll_level_adjust;		//substract the angle correction from the standardized value
		pid_roll_setpoint /= 3.0;				// to get angle in deg/s
		
		//PID setpoint in deg/s.
		pid_pitch_setpoint=0;
		//dead band of 16us
		if (rec_input_ch2 > 1508) pid_pitch_setpoint=rec_input_ch2-1508;
		else if (rec_input_ch2 < 1492) pid_pitch_setpoint = rec_input_ch2 -1492;
		
		pid_pitch_setpoint -= pitch_level_adjust;		//substract the angle correction from the standardized value
		pid_pitch_setpoint /= 3.0;				// to get angle in deg/s
		
		//PID setpoint in deg/s.
		pid_yaw_setpoint=0;
		//dead band of 16us
		if (rec_input_ch3 > 1050)
		{
			if (rec_input_ch4 > 1508) pid_yaw_setpoint= (rec_input_ch4-1508)/3.0;
			else if (rec_input_ch4 < 1492) pid_yaw_setpoint = (rec_input_ch4 -1492)/3.0;
		}
		
		calculate_pid();
		
		throttle = rec_input_ch3;		//as base signal
		
		if (start==2)
		{
			if (throttle > 1800) throttle =1800;
			esc1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
			esc2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
			esc3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
			esc4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;
			
			//limiting the pulse signal
			if (esc1 <1100) esc1=1100;
			if (esc2 <1100) esc2=1100;
			if (esc3 <1100) esc3=1100;
			if (esc4 <1100) esc4=1100;
			
			if (esc1 > 2000) esc1=2000;
			if (esc2 > 2000) esc2=2000;
			if (esc3 > 2000) esc3=2000;
			if (esc4 > 2000) esc4=2000;	
		}
		else	// still arming mode
		{
			esc1 = 1000;
			esc2 = 1000;
			esc3 = 1000;
			esc4 = 1000;
		}
		
		//looptime
		if(micros- loopTimer > 4050) PORTB &= (1 << 5); 		//LED high
		
		//esc pulse every 4ms, because refresh rate is 250Hz
		while(micros-loopTimer < 4000);
		loopTimer = micros;
		
	PORTD |= 0b11110000;
	timer1 = esc1 + loopTimer;
	timer2 = esc2 + loopTimer;
	timer3 = esc3 + loopTimer;
	timer4 = esc4 + loopTimer;
	
	gyro_signalen();
	
	}
	return 0;
}

void set_gyro_registers(void)
{
	// Setup MPU 6050
	if (eeprom_data[31] == 1)
	{
		wireBeginTransmission(gyroAdrr);
		dummInt = wireWrite(0x6B);		//write to PWR_MGMT_1 register, 6B hex
		dummInt = wireWrite (0x00);		// acticvate gyro
		dumm = wireEndTransmission();
		
		wireBeginTransmission(gyroAdrr);
		dummInt = wireWrite(0x1B);		// GYRO_CONFIG register
		dummInt = wireWrite(0x08);		//set as 00001000 equal to 500 dps
		dumm = wireEndTransmission();
		
		wireBeginTransmission(gyroAdrr);
		dummInt = wireWrite(0x1C);		// ACCEL_CONFIG register
		dummInt = wireWrite(0x10);		//set as 00010000 equal to +- 8 g full scale
		dumm = wireEndTransmission();	
		
		//check 
		wireBeginTransmission(gyroAdrr);
		dummInt = wireWrite(0x1B);
		dumm = wireEndTransmission();
		dumm = wireRequestFrom(gyroAdrr,1);		//request 1 byte
		while (wireAvailable() < 1);
		if (wireRead() != 0x08)
		{
			PORTB |= (1 << 5);		// LED High
			while(1) _delay_ms(10);
		}
		
		wireBeginTransmission(gyroAdrr);
		dummInt = wireWrite(0x1A);		//write CONFIG register
		dummInt = wireWrite(0x03);		// set bits to 00000011
		dumm = wireEndTransmission();
		
	}
	
}

void gyro_signalen	()
{
	//Read MPU 6050
	if (eeprom_data[31] ==1)
	{
		wireBeginTransmission(gyroAdrr);
		dummInt = wireWrite(0x3B);
		dumm = wireEndTransmission();
		dumm = wireRequestFrom(gyroAdrr,14);		//request 14 bytes from gyro
		
		rec_input_ch1	= convert_receiver_ch(1);
		rec_input_ch2	= convert_receiver_ch(2);
		rec_input_ch3	= convert_receiver_ch(3);
		rec_input_ch4	= convert_receiver_ch(4);
		
		while (wireAvailable() < 14);
		acc_axis[1] = wireRead()<<8 | wireRead();                               //Add the low and high byte to the acc_x variable.
		acc_axis[2] = wireRead() <<8| wireRead();                               //Add the low and high byte to the acc_y variable.
		acc_axis[3] = wireRead() <<8| wireRead();                               //Add the low and high byte to the acc_z variable.
		temperature = wireRead() <<8| wireRead();                               //Add the low and high byte to the temperature variable.
		gyro_axis[1] = wireRead() <<8| wireRead();                              //Read high and low part of the angular data.
		gyro_axis[2] = wireRead() <<8| wireRead();                              //Read high and low part of the angular data.
		gyro_axis[3] = wireRead() <<8| wireRead();                              //Read high and low part of the angular data.
	}
	
	if (cal_int == 2000)	//only compensate after calibration
	{
		gyro_axis[1] -= gyro_axis_call[1];
		gyro_axis[2] -= gyro_axis_call[2];
		gyro_axis[3] -= gyro_axis_call[3];
	}
	
	gyroRoll 	= gyro_axis[eeprom_data[28] & 0b00000011];
	if (eeprom_data[28] & 0b10000000) 	gyroRoll *= -1;			//invert if MSB of EEPROM bit 28 is set
	gyroPitch 	= gyro_axis[eeprom_data[29] & 0b00000011];
	if (eeprom_data[29] & 0b10000000) 	gyroPitch *= -1;		//invert if MSB of EEPROM bit 28 is set
	gyroYaw 	= gyro_axis[eeprom_data[30] & 0b00000011];
	if (eeprom_data[30] & 0b10000000) 	gyroYaw *= -1;			//invert if MSB of EEPROM bit 28 is set

	accX = acc_axis[eeprom_data[29] & 0b00000011];		//set accX to the correct axis that was stored in EEPROM
	if (eeprom_data[29] & 0b10000000) accX *= -1;
	accY = acc_axis[eeprom_data[28] & 0b00000011];		//set accY to the correct axis that was stored in EEPROM
	if (eeprom_data[28] & 0b10000000) accY *= -1;
	accZ = acc_axis[eeprom_data[30] & 0b00000011];		//set accZ to the correct axis that was stored in EEPROM
	if (eeprom_data[30] & 0b10000000) accZ *= -1;
}

///convert the actual receiver signal to normalized 1000-2000 us. Useful to know whether the channel is inversed, overvalued, or undervalued
int convert_receiver_ch (int x)
{
	uint8_t channel, reverse;
	int low, center, high, actual, dif;
	
	channel = eeprom_data[x+23] & 0b00000111;	//What channel corresponds with the asked variable x
	if (eeprom_data[x+23] & 0b10000000) reverse =1;		//reverse channel when msb is set
	else reverse=0;
	
	actual = rec_input[channel];
	low = (eeprom_data[channel*2 + 15] <<8) | eeprom_data[channel *2 +14];		//store low value
	center = (eeprom_data[channel*2 -1] <<8) | eeprom_data[channel *2 - 2];		//store center value
	high = (eeprom_data[channel*2 + 7] <<8) | eeprom_data[channel *2 + 6];		//store high value
	
	if (actual < center)
	{
		if (actual<low) actual=low;		// limit to the value which determined in setup configuration before
		dif = ((long) (center-actual) * (long) 500) / (center-low);		// scaling to a 1000-2000 us
		if(reverse==1) return 1500+dif;
		else return 1500-dif;
	}
	else if (actual>center)
	{
		if(actual>high) actual=high; 	// limit to the value shich determined in setup configuration
		dif = ((long) (actual-center) * (long) 500) / (high-center);
		if(reverse==1) return 1500-dif;
		else return 1500+dif;
	}
	else return 1500;

}

///PID
void calculate_pid ()
{
	//roll
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += ki_roll*pid_error_temp;
	if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
	else if (pid_i_mem_roll < pid_max_roll *-1) pid_i_mem_roll = pid_max_roll *-1;
	
	pid_output_roll = kp_roll * pid_error_temp + pid_i_mem_roll + kd_roll * (pid_error_temp - pid_last_roll_d_error);
	if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;

	///Pitch
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += ki_pitch * pid_error_temp;
	if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = kp_pitch * pid_error_temp + pid_i_mem_pitch + kd_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;

	///Yaw 
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += ki_yaw * pid_error_temp;
	if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = kp_yaw * pid_error_temp + pid_i_mem_yaw + kd_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
}

void wireBegin()
{
	rxBufferIndex = 0;
	rxBufferLength = 0;

	txBufferIndex = 0;
	txBufferLength = 0;

	twi_init();	
}

int wireBeginTransmission(int address)
{
	// indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = (uint8_t) address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
	
}

int wireWrite(int data)
{
	if(transmitting)		// in master transmitter mode
	{				
		if(txBufferLength >= BUFFER_LENGTH)
		{
		  return 0;
		}
		// put byte in tx buffer
		txBuffer[txBufferIndex] = data;
		++txBufferIndex;
		// update amount in buffer   
		txBufferLength = txBufferIndex;
	}else
	{
	  // in slave send mode
		// reply to master
		twi_transmit(&data, 1);
	}
	return 1;
}

int wireEndTransmission()
{
	// transmit buffer (blocking)
  uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, 1);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
	
}

int wireRequestFrom (int address, int quantity) 
{
	if(quantity > BUFFER_LENGTH){
		quantity = BUFFER_LENGTH;
	}
  
	// perform blocking read into buffer
	uint8_t read = twi_readFrom(address, rxBuffer, quantity, 1);
	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = read;

	return read;
	
}

int wireAvailable()
{
	return rxBufferLength - rxBufferIndex;
}

int wireRead ()
{
	int value = -1;
	  
	// get each successive byte on each call
	if(rxBufferIndex < rxBufferLength)
	{
		value = rxBuffer[rxBufferIndex];
		++rxBufferIndex;
	}
	return value;
}

void intSetup (void)
{
	//Timer 0 for micros
	TCCR0B |= (1<<CS00); //prescaler 1. Interrupt when overflow, at 1/16MHz*2^8=16us
	TIMSK0 |= (1<<TOIE0); //overflow interrupt enable
	
    sei();
}
