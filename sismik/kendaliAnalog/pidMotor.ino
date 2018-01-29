#define inputPin A3


byte PWM_pin = 3;

int pwmVal;

int inVal;
// f=16MHz, T=256/16MHz, prescaler 1024


/*working variables*/
unsigned long lastTime;
double inVal, outVal, setVal;
double errSum, lastE;
double kp, ki, kd;

//compute output PID
void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double dt = (double)(now - lastTime);
  
   /*Compute all the working error variables*/
   double e = setVal - inVal;
   errSum += (e * dt);
   double de = (e - lastE) / dt;
  
   /*Compute PID Output*/
   outVal = kp * error + ki * errSum + kd * de;
  
   /*Remember some variables for next time*/
   lastE = e;
   lastTime = now;
}
  
void SetTunings(double kpTune, double kiTune, double kdTune)
{
   kp = kpTune;
   ki = kiTune;
   kd = kdTune;
}
void setup() {

  //pin
  pinMode(PWM_pin, INPUT);
  pinMode(inputPin, INPUT);
  
}

void loop() {
  //read pwm
  pwmVal = pulseIn(PWM_pin, HIGH);
  setVal= pwmVal/16.384*1024; 

  //read input
  inVal= analogRead(inputPin);

  Compute();
  analogWrite(PWM_pin,outVal);

}
