/****************************************************
A basic implementation of PID control on line follower
robot. 

Görkem Meydan 2016
****************************************************/
int Kp,Ki,Kd;
int P,I,D;
int sensor1, sensor2, sensor3, sensor4, a, sensor5, sensor6,correct;
int pwmR,pwmL;
int error,last_error = 0;
int error_sum = 0;
int i = 0;
....................................................................................................................................................................................................................................................
void setup() {
Serial.begin(9600);

//setting the pinmodes
pinMode(10, OUTPUT); //Pin C2 Driving Motor Output 2 Right
pinMode(9, OUTPUT); //Pin C1 Driving Motor Output 1 Left
pinMode(2,OUTPUT); //Pin C4 Driving Motor Output 2 Right 
pinMode(3,OUTPUT); //Pin C5 Driving Motor Output 2 Right 
pinMode(4, OUTPUT); //Pin A4 Driving Motor Output 2 Left 
pinMode(5, OUTPUT); //Pin A5 Driving Motor Output 1 Left
pinMode(14, INPUT); //QTR Sensor Number 2 (+3)
pinMode(15, INPUT); //QTR Sensor Number 3 (+2)
pinMode(16, INPUT); //QTR Sensor Number 4 (+1)
pinMode(17, INPUT); //QTR Sensor Number 5 (-1)
pinMode(18, INPUT); //QTR Sensor Number 6 (-2)
pinMode(19, INPUT); //QTR Sensor Number 7 (-3)
}

void loop() {

//reading the QTR-8A sensor data
sensor1=analogRead(14); //Right
sensor2=analogRead(15);   
sensor3=analogRead(16);
sensor4=analogRead(17);  
sensor5=analogRead(18);
sensor6=analogRead(19); //Left

//changing analog data to digital with the threshold 512
if(sensor1<=512)
{
  sensor1=1;
}else
{
  sensor1=0;
}

if(sensor2<=512)
{
  sensor2=1;
}else
{
  sensor2=0;
}

if(sensor3<=512)
{
  sensor3=1;
}else
{
  sensor3=0;
}

if(sensor4<=512)
{
  sensor4=1;
}else
{
  sensor4=0;
}

if(sensor5<=512)
{
  sensor5=1;
}else
{
  sensor5=0;
}

if(sensor6<=512)
{
  sensor6=1;
}else
{
  sensor6=0;
}

//giving weighted values to the sensor inputs
sensor1=(+11)*sensor1; //Right
sensor2=(+4)*sensor2;  
sensor3=(+2)*sensor3;
sensor4=(-2)*sensor4;  
sensor5=(-4)*sensor5;
sensor6=(-11)*sensor6; //Left

error=sensor1+sensor2+sensor3+sensor4+sensor5+sensor6; //Lateral error from line

Kp = 10; //Proportional Constant
Kd = 4; //Derrivative Constant
Ki = 0.001; //Integral Constant

P = error*Kp;
I = Ki*error_sum;
D = Kd*(error-last_error);

correct=P + I +D; //PID

pwmL=110+correct; //pwm values
pwmR=130-correct;
/*notice that starting PWM values of the right and the left motor are different,
because even they are fabricated they are not exactly the same power. */

/*setting the range [0,255] for the computed PWM values for both left and 
right motors.*/
if (pwmL > 255) 
{
  pwmL=255;
}
else if (pwmL<0) 
{
  pwmL=0;
}

if (pwmR > 255) 
{
  pwmR=255;
}
else if (pwmR<0) 
{
  pwmR=0;
}

/*if the robot goes out of the circuit, it goes back until it sees the line again*/
if (sensor1==0 && sensor2==0 && sensor3==0 && sensor4==0 && sensor5==0 && sensor6==0)
{
  delay(200);
  if (sensor1==0 && sensor2==0 && sensor3==0 && sensor4==0 && sensor5==0 && sensor6==0) 
  {
    digitalWrite(2, HIGH); //power outputs
    digitalWrite(3,LOW);
    digitalWrite(4, LOW);
    digitalWrite(5,HIGH);
    analogWrite(9, pwmL);
    analogWrite(10, pwmR);
    delay(300);
    i++;
    if(i > 15 && i < 20)
    {
      digitalWrite(2, LOW); //power outputs
      digitalWrite(3,HIGH);
      digitalWrite(4, HIGH);
      digitalWrite(5,LOW);
      analogWrite(9, pwmL);
      analogWrite(10, pwmR);
      delay(300);
    }
    else if (i > 20)
    {
      delay(1);
    }
  }
}
 
  //giving motor driver setup
  digitalWrite(2, LOW); 
  digitalWrite(3,HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5,LOW);
  analogWrite(9, pwmL);
  analogWrite(10, pwmR);

  //error values for integral and derriative
  error_sum += error;
  last_error = error;

}
