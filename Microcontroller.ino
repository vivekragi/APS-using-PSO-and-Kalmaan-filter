/*
 * CHANNELS:PINS
 * 1:10
 * 2:11
 * 3:12
 * 4:13
 * 5:8
 * 6:9
 * 
 * ISR(PC0) IS USED TO CALCULATE THE PULSE LENGTH WHENEVER THE STATE OF ABOVE PIN CHANGES
 * IF STATEMENTS HAS THE INFORMATION ABOUT WHEN THE PULSE RISES
 * IF ELSE STATEMENTS CAPTURES THE AMOUNT OF TIME THAT THE PULSE WAS HIGH
 * 
 * ISR(PC1) IS USED AS A FLAG VARIABLE TO SWITCH BETWEEN AUTONOMOUS MODE AND PILOT MODE 
 * 
 * PINS:PORTS
 * 23:PORTA1
 * 53:PORTB0
 * 30:PORTC1
 * 32:PORTC3
 * 34:PORTC5
 * 36:PORTC7
 */

#include <Wire.h>


byte state_ch1, state_ch2, state_ch3, state_ch4, state_ch5, state_ch6;
int ch1, ch2, ch3, ch4, ch5, ch6;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, timer_pci0, timer_pci2;
unsigned long t0,t,t1,t2,t3,t4,e1,e2,e3,e4;
float state_of_charge,bias;

long ax,ay,az;
long gx,gy,gz;
long cx,cy,cz;
float mx,my,mz;
float fx,fy,fz;
float wx,wy,wz;
float q0 = 1.0;
float q1 = 0.0;
float q2 = 0.0;
float q3 = 0.0;
float eta = 12.4;
float invSampleFreq = 1.0/250.0;
float alpha, beta, gamma;
float alpha_cal = 0.0;
float beta_cal = 0.0;
float gamma_cal = 0.0;
float roll_cal = 0.0;
float pitch_cal = 0.0;
float yaw_cal = 0.0;
float cal = 500.0;

float p_roll = 1.2;
float p_pitch =0.0;// 1.6;
float p_yaw = 0.0;//1.0;
float i_roll = 0.1;//0.1;
float i_pitch = 0.0;//0.1;
float i_yaw = 0.0;//0.1;
float d_roll = 10.0;
float d_pitch = 0.0;//12.0;
float d_yaw = 0.0;//2.8;
float error_roll;
float error_pitch;
float error_yaw;
float error_sum_roll = 0.0;
float error_sum_pitch = 0.0;
float error_sum_yaw = 0.0;
float error_diff_roll = 0.0;
float error_diff_pitch = 0.0;
float error_diff_yaw = 0.0;
float max_roll = 400.0;
float max_pitch = 400.0;
float max_yaw = 400.0;
float gyro_roll,gyro_pitch,gyro_yaw;
float adjust_roll,adjust_pitch,adjust_yaw;
float setpoint_roll,setpoint_pitch,setpoint_yaw;
float pid_roll,pid_pitch,pid_yaw;


void pid(){
  //ROLL
  error_roll = gyro_roll - setpoint_roll;   //PROPORTIONAL
  error_sum_roll += error_roll;  //INTEGRATIVE
  if(error_sum_roll < -1*max_roll)
    error_sum_roll = -1*max_roll;  
  else if(error_sum_roll > max_roll)
    error_sum_roll = max_roll; 
  error_diff_roll = error_roll - error_diff_roll;  //DERIVATIVE   
  pid_roll = p_roll*error_roll + i_roll*error_sum_roll + d_roll*error_diff_roll;
  if(pid_roll > max_roll)
    pid_roll = max_roll;
  else if(pid_roll < -1*max_roll)
    pid_roll = -1*max_roll;     
  error_diff_roll = error_roll;

  //PITCH
  error_pitch = gyro_pitch - setpoint_pitch;  //PROPORTIONAL
  error_sum_pitch += error_pitch;   //INTEGRATIVE
  if(error_sum_pitch < -1*max_pitch)
    error_sum_pitch = -1*max_pitch;
  else if(error_sum_pitch > max_pitch)
    error_sum_pitch = max_pitch; 
  error_diff_pitch = error_pitch - error_diff_pitch;  //DERIVATIVE
  pid_pitch = p_pitch*error_pitch + i_pitch*error_sum_pitch + d_pitch*error_diff_pitch;
  if(pid_pitch > max_pitch)
    pid_pitch = max_pitch;
  else if(pid_pitch < -1*max_pitch)
    pid_pitch = -1*max_pitch;     
  error_diff_pitch = error_pitch;

  //YAW
  error_yaw = gyro_yaw - setpoint_yaw;  //PROPORTIONAL
  error_sum_yaw += error_yaw;  //INTEGRATIVE
  if(error_sum_yaw < -1*max_yaw)
    error_sum_yaw = -1*max_yaw;
  else if(error_sum_yaw > max_yaw)
    error_sum_yaw = max_yaw; 
  error_diff_yaw = error_yaw - error_diff_yaw;  //DERIVATIVE
  pid_yaw = p_yaw*error_yaw + i_yaw*(error_sum_yaw) + d_yaw*error_diff_yaw;
  if(pid_yaw > max_yaw)
    pid_yaw = max_yaw;
  else if(pid_yaw < -1*max_yaw)
    pid_yaw = -1*max_yaw;     
  error_diff_yaw = error_yaw;
}


void setupHMC(){
  Wire.beginTransmission(0b0011110);  //I2C ADDR
  Wire.write(0x00);                   //CONFIG REG A
  Wire.write(0x01110100);             //SAMPLE RATE TO 8,DATA RATE TO 30
  Wire.endTransmission();
  Wire.beginTransmission(0b0011110);
  Wire.write(0x01);                   //CONFIG REG B
  Wire.write(0x01100000);             //GAIN SET TO 660@+-2.5GAUSS 
  Wire.endTransmission();
}

void setupMPU(){
  
  Wire.beginTransmission(0b1101000);  //I2C ADDR
  Wire.write(0x6B);                   //POWER
  Wire.write(0b00000000);             //SLEEP REGISTER TO 0
  Wire.endTransmission();
  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                   //GYRO REGISTER
  Wire.write(0x00000000);             //250DEG/SEC sENSITIVITY
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                   //ACCEL REGISTER
  Wire.write(0b00001000);             //+-4G SENSITIVITY
  Wire.endTransmission();           
}

void gyro(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);                   //START REGISTER
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6);      //REQUEST 6 BITS FROM 3B
  while(Wire.available()<6);
  gx=Wire.read()<<8|Wire.read();      //FIRST 2 BYTES
  gy=Wire.read()<<8|Wire.read();      //SECOND 2 BYTES
  gz=Wire.read()<<8|Wire.read();      //THIRD 2 BYTES
  
  //RESOLUTION
  wx=(float)(gx/131.0);               
  wy=(float)(gy/131.0);
  wz=(float)(gz/131.0);
}

void accel(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);                   //START REGISTER
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6);      //REQUEST 6 BITS FROM 3B
  while(Wire.available()<6);
  ax=Wire.read()<<8|Wire.read();      //FIRST 2 BYTES
  ay=Wire.read()<<8|Wire.read();      //SECOND 2 BYTES
  az=Wire.read()<<8|Wire.read();      //THIRD 2 BYTES
  
  //RESOLUTION
  fx=(float)(ax/8192.0);
  fy=(float)(ay/8192.0);
  fz=(float)(az/8192.0);
}

void compass(){
   Wire.beginTransmission(0b0011110);
   Wire.write(0x02);                  //MODE REGISTER
   Wire.write(000000001);             //SETTING MODE REGISTER TO CONTINUOUS OUTPUT MODE 
   Wire.endTransmission(); 
  
   Wire.beginTransmission(0b0011110); 
   Wire.write(0x03);                  //START REGISTER
   Wire.endTransmission();

   Wire.requestFrom(0b0011110,6);
   while(Wire.available()<6);
   cx=Wire.read()<<8|Wire.read();
   cy=Wire.read()<<8|Wire.read();
   cz=Wire.read()<<8|Wire.read();

   //RESOLUTION
   mx=(float)(cx/1.52);
   my=(float)(cy/1.52);
   mz=(float)(cz/1.52);
}



void setup() {
  
  //SET PINS 13,12,11,10 AS PIN CHANGE INTERRUPT PINS
  PCICR |= (1 << PCIE0);    
  PCMSK0 |= (1 << PCINT7);   
  PCMSK0 |= (1 << PCINT6);  
  PCMSK0 |= (1 << PCINT5);  
  PCMSK0 |= (1 << PCINT4);

  //SET PINS 8,9 AS PIN CHANGE INTERRUPT PINS
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT16);
  PCMSK2 |= (1 << PCINT17);

  //COFIGURING OUTPUT PORTS 23,53,30,32,34,36
  DDRA |= B00000010;
  DDRB |= B00000001;
  DDRC |= B10101010;
  
  //SETTING UP AND INITIALIZING THE GYROSCOPE, ACCELEROMETER AND MAGNETOMETER SENSORS
  Wire.begin();
  setupMPU();
  setupHMC();

  //SERIAL DEBUGGING
  Serial.begin(9600);
  
  //CALCULATING DRIFT VALUES
  for(int i=0;i<cal;i++){
    computeAngles();
    alpha_cal += alpha;
    beta_cal += beta;
    gamma_cal += gamma; 
    gyro();
    roll_cal += wx;
    pitch_cal += wy;
    yaw_cal += wz;
    Serial.print(alpha);
    Serial.print(" ");
    Serial.print(beta);
    Serial.print(" ");
    Serial.print(" ");
    Serial.println(gamma);
  }
  alpha_cal /= cal;
  beta_cal /= cal;
  gamma_cal /= cal;
  roll_cal /= cal;
  pitch_cal /= cal;
  yaw_cal /= cal;
  
  int start = 0;
  while(ch3<992 || ch3>1050){
    start++;
    PORTC |= B10101010;
    delayMicroseconds(1000);
    PORTC &= B01010101;
    delay(3);
    if(start == 125){
      PORTA |= B00000010;
    }
    else if(start == 250){
      PORTA &= B11111101;
      start = 0;
    }
  }
  delay(2000);
  Serial.println("Start Arming");
  bias = -25.0;
  state_of_charge = (analogRead(11) + bias)*1.2317;
  t0 = micros();
}

void loop() {

  //APPLYING A TYPE OF COMPLEMENTARY FILTER TO GYRO READINGS FOR NOISE REDUCTION
  gyro_roll = 0.92*(gyro_roll - roll_cal) + 0.08*(wx);
  gyro_pitch = 0.92*(gyro_pitch - pitch_cal) + 0.08*(wy);
  gyro_yaw = 0.92*(gyro_yaw - yaw_cal) + 0.08*(wz);

  //APPLYING MADGWICK FILTER
  computeAngles();
  adjust_roll = (alpha)*20.0;
  adjust_pitch = (beta)*20.0;
  adjust_yaw = (gamma)*20.0;
  Serial.print(alpha);
  Serial.print(" ");
  Serial.print(beta);
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(gamma);
  Serial.print(" ");
  Serial.println(ch5);
  //INITIALIZING PID SET POINTS
  setpoint_roll = 0.0;
  if(ch1 < 1492)
    setpoint_roll = ch1 - 1492;
  else if(ch1 > 1508)
    setpoint_roll = ch1 -1508;
  setpoint_roll -= adjust_roll;
  setpoint_roll /= 3.0; 

  setpoint_pitch = 0.0;
  if(ch2 < 1492)
    setpoint_pitch = ch2 - 1492;
  else if(ch2 > 1508)
    setpoint_pitch = ch2 - 1508;
  setpoint_pitch -= adjust_pitch;
  setpoint_pitch /= 3.0; 

  setpoint_yaw = 0.0;
  if(ch4 < 1492)
    setpoint_yaw = ch4 - 1492;
  else if(ch4 > 1508)
    setpoint_yaw = ch4 - 1508;
  setpoint_yaw -= adjust_yaw;
  setpoint_yaw /= 3.0; 

  //PID FEEDBACK LOOP
  pid();

  //CALCULATING ESC PULSES
  int throttle = ch3;
  if(throttle > 1800)
    throttle = 1800;  
  e1 = throttle - pid_roll - pid_pitch + pid_yaw;  
  e2 = throttle + pid_roll - pid_pitch - pid_yaw;
  e3 = throttle + pid_roll + pid_pitch + pid_yaw;
  e4 = throttle - pid_roll + pid_pitch - pid_yaw;
  
  // BATTERY VOLTAGE COMPENSATION
  state_of_charge = state_of_charge*0.92 + (analogRead(11) + bias)*1.2317*0.08; //complementary filter
  if(state_of_charge < 1070 && state_of_charge > 400)
    PORTB |= B00000001; 
  else
    PORTB &= B11111110;

  if(ch5<=1100)
  {
    if(e1>=1900)
      e1=1900;
    else if(e1<=1100)
      e1=1050;
    
    if(e2>=1900)
      e2=1900;
    else if(e2<=1100)
      e2=1050;
      
    if(e3>=1900)
      e3=1900;
    else if(e3<=1100)
      e3=1050;
      
    if(e4>=1900)
      e4=1900;
    else if(e4<=1100)
      e4=1050;
  }
  else
  {
    e1=1000;
    e2=1000;
    e3=1000;
    e4=1000;    
  }
  //CREATING PULSE
  while(t0 + 11000 > micros());
  t0 = micros();
  PORTC |= B10101010;
  t1 = t0 + e1;
  t2 = t0 + e2;
  t3 = t0 + e3;
  t4 = t0 + e4;

  while(PORTC >= 2){
    t = micros();
    if(t1 <= t)PORTC &= B11111101;
    if(t2 <= t)PORTC &= B11110111;
    if(t3 <= t)PORTC &= B11011111;
    if(t4 <= t)PORTC &= B01111111;
  }
}

ISR(PCINT0_vect){
   timer_pci0 = micros();
   //************************************************** CHANNEL 1 **************************************************   
    if(state_ch1==0 && (PINB & B00010000)){         
      state_ch1 = 1;                                 
      timer_1 = timer_pci0;                                 
    }
    else if(state_ch1==1 && !(PINB & B00010000)){  
      state_ch1 = 0;                                 
      ch1 = timer_pci0 - timer_1;      
    }
   //************************************************** CHANNEL 2 **************************************************
    if(state_ch2==0 && (PINB & B00100000)){         
      state_ch2 = 1;                                 
      timer_2 = timer_pci0;                               
    }
    else if(state_ch2==1 && !(PINB & B00100000)){  
      state_ch2 = 0;                                 
      ch2 = timer_pci0 - timer_2;      
    }
   //************************************************** CHANNEL 3 **************************************************
    if(state_ch3==0 && (PINB & B01000000)){         
      state_ch3 = 1;                                 
      timer_3 = timer_pci0;                                 
    }
    else if(state_ch3==1 && !(PINB & B01000000)){  
      state_ch3 = 0;                                 
      ch3 = timer_pci0 - timer_3;      
    }
   //************************************************** CHANNEL 4 **************************************************
    if(state_ch4==0 && (PINB & B10000000)){         
      state_ch4 = 1;                                 
      timer_4 = timer_pci0;                                 
    }
    else if(state_ch4==1 && !(PINB & B10000000)){  
      state_ch4 = 0;                                
      ch4 = timer_pci0 - timer_4;     
    }
}

ISR(PCINT2_vect){
    timer_pci2 = micros();
  //************************************************** CHANNEL 5 **************************************************   
    if(state_ch5==0 && (PINK & B00000001)){         
      state_ch5 = 1;                                
      timer_5 = timer_pci2;
    }                                  
    else if(state_ch5==1 && !(PINK & B00000001)){  
      state_ch5 = 0;
      ch5 = timer_pci2 - timer_5;
    }                                       
   //************************************************** CHANNEL 6 **************************************************
    if(state_ch6==0 && (PINK & B00000010)){         
      state_ch6 = 1;
      timer_6 = timer_pci2;
    }
    else if(state_ch6==1 && !(PINK & B00000010)){
      state_ch6 = 0;                             
      ch6 = timer_pci2 - timer_6;
    }          
}



void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  
    //gyro from degrees to radians
    gx *= 0.0174533;
    gy *= 0.0174533;
    gz *= 0.0174533;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
     
      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;
  
      // Normalise magnetometer measurement
      recipNorm = invSqrt(mx * mx + my * my + mz * mz);
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;
  
      // Auxiliary variables to avoid repeated arithmetic
      _2q0mx = 2.0f * q0 * mx;
      _2q0my = 2.0f * q0 * my;
      _2q0mz = 2.0f * q0 * mz;
      _2q1mx = 2.0f * q1 * mx;
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _2q0q2 = 2.0f * q0 * q2;
      _2q2q3 = 2.0f * q2 * q3;
      q0q0 = q0 * q0;
      q0q1 = q0 * q1;
      q0q2 = q0 * q2;
      q0q3 = q0 * q3;
      q1q1 = q1 * q1;
      q1q2 = q1 * q2;
      q1q3 = q1 * q3;
      q2q2 = q2 * q2;
      q2q3 = q2 * q3;
      q3q3 = q3 * q3;
      
      // Reference direction of Earth's magnetic field
      hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
      hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
      _2bx = sqrtf(hx * hx + hy * hy);
      _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
      _4bx = 2.0f * _2bx;
      _4bz = 2.0f * _2bz;
  
      // Gradient decent algorithm corrective step
      s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      
      // normalise step magnitude
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;
  
      // Apply feedback step
      qDot1 -= eta * s0;
      qDot2 -= eta * s1;
      qDot3 -= eta * s2;
      qDot4 -= eta * s3;
    }
  
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;
  
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void computeAngles()
{
  gyro();
  accel();
  compass();
  update(wx,wy,wz,fx,fy,fz,-1*my,-1*mx,mz);
  alpha = atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
  beta = asin(-2.0 * (q1*q3 - q0*q2));
  gamma = atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
  alpha /= 0.0174533;
  beta /= 0.0174533;
  gamma /= 0.0174533;
}
