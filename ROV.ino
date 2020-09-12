#include <Wire.h>
#include <math.h>
#include <L3G4200D.h>
#define Addr 0x53
L3G4200D gyro; 

// Setup
float Theta_LP_a,Phi_LP_a;
float Phi_HP_g,Theta_HP_g,Psi_HP_g;
float xm,ym,zm;
float dotPhi,dotTheta,dotPsi;
float Phi_g    = 0;
float Theta_g  = 0;
float Psi_g    = 0;
float offset_x = -7.00;
float offset_y = 95.000;
float offset_z = 82.500;
float scale_x  = 0.968;
float scale_y  = 1.047;
float scale_z  = 0.987;
void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  gyro.enableDefault();
  Wire.beginTransmission(Addr);
  Wire.write(0x2C);
  Wire.write(0x0A);
  Wire.endTransmission();
  Wire.beginTransmission(Addr);
  Wire.write(0x2D);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(Addr);
  Wire.write(0x31);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(Addr);
  Wire.write(0x1F); // Y-axis offset register
  Wire.write(2);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(Addr);
  Wire.write(0x20); // Z-axis offset register
  Wire.write(4);
  Wire.endTransmission();
  Wire.begin();
  Wire.beginTransmission(0x1E);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(0x1E);
  Wire.write(0x01);
  Wire.write(0xA0);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(0x1E);
  Wire.write(0x02);
  Wire.write(0);
  Wire.endTransmission();
  delay(300);
}

void loop() 
{
  float PreviousTime = millis();
  unsigned int data[6];
  Wire.beginTransmission(Addr);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(Addr,6,true);
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();
  data[4] = Wire.read();
  data[5] = Wire.read();
  // Convert the data to 10-bits
  float xa = (((data[1] & 0x03) * 256) + data[0]);
  if(xa > 511)
  {
    xa -= 1024;
  }
  float ya = (((data[3] & 0x03) * 256) + data[2]);
  if(ya > 511)
  {
    ya -= 1024;
  }
  float za = (((data[5] & 0x03) * 256) + data[4]);
  if(za > 511)
  {
    za -= 1024;
  }
  gyro.read();
  float xg = (int)gyro.g.x*0.00875f;
  float yg = (int)gyro.g.y*0.00875f;
  float zg = (int)gyro.g.z*0.00875f;
  Wire.requestFrom(0x1E,6);
  if (Wire.available() > 5)
  {
     xm = Wire.read()<<8|Wire.read();
     ym = Wire.read()<<8|Wire.read();
     zm = Wire.read()<<8|Wire.read();
  }
  Wire.endTransmission();
  Wire.beginTransmission(0x1E);
  Wire.write(0x03);
  Wire.endTransmission();
  
  float CurrentTime = millis();
  float dt          = (PreviousTime-CurrentTime)/1000;
  //
  // Magnetometer Calibration 
  xm = (xm-offset_x)*scale_x;
  ym = (ym-offset_y)*scale_y;
  zm = (zm-offset_z)*scale_z;
  


  float Theta_a = Pitch_a(xa,za); //acclerometer pitch angle
  float Phi_a   = Roll_a(ya,za); // acclerometer roll angle

  //
  // Gyroscope readings from body to inertial frame
  BodyToInertial(xg,yg,zg,Phi_a,Theta_a,&dotPhi,&dotTheta,&dotPsi);
  
  // 
  // Gyroscope DPS to angle conversion
  Phi_g   = Phi_g   + dt*dotPhi;
  Theta_g = Theta_g + dt*dotTheta;
  Psi_g   = Psi_g   + dt*dotPsi;
  
  Low_pass_filter(Theta_a,Phi_a,&Theta_LP_a,&Phi_LP_a); //Low pass filter the accelerometer data
  High_pass_filter(Phi_g,Theta_g,Psi_g,&Phi_HP_g,&Theta_HP_g,&Psi_HP_g); //High pass filter the gyroscope data

  float Phi_comp   = Phi_LP_a + Phi_HP_g;
  float Theta_comp = Theta_LP_a  + Theta_HP_g;
  String Data = String(Phi_comp)+","+String(Theta_comp)+","+String(Phi_a)+","+String(Theta_a);
  Serial.println(Data);  
}

//
// Pitch angle from accelerometer
float Pitch_a(double xa, double za)
{
  xa = xa*9.8/256;
  za = za*9.8/256;
  float result;
  result = atan2(xa,za)*(-180)/3.14;
  return result;
}

//
// Roll angle from accelerometer
float Roll_a(double ya, double za)
{
  ya = ya*9.8/256;
  za = za*9.8/256;
  float result;
  result = atan2(ya,za)*(180)/3.14;
  return result;
}

//
// Convert Gyroscope readings form body frame to inertial frame
void BodyToInertial(float p, float q, float r, float phi, float theta, float *dotPhi,float *dotTheta,float *dotPsi)
{
    float sinPhi   = sin((3.14*phi/180));
    float cosPhi   = cos((3.14*phi/180));
    float cosTheta = cos((3.14*theta/180));
    float tanTheta = tan((3.14*theta/180));

    *dotPhi   = p + q*sinPhi*tanTheta + r*cosPhi*tanTheta;
    *dotTheta = q*cosPhi              - r*sinPhi;
    *dotPsi   = q*sinPhi/cosTheta     + r*cosPhi/cosTheta;
    /* Example calling this function
    float dotPhi,dotTheta,dotPsi;
    BodyToInertial(p,q,r,phi,theta,&dotPhi,&dotTheta,&dotPsi)
    Serial.print(.....)*/
}

//
// Low pass filter
void Low_pass_filter(float Theta_a,float Phi_a,float *Theta_LP_a, float *Phi_LP_a)
{
    static float prevaX = Theta_a;
    static float prevaY = Phi_a;
    float alpha = 0.87;
    *Theta_LP_a = alpha*prevaX + (1-alpha)*Theta_a;
    *Phi_LP_a   = alpha*prevaY + (1-alpha)*Phi_a;
    prevaX = *Theta_LP_a;
    prevaY = *Phi_LP_a;
}

//
// High Pass filter
void High_pass_filter(float Phi_g,float Theta_g, float Psi_g,float *Phi_HP_g,float *Theta_HP_g,float *Psi_HP_g)
{
    static float prevxg = 0;
    static float prevyg = 0;
    static float prevzg = 0;
    static float prevxHF= 0;
    static float prevyHF= 0;
    static float prevzHF= 0;

    float alpha = 0.87;
    *Phi_HP_g    = alpha*prevxHF + alpha*(Phi_g - prevxg);
    *Theta_HP_g  = alpha*prevyHF + alpha*(Theta_g - prevyg);
    *Psi_HP_g    = alpha*prevyHF + alpha*(Psi_g - prevyg);
    prevxHF      = *Phi_HP_g;
    prevyHF      = *Theta_HP_g;
    prevzHF      = *Psi_HP_g;
    prevxg       = Phi_g;
    prevyg       = Theta_g;
    prevzg       = Psi_g;
}
//
