//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Code updated by Cara Nunez 4.17.19
//--------------------------------------------------------------------------
// Parameters that define what environment to render
//#define ENABLE_VIRTUAL_LEFT_WALL
//#define ENABLE_VIRTUAL_RIGHT_WALL
//#define ENABLE_LINEAR_DAMPING
//#define ENABLE_NONLINEAR_FRICTION
//#define ENABLE_HARD_SURFACE
//#define ENABLE_BUMP_VALLEY
//#define ENABLE_TEXTURE

// Includes
#include <math.h>

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

// Kinematics variables
double xh = 0;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

String incoming = "";

int leftPin = 13;
int okPin = 12;
int rightPin = 11;

double xd = 0;
double xe = 0;
double xe1 = 0;
double dxe = 0;
double ixe = 0;
double xe_prev = 0;
int setting = 0;

double force1 = 0;
double x_wall = 0;

int walling = 0;
int count = 0;
double xh0 = 0;
// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A

  pinMode(leftPin, OUTPUT);
  pinMode(okPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);

 initialLoop();
  //digitalWrite(leftPin, HIGH);
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    if(rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber*rawOffset; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber*lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is 
    flipped = false;
  }
 
  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE (Use your code from Problems 1 and 2)
  // Define kinematic parameters you may need
  double rh = 0.09;   //[m]
  double ts = 0.0148*updatedPos + 10;
  if (count == 0){
    xh0 = rh*(ts*3.14159/180) - 0.0285192;
    count = count + 1;
  }
  xh = (rh*(ts*3.14159/180) - 0.0285192) - xh0;
  //Serial.println(xh, 5);
  // Calculate velocity with loop time estimation
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9*dxh + 0.1*dxh_prev; 
    
  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;
  
  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;
  
  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;

  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
  if (Serial.available() > 0){
    incoming = Serial.readString();
    int str_length = incoming.length();
    String option = incoming.substring(0,2);
    if (option == "01"){
      send_data(xh, dxh_filt_prev);  
    } else if (option == "02"){
      xd = incoming.substring(2, str_length).toDouble();
      xe = xd - xh;
      setting = 1;
      Serial.println("pos");
    } else if(option == "03"){
      force = incoming.substring(2, str_length).toDouble();
      Serial.println("force");
    } else if(option == "04"){
      x_wall = incoming.substring(2, str_length).toDouble();
      walling = 1;
      Serial.println("wall");
    } else if(option == "05"){
      walling = -1;
      Serial.println("wall off");
    } else if (option == "06"){
      x_wall = incoming.substring(2, str_length).toDouble();
      walling = 2;
      Serial.println("dwall");
    } else if (option == "10"){
      left_LED_off();
      Serial.println("ok");
      //left_wall(incoming.substring(1,str_length));
    } else if (option == "11"){
      left_LED_on();
      Serial.println("ok");
    } else if (option == "20"){
      middle_LED_off(); 
      Serial.println("ok"); 
    } else if (option == "21"){
      middle_LED_on();  
      Serial.println("ok");
    } else if (option == "30"){
      right_LED_off();  
      Serial.println("ok");
    } else if (option == "31"){
      right_LED_on();
      Serial.println("ok");
    } else if (option == "40"){
      left_LED_off();
      middle_LED_off();
      right_LED_off();
      Serial.println("ok");
    } else if (option == "41"){
      all_LEDs_on();
      Serial.println("ok");
    } else {
      left_LED_off();
      middle_LED_off();
      right_LED_off(); 
      setting = 0;
      force = 0; 
      walling = 0;
      count = 0;
      Serial.println("reset");
    }
  }

  if (setting == 1){
    xe = (xd - xh)*0.8;
    dxe = xe - xe_prev;
    ixe = ixe + xe;
    xe_prev = xe1;
    force = xe*0.01 + dxe*0.01 + ixe*0.1; 
    //Serial.println(force);
  }
  
  if ((abs(xe) < 0.0001 && abs(dxe) < 0.001) && setting == 1){
//    Serial.println("----");
//    Serial.println(xe);
//    Serial.println(dxe);
//    Serial.println(xh);
//    Serial.println("----");
    setting = 0;
    force = 0;
    xe = 0;
    xe_prev = 0;
    ixe = 0;
  }

  if (walling == 1){
    set_wall(x_wall, xh);
  } else if (walling == 2){
    set_double_wall(x_wall, xh, dxh_filt);
  } else if (walling == -1){
    force = 0;  
  }

      
  // Define kinematic parameters you may need
  double rp = 0.005;   //[m]
  double rs = 0.074;   //[m]
  Tp = force*(rh*rp)/rs;
  
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal

  Serial.flush();
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void initialLoop(){
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  delay(10000);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  delay(10000);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);  
  delay(10000);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  delay(10000);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);  
  delay(10000);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  delay(10000);
  Serial.println("00");
  }

void send_data(double xh, double dxh_filt_prev){
  Serial.println(String(xh,4)+"%"+String(dxh_filt_prev,4));
  }

void set_wall(double x_wall, double xh){
  double k_wall = 300;
  if (x_wall > 0){
    if (xh > x_wall){
      force = k_wall*(x_wall - xh);
    } else {
      force = 0;  
    }
    //Serial.println(force);
    //Serial.println(data);
  } else if (x_wall < 0){
    if (xh < x_wall){
      force = k_wall*(x_wall - xh); 
    } else {
      force = 0;      
    }
  }
  }

void set_double_wall(double x_wall, double xh, double vel){
  double k_wall = 200;
  double b = 27.5;
  if (x_wall > 0){
    if (xh > x_wall){
      force = k_wall*(x_wall - xh);
    } else if (xh < -x_wall){
      force = k_wall*(-x_wall - xh);
    } else {
      force = 0;  
    }
  } else if (x_wall < 0){
    if (xh < x_wall){
      force = k_wall*(x_wall - xh); 
    } else if (xh > -x_wall) { 
      force = k_wall*(-x_wall - xh); 
    } else {
      force = 0;      
    }
  }
  }
  
void left_LED_off(){
  digitalWrite(leftPin, LOW);
  }

void left_LED_on(){
  digitalWrite(leftPin, HIGH);
  }

void middle_LED_off(){
  digitalWrite(okPin, LOW);
  }

void middle_LED_on(){
  digitalWrite(okPin, HIGH);
  }
  
void right_LED_off(){
  digitalWrite(rightPin, LOW);
  }

void right_LED_on(){
  digitalWrite(rightPin, HIGH);
  }

void all_LEDs_on(){
  digitalWrite(leftPin, HIGH);
  digitalWrite(okPin, HIGH);
  digitalWrite(rightPin, HIGH);
  }

void all_LEDs_off(){
  digitalWrite(leftPin, LOW);
  digitalWrite(okPin, LOW);
  digitalWrite(rightPin, LOW);
  }
