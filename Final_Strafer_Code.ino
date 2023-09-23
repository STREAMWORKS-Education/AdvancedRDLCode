//Code for the RDL Strafer Robot Kit
//Hookup guide:
//Motor 1(Front Left): 2
//Motor 2(Back Left): 3
//Motor 3(Back Right): 4
//Motor 4(Front Right): 5
//PS2 Pins refrenced down below in code (Lines 18-21)



#include <PS2X_lib.h> //link to connection guide http://www.billporter.info/2010/06/05/playstation-2-controller-arduino-library-v1-0/

#include <Servo.h>


#define RotateSensitivity .6  //rotation sensetivity, the higher the number the faster it rotates, max 1
//PS2 Pins (Black is ground and red is 3V3)
#define PS2_DAT        13  //brown 
#define PS2_CMD        11  //orange 
#define PS2_SEL        10  //yellow
#define PS2_CLK        12  //blue

#define pressures   false
#define rumble      false
int analogValue[4]; // variable's for joystick data 
Servo MTR1;  //defining ESC's
Servo MTR2;
Servo MTR3;
Servo MTR4;
Servo AMTR1;
Servo ASERVO; //defining Gripper Servo

int DPadV[4];

PS2X ps2x; // create PS2 Controller Class
float time;
int ASERVOVal = 1500;
int error = 0;
byte type = 0;
byte vibrate = 0;
bool motorSaftey = 0;

void setup() {
//setup for ESC
MTR1.attach(2); //FL
MTR2.attach(3); //BL
MTR3.attach(4); //BR
MTR4.attach(5); //FR
AMTR1.attach(7); //arm motor
ASERVO.attach(6); //gripper servo

ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

Serial.begin(57600);
}

void loop() {
ReadPS2(); //Get data from PS2 controller
motorZero(); //Zero the ESC's
armSend(); //Send data to Arm
motorSend(); //Send data to ESC's

delay(50); //delay for stability
}


void motorZero() {
MTR1.writeMicroseconds(1500); //zero point of the esc's
MTR2.writeMicroseconds(1500);
MTR3.writeMicroseconds(1500);
MTR4.writeMicroseconds(1500);
}

void ReadPS2() {

ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
  
  analogValue[0] = ps2x.Analog(PSS_LY); //left joystick, y-axis
 
  analogValue[1] = ps2x.Analog(PSS_LX); //left joystick, x-axis
   
  analogValue[2] = ps2x.Analog(PSS_RY); //right joystick, y-axis

  analogValue[3] = ps2x.Analog(PSS_RX); //right joystick, x-axis





 //detects if the right bumber has been pressed to toggle saftey, can only be toggled once a second
  if(ps2x.Button(PSB_R1) && millis() - time > 1000) {
    Serial.println("on");
    motorSaftey = !motorSaftey;
     time = millis();
  }
 
}

void motorSend(){
//switch for saftey mechanism 
switch(motorSaftey){ 
  
case 0: 
  Serial.println("Motors disabled");//if saftey is on, the arduino will not write to the motor controllers
  break;
  
case 1: //runs nomral code if saftey is off
 


  float XPos = map(analogValue[3], 0, 255, -100, 100);  //convert joystick values to a easier to understand value
  float YPos = map(analogValue[2], 0, 255, -100, 100);
  float TPosa = map(analogValue[1], 0, 255, -100, 100);


  float TPos = (TPosa + 100) * (RotateSensitivity + RotateSensitivity) / (100 + 100) -RotateSensitivity; //convert the rotation value to what is needed for the equation
  float atanData = atan2(YPos,XPos) + (.5* PI); //calculate the inverse tangent of x/y (Outputs the rotational value of the joystick in the range of +- 1/2pi)
  atanData = (atanData * -1); //invert calculation

  float MagTemp = sqrt(pow((XPos/100), 2) + pow((YPos/100), 2)); //determine magnitude for speed calculation
  float Mag = (MagTemp - 0) * (1 - 0) / (1.41 - 0) + 0; //convert to the needed range for later calculations

  //calculate the speed for each motor in the range of -1 - 1
  float FL = sin(atanData - (.25 * PI)) * Mag - TPos; 
  float BR = sin(atanData - (.25 * PI)) * Mag + TPos;

  float FR = sin(atanData + (.25 * PI)) * Mag - TPos;
  float BL = sin(atanData + (.25 * PI)) * Mag + TPos;

  //convert the previous calculation to the corresponding value in microseconds
  float MTR1Val = (FL + 1) * (1000 - 2000) / (1 + 1) + 2000;
  float MTR2Val = (BL + 1) * (2000 - 1000) / (1 + 1) + 1000;
  float MTR3Val = (BR + 1) * (2000 - 1000) / (1 + 1) + 1000;
  float MTR4Val = (FR + 1) * (1000 - 2000) / (1 + 1) + 2000;

  //Write all motor speeds to the serial monitor for debugging (Comment this out if not needed)

  Serial.print(FL);
  Serial.print(", ");
  Serial.print(BL);
  Serial.print(", ");
  Serial.print(BR);
  Serial.print(", ");
  Serial.print(FR);
  Serial.print(", ");
  Serial.print(MTR1Val);
  Serial.print(", ");
  Serial.print(MTR2Val);
  Serial.print(", ");
  Serial.print(MTR3Val);
  Serial.print(", ");
  Serial.print(MTR4Val);
  Serial.print(", ");
  Serial.println(ASERVOVal);


  //write the value to the ESC's and Servos
  MTR1.writeMicroseconds(MTR1Val);
  MTR2.writeMicroseconds(MTR2Val);
  MTR3.writeMicroseconds(MTR3Val);
  MTR4.writeMicroseconds(MTR4Val);
  ASERVO.writeMicroseconds(ASERVOVal);
  break;


  } 
    }

void armSend() {
switch(motorSaftey){ 
  

  
case 1: //


  if(ps2x.Button(PSB_PAD_UP)) {      //reads up button on dpad and writes to the arm motor
   AMTR1.writeMicroseconds(1600);
   DPadV[0] = 1;
  }
  else{
   DPadV[0] = 0;
 }


  if(ps2x.Button(PSB_PAD_DOWN)){ //reads down button on dpad and writes to the arm motor
    AMTR1.writeMicroseconds(1500);
    DPadV[1] = 1;
  }
  else{
   DPadV[1] = 0;
 }


  if(DPadV[0] == 0 && DPadV[1] == 0){  //if no buttons are being pressed keep arm in a steady spot
    AMTR1.writeMicroseconds(1540);
  }

  
  
  if(ps2x.Button(PSB_PAD_RIGHT)){  //reads right dpad button and writes to the servo value, opens
    
    if(ASERVOVal <= 2000){
       ASERVOVal = ASERVOVal + 20;
    }
    else{
      ASERVOVal = 2000;
    }

    DPadV[2] = 1;

  }
  else{
    DPadV[2] = 0;
  }

  if(ps2x.Button(PSB_PAD_LEFT)){  //reads left dpad button and writes to the servo value, closes
    DPadV[3] = 1;
    if(ASERVOVal >= 1000){
      ASERVOVal = ASERVOVal - 20;
  }
  else{
    ASERVOVal = 1000;
  } }
  else{
    DPadV[3] = 0;
  }
  break;
  }
}



