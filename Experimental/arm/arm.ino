#include <RC100.h>

#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM9.04

#define STOP_SPEED 100//in dxl units
#define BASE_SPEED 0.2//in meters per second

#define INITIAL_1 1072//dxl units
#define INITIAL_3 3800
#define INITIAL_4 1649
#define INITIAL_5 1200

#define X_MAX 0.9
#define X_MIN 0
#define Y_MAX -0.17
#define Y_MIN -0.3
#define ALPHA_MIN 0.1
#define ALPHA_MAX 0.9254
#define BETA_MIN 0.25
#define BETA_MAX 3.14
#define PHI_MIN 1.6
#define PHI_MAX 4.5

#define LEFT_LIM 1494//dxl units
#define RIGHT_LIM 828

//declare RC100 stuff
RC100 Controller;
int RcvData = 0;

//declare kinematics parameters
double dx_dt;//in meters per second
double dy_dt;
double dalpha_dt;//in 0.229 revolutions per second
double dbeta_dt;
double dphi_dt;
double dtheta_dt;
double alpha;//in radians
double beta;
double phi;
double x;
double y;
double a;//upper arm length in meters
double b;//lower arm length
double c;//hand length
double d;//length of intermediate between upper and lower arm

//declare dynamixel
Dynamixel Dxl(DXL_BUS_SERIAL3);

//variables to store current stop positions
int stopPos1;
int stopPos3;
int stopPos4;
int stopPos5;
boolean isStoppedXY;//forward, backward, upward, and downward motion is stopped
boolean isStoppedZ;//left and right motion is stopped (curved z-axis)

//variables to store current positions
int pos1;
int pos3;
int pos4;
int pos5;

//variable to store if we're in that weird case of far back motion
boolean far_back;

boolean auton;

void setup() {
  auton = false;//start in teleop
  
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(18,OUTPUT);
  pinMode(19,OUTPUT);
  pinMode(20,OUTPUT);
  Controller.begin(1);//begin 1 means bluetooth/zigbee mode
  
  //initialize kinematics parameters
  dx_dt = 0;
  dy_dt = 0;
  dalpha_dt = 0;
  dbeta_dt = 0;
  dphi_dt = 0;
  dtheta_dt = 0;
  a = 0.3302;//upper arm length in meters
  b = 0.235;//lower arm length
  c = 6.5*2.54/100;//hand length
  
  //begin dynamixel communication, set dynamiels to joint mode and wheel mode as needed
  Dxl.begin(3);//3 -> 1 Mbps
  Dxl.jointMode(1);
  Dxl.jointMode(3);
  Dxl.jointMode(4);
  Dxl.jointMode(5);
  Dxl.wheelMode(6);//end effector
  
  initial_positions();
  
  //until user provides input, stop motors at their current positions
  stopPos1 = Dxl.getPosition(1);
  stopPos3 = Dxl.getPosition(3);
  stopPos4 = Dxl.getPosition(4);
  stopPos5 = Dxl.getPosition(5);
  stopXY();
  stopZ();
  
  far_back = false;
}

void loop() {
  //determine current angles and positions
  pos1 = Dxl.getPosition(1);
  pos3 = Dxl.getPosition(3);
  pos4 = Dxl.getPosition(4);
  pos5 = Dxl.getPosition(5);
  alpha = 2*PI*pos3/4095 - 5.212;
  beta = 2*PI*pos4/4095 + 0.2282 - PI/2;
  phi = 3*PI/2 - alpha - beta;
  x = a*cos(alpha) + b*cos(PI - beta - alpha);
  y = a*sin(alpha) - b*sin(PI - beta - alpha) - c;
  
  double prev_dalpha_dt = dalpha_dt;
  double prev_dbeta_dt = dbeta_dt;
  double prev_dphi_dt = dphi_dt;
  
  //if controller data is available, modify speeds based on data
  if(Controller.available()){
      RcvData = Controller.readData();
      //turn end effector on and off based on user input
      if(RcvData & RC100_BTN_5){
        Dxl.goalSpeed(6,500);
      }else if(RcvData & RC100_BTN_6){
        Dxl.goalSpeed(6,0);
      }
      
      //commands for entering and exiting autonomous
      if((RcvData & RC100_BTN_L) && !auton){
        auton = true;
        initial_positions();
        dx_dt = BASE_SPEED;
        dy_dt = 0;
        /*dalpha_dt = 60*(sin(phi - PI/2)*dy_dt - cos(phi - PI/2)*dx_dt)/(2*PI*0.229*a*sin(alpha + phi - PI/2));
        dphi_dt = -60*(sin(alpha)*dy_dt + cos(alpha)*dx_dt)/(2*PI*0.229*b*sin(alpha + phi - PI/2));
        dbeta_dt = -dphi_dt - dalpha_dt;*/
        double denom = a*cos(alpha)*tan(PI - beta - alpha) + a*sin(alpha);
        dalpha_dt = 60*((-dx_dt/denom)/(2*PI))/0.229;
        dbeta_dt = -dalpha_dt*(1 + a*cos(alpha)/(b*cos(PI - beta - alpha)));
        dphi_dt = -dbeta_dt - dalpha_dt;
        isStoppedXY = false;
        isStoppedZ = false;
        dtheta_dt = 1;
      }else if ((RcvData & RC100_BTN_R) && auton){
        auton = false;
        initial_positions();
        stopXY();
        stopZ();
      }
    }
    //right now there's lots of redundancy where we're computing angular velocities in each case. when everything works, make it so that only linear velocities are computed in cases
    if(auton){
      do_auton();
    }else{
      do_teleop();
    }
    
    if(prev_dalpha_dt*dalpha_dt < 0 || prev_dbeta_dt*dbeta_dt < 0 || prev_dphi_dt*dphi_dt < 0){
      stopXY();
      moveXY();
      delay(100);
      isStoppedXY = false;
    }
    
    /*if(Dxl.getSpeed(3) > 1080){
      jerk_detector(3);
    }else if(Dxl.getSpeed(4) > 1100){
      jerk_detector(4);
    }*/
    
    //stop if any limits have been reached
    stop_conditions();
    
    //if motors are not at their limits, apply kinematics parameters to motor movement
    moveZ();
    
    if(far_back){
      moveXYFarBack();
    }else{
      moveXY();
    }
    
    SerialUSB.print(x);
    SerialUSB.print(", ");
    SerialUSB.println(dx_dt);
}

void stop_conditions(){
  //stop XY motion if any limits have been reached
  if((dx_dt == 0 && dy_dt == 0) || (dx_dt > 0 && x > X_MAX-0.01) || (dx_dt < 0 && x < X_MIN + 0.01) || (dy_dt > 0 && y > Y_MAX - 0.01) || (dy_dt < 0 && y < Y_MIN + 0.01)){
    stopXY();
  }else if((dalpha_dt > 0 && alpha > ALPHA_MAX - 0.01) || (dalpha_dt < 0 && alpha < ALPHA_MIN + 0.01) || (dbeta_dt > 0 && beta > BETA_MAX - 0.01) || (dbeta_dt < 0 && beta < BETA_MIN + 0.01) || (dphi_dt > 0 && phi > PHI_MAX - 0.01) || (dphi_dt < 0 && phi < PHI_MIN + 0.01)){
    stopXY();
  }
  
  //stop Z motion if any limits have been reached
  if(dtheta_dt == 0 || (abs(pos1 - LEFT_LIM) < 3 && dtheta_dt > 0) || (abs(pos1 - RIGHT_LIM) < 3 && dtheta_dt < 0)){
    stopZ();
  }
}

void moveZ(){
  if(isStoppedZ){
    Dxl.setPosition(1,stopPos1,STOP_SPEED);
  }else{
    if(dtheta_dt > 0){
      Dxl.setPosition(1,LEFT_LIM,abs(dtheta_dt));
    }else{
      Dxl.setPosition(1,RIGHT_LIM,abs(dtheta_dt));
    }
  }
}
void moveXY(){
  if(isStoppedXY){
    Dxl.setPosition(3,stopPos3,STOP_SPEED);//shoulder
    Dxl.setPosition(4,stopPos4,STOP_SPEED);//elbow
    Dxl.setPosition(5,stopPos5,STOP_SPEED);//wrist
  }else{
    if(dalpha_dt < 0){//positive x or negative y
      Dxl.setPosition(3,0,abs((int)dalpha_dt));
    }else if(dalpha_dt > 0){//negative x or positive y
      if(dx_dt != 0){//x
        Dxl.setPosition(3,INITIAL_3,abs((int)dalpha_dt));
      }else{//y
        Dxl.setPosition(3,4000,abs((int)dalpha_dt));
      }
    }
    
    if(dbeta_dt > 0){//positive x or positive y
      Dxl.setPosition(4,4000,abs((int)dbeta_dt));
    }else if(dbeta_dt < 0){//negative x or negative y
      if(dx_dt != 0){//x
        Dxl.setPosition(4,INITIAL_4,abs((int)dbeta_dt));
      }else{//y
        Dxl.setPosition(4,0,abs((int)dbeta_dt));
      }
    }
    
    if(dphi_dt < 0){//positive x or positive y
      Dxl.setPosition(5,0,abs((int)dphi_dt));
    }else if(dphi_dt > 0){//negative x or negative y
      if(dx_dt != 0){
        Dxl.setPosition(5,INITIAL_5,abs((int)dphi_dt));
      }else{
        Dxl.setPosition(5,4000,abs((int)dphi_dt));
      }
    }
  }
}

void moveXYFarBack(){
  if(isStoppedXY){
    Dxl.setPosition(3,stopPos3,STOP_SPEED);//shoulder
    Dxl.setPosition(4,stopPos4,STOP_SPEED);//elbow
    Dxl.setPosition(5,stopPos5,STOP_SPEED);//wrist
  }else{
    if(dalpha_dt < 0){//negative x or negative y
      Dxl.setPosition(3,0,abs((int)dalpha_dt));
    }else if(dalpha_dt > 0){//positive x or positive y
      if(dx_dt != 0){//x
        Dxl.setPosition(3,INITIAL_3,abs((int)dalpha_dt));
      }else{//y
        Dxl.setPosition(3,4000,abs((int)dalpha_dt));
      }
    }
    
    if(dbeta_dt > 0){//positive x or positive y
      Dxl.setPosition(4,INITIAL_4,abs((int)dbeta_dt));
    }else if(dbeta_dt < 0){//negative x or negative y
      if(dx_dt != 0){//x
        //Dxl.setPosition(4,INITIAL_4,abs((int)dbeta_dt));
        Dxl.setPosition(4,0,abs((int)dbeta_dt));
      }else{//y
        Dxl.setPosition(4,0,abs((int)dbeta_dt));
      }
    }
    
    if(dphi_dt < 0){//positive x or positive y
      Dxl.setPosition(5,INITIAL_5,abs((int)dphi_dt));
    }else if(dphi_dt > 0){//negative x or negative y
      if(dx_dt != 0){
        //Dxl.setPosition(5,INITIAL_5,abs((int)dphi_dt));
        Dxl.setPosition(5,4000,abs((int)dphi_dt));
      }else{
        Dxl.setPosition(5,4000,abs((int)dphi_dt));
      }
    }
  }
}

void write_all_low(){
  digitalWrite(BOARD_LED_PIN,LOW);
  digitalWrite(18,LOW);
  digitalWrite(19,LOW);
  digitalWrite(20,LOW);
}
void write_all_high(){
  digitalWrite(BOARD_LED_PIN,HIGH);
  digitalWrite(18,HIGH);
  digitalWrite(19,HIGH);
  digitalWrite(20,HIGH);
}
void stopXY(){
  if(!isStoppedXY){
    stopPos3 = Dxl.getPosition(3);
    stopPos4 = Dxl.getPosition(4);
    stopPos5 = Dxl.getPosition(5);
    isStoppedXY = true;
  }
}
void stopZ(){
  if(!isStoppedZ){
    isStoppedZ = true;
    stopPos1 = Dxl.getPosition(1);
  }
}

void initial_positions(){
  write_all_low();
  delay(500);
  write_all_high();
  delay(500);
  long time = millis();
  write_all_low();//while setup motion is occurring, all LEDS will be on
  
  //make shoulder go to standard initial position
  Dxl.setPosition(3,INITIAL_3,100);
  while(abs(Dxl.getPosition(3) - INITIAL_3) > 3 && millis()-time < 2000){}
  time = millis();
  //make elbow go to standard initial position
  Dxl.setPosition(4,INITIAL_4,100);
  while(abs(Dxl.getPosition(4) - INITIAL_4) > 3 && millis()-time < 2000){}
  time = millis();
  //make wrist go to standard initial position
  Dxl.setPosition(5,INITIAL_5,100);
  while(abs(Dxl.getPosition(5) - INITIAL_5) > 3 && millis()-time < 2000){}
  time = millis();
  //make rotator go to standard initial position (center for teleop, left limit for auto)
  Dxl.setPosition(1,INITIAL_1,100);
  while(abs(Dxl.getPosition(1) - INITIAL_1) > 3 && millis()-time < 2000){}
  
  Dxl.goalSpeed(6,0);//do not rotate end effector initially
  
  write_all_low();
  delay(500);
  write_all_high();
  delay(500);
}

//converts angle in radians to dxl units for motor 3
int convert3(double angle){
  return (int)((angle + 5.212)*4095/(2*PI));
}
//same for motor 4
int convert4(double angle){
  return (int)((angle - 1.757)*4095/(2*PI));
}
//same for motor 5
int convert5(double angle){
  return (int)((angle - 1.269)*4095/(2*PI));
}

void jerk_detector(int motor){
  SerialUSB.print("Jerk ");
  SerialUSB.println(motor);
  dx_dt = 0;
  dy_dt = 0;
  dalpha_dt = 0;
  dbeta_dt = 0;
  dphi_dt = 0;
  delay(100);
}

void do_teleop(){
  //move end effector forward, backward, up, and down based on user input
  if(RcvData & RC100_BTN_U){//forward
    write_all_low();
    dy_dt = 0;
    dx_dt = BASE_SPEED;
    double denom = a*cos(alpha)*tan(PI - beta - alpha) + a*sin(alpha);
    dalpha_dt = 60*((-dx_dt/denom)/(2*PI))/0.229;
    dbeta_dt = -dalpha_dt*(1 + a*cos(alpha)/(b*cos(PI - beta - alpha)));
    isStoppedXY = false;
    
    if(pos4 <= INITIAL_4 - 20 && pos5 >= INITIAL_5 + 20){
      far_back = true;
    }else{
      far_back = false;
    }
  }else if(RcvData & RC100_BTN_D){//backward
    write_all_low();
    if(isStoppedXY && y < Y_MIN + 0.03){
      //
    }else{
      dy_dt = 0;
      dx_dt = -BASE_SPEED;
      double denom = a*cos(alpha)*tan(PI - beta - alpha) + a*sin(alpha);
      dalpha_dt = 60*((-dx_dt/denom)/(2*PI))/0.229;
      dbeta_dt = -dalpha_dt*(1 + a*cos(alpha)/(b*cos(PI - beta - alpha)));
      isStoppedXY = false;
    }
    
    if(pos4 <= INITIAL_4 + 20 && pos5 >= INITIAL_5 - 20){
      far_back = true;
    }else{
      far_back = false;
    }
  }else if(RcvData & RC100_BTN_1){//up
    write_all_low();
    if(isStoppedXY && y > Y_MAX - 0.03){
      //counter against gravity
    }else{
      dx_dt = 0;
      dy_dt = BASE_SPEED;
      double denom = a*sin(alpha)/tan(PI - beta - alpha) + a*cos(alpha);
      dalpha_dt = 60*((dy_dt/denom)/(2*PI))/0.229;
      dbeta_dt = -dalpha_dt*(1 - a*sin(alpha)/(b*sin(PI - beta - alpha)));
      isStoppedXY = false;
    }
  }else if(RcvData & RC100_BTN_3){//down
    write_all_low();
    if(isStoppedXY && y < Y_MIN + 0.03){
      //
    }else{
      dx_dt = 0;
      dy_dt = -BASE_SPEED;
      double denom = a*sin(alpha)/tan(PI - beta - alpha) + a*cos(alpha);
      dalpha_dt = 60*((dy_dt/denom)/(2*PI))/0.229;
      dbeta_dt = -dalpha_dt*(1 - a*sin(alpha)/(b*sin(PI - beta - alpha)));
      isStoppedXY = false;
    }
  }else{
    write_all_high();
    dx_dt = 0;
    dy_dt = 0;
    dalpha_dt = 0;
    dbeta_dt = 0;
    stopXY();
  }
  dphi_dt = -dbeta_dt-dalpha_dt;

  //set kinematics parameters for left/right motion (rotation)
  if(RcvData & RC100_BTN_2){//counterclockwise is left
    isStoppedZ = false;
    dtheta_dt = 25;
  }else if(RcvData & RC100_BTN_4){//right
    isStoppedZ = false;
    dtheta_dt = -25;
  }else{//stop
    dtheta_dt = 0;
    stopZ();
  }
}

void do_auton(){
  SerialUSB.println("auto");
  //if anything approaches a limit, switch direction
  if((pos1 < RIGHT_LIM + 50 && dtheta_dt < 0) || (pos1 > LEFT_LIM - 50 && dtheta_dt > 0)){
    dtheta_dt = -dtheta_dt;
    isStoppedXY = false;
    isStoppedZ = false;
  }
  if((dx_dt > 0 && x > 0.45) || (dx_dt < 0 && x < 0.05)){
    dx_dt = -dx_dt;
    isStoppedXY = false;
    isStoppedZ = false;
  }
  
  //if the spot ahead is at a different elevation, move to that elevation
  /*int sensorVal = analogRead(0);
  if(sensorVal > 2175){
    dy_dt = BASE_SPEED;
  }else if(sensorVal < 2100){
    dy_dt = -BASE_SPEED;
  }*/
  /*dalpha_dt = 60*(sin(phi - PI/2)*dy_dt - cos(phi - PI/2)*dx_dt)/(2*PI*0.229*a*sin(alpha + phi - PI/2));
  dphi_dt = -60*(sin(alpha)*dy_dt + cos(alpha)*dx_dt)/(2*PI*0.229*b*sin(alpha + phi - PI/2));
  dbeta_dt = -dphi_dt - dalpha_dt;*/
  double denom = a*cos(alpha)*tan(PI - beta - alpha) + a*sin(alpha);
  dalpha_dt = 60*((-dx_dt/denom)/(2*PI))/0.229;
  dbeta_dt = -dalpha_dt*(1 + a*cos(alpha)/(b*cos(PI - beta - alpha)));
  dphi_dt = -dbeta_dt-dalpha_dt;
}
