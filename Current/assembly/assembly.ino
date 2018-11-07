#include <RC100.h>

#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM9.04

//declare dynamixel
Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup() {
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(18,OUTPUT);
  pinMode(19,OUTPUT);
  pinMode(20,OUTPUT);
  
  //begin dynamixel communication, set dynamiels to joint mode and wheel mode as needed
  Dxl.begin(3);//3 -> 1 Mbps
  
  for(int i = 0; i < 2; i++){//indicate that setup motion will begin
    write_all_low();
    delay(500);
    write_all_high();
    delay(500);
  }
  write_all_low();//while setup motion is occurring, all LEDS will be on
  
  Dxl.jointMode(1);
  while(Dxl.getPosition(1) > 4095){}
  long time = millis();
  //make rotator go to standard initial position (center) when the on-board button is pressed
  while(digitalRead(BOARD_BUTTON_PIN) == LOW){}
  Dxl.setPosition(1,1606,100);
  while(abs(Dxl.getPosition(1) - 1606) > 3 && millis() - time < 3000){}
  SerialUSB.println("Rotator has been positioned. Please attach arm base pointing forward if you haven't already.");
  
  Dxl.jointMode(3);
  if(Dxl.getPosition(3) > 4095){
    SerialUSB.println("Before plugging in the next motor, turn off the board, close Serial Monitor, and disconnect the USB from the computer. After making the connection, turn on the board, connect it to the computer, and open Serial Monitor once more.");
  }
  while(Dxl.getPosition(3) > 4095){}
  time = millis();
  //make shoulder go to standard initial position when the on-board button is pressed
  while(digitalRead(BOARD_BUTTON_PIN) == LOW){}
  Dxl.setPosition(3,4000,100);
  while(abs(Dxl.getPosition(3) - 4000) > 3 && millis() - time < 3000){}
  SerialUSB.println("Shoulder has been positioned. Please attach upper arm pointing forward and upward if you haven't already.");
  
  Dxl.jointMode(4);
  if(Dxl.getPosition(4) > 4095){
    SerialUSB.println("Before plugging in the next motor, turn off the board, close Serial Monitor, and disconnect the USB from the computer. After making the connection, turn on the board, connect it to the computer, and open Serial Monitor once more.");
  }
  while(Dxl.getPosition(4) > 4095){}
  time = millis();
  //make elbow go to standard initial position when the on-board button is pressed
  while(digitalRead(BOARD_BUTTON_PIN) == LOW){}
  Dxl.setPosition(4,1749,100);
  while(abs(Dxl.getPosition(4) - 1749) > 3 && millis() - time < 3000){}
  SerialUSB.println("Elbow has been positioned. Please attach lower arm pointing forward and downward if you haven't already.");
  
  Dxl.jointMode(5);
  if(Dxl.getPosition(5) > 4095){
    SerialUSB.println("Before plugging in the next motor, turn off the board, close Serial Monitor, and disconnect the USB from the computer. After making the connection, turn on the board, connect it to the computer, and open Serial Monitor once more.");
  }
  while(Dxl.getPosition(5) > 4095){}
  time = millis();
  //make wrist go to standard initial position when the on-board button is pressed
  while(digitalRead(BOARD_BUTTON_PIN) == LOW){}
  Dxl.setPosition(5,900,100);
  while(abs(Dxl.getPosition(5) - 900) > 3 && millis() - time < 3000){}
  SerialUSB.println("Wrist has been positioned. Please attach end effector pointing directly downward if you haven't already. Then download the arm2.ino software to the board.");
  
  Dxl.jointMode(6);
  if(Dxl.getPosition(6) > 4095){
    SerialUSB.println("Before plugging in the next motor, turn off the board, close Serial Monitor, and disconnect the USB from the computer. After making the connection, turn on the board, connect it to the computer, and open Serial Monitor once more.");
  }
  
  for(int i = 0; i < 2; i++){//indicate that setup motion has concluded
    write_all_low();
    delay(500);
    write_all_high();
    delay(500);
  }
}

void loop() {
  write_all_low();
  delay(500);
  write_all_high();
  delay(500);
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
