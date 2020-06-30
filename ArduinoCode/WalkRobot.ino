/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// RECIVER
//#include <SPI.h>
  #include <nRF24L01.h>
  #include <RF24.h>
  #include <math.h>
  
  RF24 radio(3, 2);   // nRF24L01 (CE, CSN)
  const byte address[6] = "00001";
  unsigned long lastReceiveTime = 0;
  unsigned long currentTime = 0;
  // Max size of this struct is 32 bytes
  struct Data_Package {
    byte j1PotX;
    byte j1PotY;
    byte j1Button;
    byte j2PotX;
    byte j2PotY;
    byte j2Button;
    byte pot1;
    byte pot2;
    byte tSwitch1;
    byte tSwitch2;
    byte button1;
    byte button2;
    byte button3;
    byte button4;
  };
  Data_Package data; //Create a variable with the above structure
// END RECIVER


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
uint8_t servonum = 0;
int degree[16];// = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

float a = 0;
float Z = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  yield();

  // RECIVER
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
  resetData();

}


void loop() {
  // RECIVER
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }

  // Print Result
  
  Serial.print("Joystick 1X: ");
  Serial.print(data.j1PotX);
  Serial.print(" Joystick 1Y: ");
  Serial.println(data.j1PotY);
  /*
  Serial.print(" Joystick 2X: ");
  Serial.print(data.j2PotX);
  Serial.print(" Joystick 2Y: ");
  Serial.print(data.j2PotY);

  Serial.print(" Button 1: ");
  Serial.print(data.button1);
  Serial.print(" Button 2: ");
  Serial.print(data.button2);
  Serial.print(" Button 3: ");
  Serial.print(data.button3);
  Serial.print(" Button 4: ");
  Serial.print(data.button4);

  Serial.print(" Pot 1: ");
  Serial.print(data.pot1);
  Serial.print(" Pot 2: ");
  Serial.println(data.pot2);
  
  */


  // HC-05 Blutooth
  if(Serial.available()>0){
  int i;
  char data = Serial.read();
    switch(data){
      case 'a':
            for (i=0; i<11;i++){
              Z++;
              }
              break;
  
  
      case 'c':
      for (i=0; i<11;i++){
              Z--;
              }
                break;
  
      
    }
  }



  
  //Serial.println(data.j1PotX);
  // Z = map(data.j1PotX,0,255, 0, 90);
  Z = 50;


  // MAIN
  calcLeg();

  // command Stand
  // Vorne Rechts
  degree[0] = a;
  degree[1] = -a;
  degree[2] = 0;
  degree[3] = 0;
  
  // Hinten Rechts
  degree[4] = -a;
  degree[5] = a;
  degree[6] = 0;
  degree[7] = 0;
 
  // Hinten Links
  degree[8] = a;
  degree[9] = -a;
  degree[10] = 0;
  degree[11] = 0;

  // Vorne Links
  degree[12] = -a;
  degree[13] = a;
  degree[14] = 0;
  degree[15] = 0;
          

  
  servo();


  delay(5);
}



void calcLeg(){
  
  int L0 = 45;
  int L1 = 45;
  int S1 = 23;
  int H1 = 23;
  a = acos((Z/2)/L0)*(180/3.14);
 



 /*
  
  int a0 = degree[0];
  int a1 = degree[1];
  int a2 = degree[2];
  int a3 = degree[3];

  //LEG
  int ZLeg;
  int YLeg;
  int XLeg;

  ZLeg = L0*cos(a0)+L1*cos(a1);
  XLeg = 9;
  YLeg = L0*sin(a0)+L1*sin(a1);

  // Shoulder
  int ZShoulder;
  int YShoulder;
  int XShoulder;

  ZShoulder = ZLeg+20;
  YShoulder = XLeg*cos(a2)-YLeg*sin(a2)+S1*sin(a2);
  XShoulder = YLeg*sin(a2)+XLeg*cos(a2)+S1*cos(a2);


  // HIP
  int ZHip;
  int YHip;
  int XHip;

  ZHip = H1*sin(a3)-ZShoulder*sin(a3);
  XHip = -(H1*cos(a3))-XShoulder*cos(a3);
  YHip = YShoulder;

  // Origin
  int Z;
  int X;
  int Y;
  Z = ZHip+10;
  X = XHip-11;
  Y = YHip-48;
  */
  
  }



 
void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}


 void servo() {
  // put your main code here, to run repeatedly://100

  // Vorne Rechts
  int pulse0 = map(degree[0], -90,90, 510, 100);
  int pulse1 = map(degree[1], -90,90, 510, 100); 
  int pulse2 = map(degree[2], -90,90, 100, 510);
  int pulse3 = map(degree[3], -90,90, 100, 510);
   
  pwm.setPWM(0,0,pulse0);
  pwm.setPWM(1,0,pulse1);
  pwm.setPWM(2,0,pulse2);
  pwm.setPWM(3,0,pulse3);
  
  // Hinten Rechts
  int pulse4 = map(degree[4], -90,90, 100, 510);
  int pulse5 = map(degree[5], -90,90, 100, 510); 
  int pulse6 = map(degree[6], -90,90, 510, 100);
  int pulse7 = map(degree[7], -90,90, 510, 100);

  pwm.setPWM(4,0,pulse4);
  pwm.setPWM(5,0,pulse5);
  pwm.setPWM(6,0,pulse6);
  pwm.setPWM(7,0,pulse7);
  
 // Hinten Links
  int pulse8 = map(degree[8], -90,90, 100, 510);
  int pulse9 = map(degree[9], -90,90, 100, 510); 
  int pulse10 = map(degree[10], -90,90, 510, 100);
  int pulse11 = map(degree[11], -90,90, 510, 100);
   
  pwm.setPWM(8,0,pulse8);
  pwm.setPWM(9,0,pulse9);
  pwm.setPWM(10,0,pulse10);
  pwm.setPWM(11,0,pulse11);
  
  // Vorne Links
  int pulse12 = map(degree[12], -90,90, 510, 100);
  int pulse13 = map(degree[13], -90,90, 510, 100); 
  int pulse14 = map(degree[14], -90,90, 100, 510);
  int pulse15 = map(degree[15], -90,90, 50, 510);
  
  pwm.setPWM(12,0,pulse12);
  pwm.setPWM(13,0,pulse13);
  pwm.setPWM(14,0,pulse14);
  pwm.setPWM(15,0,pulse15);
}
