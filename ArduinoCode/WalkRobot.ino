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
 

/****************************************************
 * DECLARATION RECIVIER 
 ****************************************************/
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

/****************************************************
 * DECLARATION PWM SERVODRIVER
 ****************************************************/
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

/****************************************************
 * DECLARATION INTERNAL GLOBAL VAR
 ****************************************************/
int CalibAngle = 100;
int Calibnumber = 0;
int Calibration[16] = {310,270,320,300, 350,350,310,310, 290,300,300,290, 310,280,310,290};

struct Orientation {
    float Xaxis;
    float Yaxis;
    float Zaxis;
};
Orientation RobotPosition; //Create a variable with the above structure

class  Legpos {
    // Access specifier 
    public: 
    // Data Members 
    float HipAngle;
    float ShoulderAngle;
    float UpperlegAngle;
    float LowerlegAngle;
};

Legpos LeftFrontleg;  //Create a variable with the above structure
Legpos LeftBackleg;   //Create a variable with the above structure
Legpos RightFrontleg; //Create a variable with the above structure
Legpos RightBackleg;  //Create a variable with the above structure

/****************************************************
 * CODE SETUP
 ****************************************************/
void setup() {
  Serial.begin(9600);
  Serial.println("Start setup code");

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
  
  Serial.println("Start execution code");
}

/****************************************************
 * CODE MAIN LOOP
 ****************************************************/
void loop() {
  // --- MANUAL CALIBRATION
  //ManualCalibration();
  
  // --- RC COMMAND ---
  rcReciver(0);
  //  RobotPosition.Zaxis = map(data.j1PotX,0,255, 0, 90);

  // --- BLUETOOTH COMMAND ---
  //RobotPosition = bluetooth(RobotPosition);
  bluetooth(RobotPosition);
  // --- MAIN ---
  CalcLeg(RobotPosition);
  ServoControl();

  

  delay(5);
}




/****************************************************
 * CODE DATA BLUETOOTH HC-05
 ****************************************************/
// Acces adress of input and manipulate it
void bluetooth(Orientation &Value){
  // --- HC-05 Blutooth ---

  //Orientation &Value
  char data = Serial.read();

  //if(Serial.available()>0){
    int i;
      switch(data){
        case 'a':
                Value.Zaxis += 10;
                break;
    
    
        case 'b':
                Value.Zaxis -= 10;
                break;

        case 'c':
                Value.Xaxis += 10;
                break;
    
        case 'd':
                Value.Xaxis -= 10;
                break;

        case 'e':
                Value.Yaxis += 10;
                break;

        case 'f':
                Value.Yaxis -= 10;
                break;
      }
  //}

  // Xaxis,Yaxis,Zaxis
  /*
  Serial.print("ZAxis Command : ");
  Serial.print(RobotPosition.Zaxis);
  Serial.print(" YAxis Command : ");
  Serial.print(RobotPosition.Yaxis);
  Serial.print(" XAxis Command : ");
  Serial.println(RobotPosition.Xaxis);
 */
  //return {Value} ;
}



/****************************************************
 * CODE RESET DATA RC-TRANSMITTER
 ****************************************************/
void rcReciver(bool printResult){
    // --- RECIVER ---
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
  
    // --- Print Result ---
    if (printResult == 1){
      Serial.print("Joystick 1X: ");
      Serial.print(data.j1PotX);
      Serial.print(" Joystick 1Y: ");
      Serial.println(data.j1PotY);
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
    }
 }

/****************************************************
 * CODE RESET DATA RC-TRANSMITTER
 ****************************************************/
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

/****************************************************
 * CODE CALCULATION MOTOR POSITIONS
 ****************************************************/
void CalcLeg(Orientation Value){
  float AngleUp;
  float AngleLo;
  float AngleUp2;
  float Zaxis2;
  float AngleHip;
  float LegnthM;
  float Zaxis3;

  float AngleUp_1;
  float AngleLo_Deg;
  float AngleUp_Deg;
  float AngleHip_Deg;
  
  // --- INVERSE KINEMATIC
  float L0 = 45;  // Lowerleg length
  float L1 = 45;  // Upperleg length
  float S1 = 30;  // Shoulder length
  float H1 = 23;  // Hip length


  // Leg in Y
  LegnthM = sqrt(sq(Value.Zaxis)+sq(Value.Yaxis));
  Zaxis3 = sqrt(sq(S1)+sq(LegnthM));
  AngleHip = atan(Value.Yaxis/Value.Zaxis);
  
  // Leg in X
  AngleUp2 = atan((Value.Xaxis/Zaxis3));
  Zaxis2 = Zaxis3 / cos(AngleUp2);
  
  // Leg in Z in DEG
  AngleUp_1 = acos(((-0.5)*sq(L1)+0.5*sq(L0)+0.5*sq(Zaxis2))/(L0*Zaxis2)); // Same result = for unefen length L0, L1

  //AngleUp_1 = acos((Zaxis2/2)/L0); // Alternativ result

  AngleUp = AngleUp_1+AngleUp2;
  AngleLo = (2*AngleUp_1);


  // DEGREES
  AngleLo_Deg = AngleLo*(180/PI);
  AngleUp_Deg = AngleUp*(180/PI);
  AngleHip_Deg = AngleHip*(180/PI);

  
  ::RightFrontleg.HipAngle = AngleHip_Deg;
  ::RightFrontleg.ShoulderAngle = 0;
  ::RightFrontleg.UpperlegAngle = AngleUp_Deg;
  ::RightFrontleg.LowerlegAngle = (-1)*AngleLo_Deg;

  
  Serial.print(" ZAxis Command : ");
  Serial.print(RobotPosition.Zaxis);
  Serial.print(" YAxis Command : ");
  Serial.print(RobotPosition.Yaxis);
  Serial.print(" XAxis Command : ");
  Serial.println(RobotPosition.Xaxis);
 

  ::RightBackleg.HipAngle = AngleHip_Deg;
  ::RightBackleg.ShoulderAngle = 0;
  ::RightBackleg.UpperlegAngle = AngleUp_Deg;
  ::RightBackleg.LowerlegAngle = (-1)*AngleLo_Deg;

  
  ::LeftFrontleg.HipAngle =  (-1)*AngleHip_Deg;
  ::LeftFrontleg.ShoulderAngle = 0;
  ::LeftFrontleg.UpperlegAngle = AngleUp_Deg;
  ::LeftFrontleg.LowerlegAngle = (-1)*AngleLo_Deg;

  ::LeftBackleg.HipAngle =  (-1)*AngleHip_Deg;
  ::LeftBackleg.ShoulderAngle = 0;
  ::LeftBackleg.UpperlegAngle = AngleUp_Deg;
  ::LeftBackleg.LowerlegAngle = (-1)*AngleLo_Deg;

/*

   ::LeftFrontleg.HipAngle = 15;
  ::LeftFrontleg.ShoulderAngle = 15;
  ::LeftFrontleg.UpperlegAngle = 15;
  ::LeftFrontleg.LowerlegAngle = 15;

  ::LeftBackleg.HipAngle = 15;
  ::LeftBackleg.ShoulderAngle = 15;
  ::LeftBackleg.UpperlegAngle = 15;
  ::LeftBackleg.LowerlegAngle = 15;

  ::RightFrontleg.HipAngle = 0;
  ::RightFrontleg.ShoulderAngle = 15;
  ::RightFrontleg.UpperlegAngle = 15;
  ::RightFrontleg.LowerlegAngle = 15;

  ::RightBackleg.HipAngle = 15;
  ::RightBackleg.ShoulderAngle = 15;
  ::RightBackleg.UpperlegAngle = 15;
  ::RightBackleg.LowerlegAngle = 15;
*/

 /*
  --- FORWARD KINEMATIC ---
  
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

/****************************************************
 * CODE DRIVE SERVOS
 * 
 * Maps command in degrees to calibrated motors as an
 * pwm output
 ****************************************************/
 void ServoControl() {
  // --- PCA9685 ---
  int Limit = 200;
  
  // FRONT RIGHT LEG
  int pulse0 = map(RightFrontleg.LowerlegAngle, -90,90, Calibration[0]-Limit,Calibration[0]+Limit);  // Lower leg
  int pulse1 = map(RightFrontleg.UpperlegAngle, -90,90, Calibration[1]-Limit,Calibration[1]+Limit);  // Upper leg
  int pulse2 = map(RightFrontleg.ShoulderAngle, -90,90, Calibration[2]+Limit,Calibration[2]-Limit);  // Soulder
  int pulse3 = map(RightFrontleg.HipAngle, -90,90, Calibration[3]+Limit,Calibration[3]-Limit);  // Hip
   
  pwm.setPWM(0,0,pulse0);
  pwm.setPWM(1,0,pulse1);
  pwm.setPWM(2,0,pulse2);
  pwm.setPWM(3,0,pulse3);
  
  // BACK RIGHT LEG
  int pulse4 = map(RightBackleg.LowerlegAngle, -90,90, Calibration[4]-Limit,Calibration[4]+Limit);  // Lower leg
  int pulse5 = map(RightBackleg.UpperlegAngle, -90,90, Calibration[5]-Limit,Calibration[5]+Limit);  // Upper leg
  int pulse6 = map(RightBackleg.ShoulderAngle, -90,90, Calibration[6]-Limit,Calibration[6]+Limit);  // Soulder
  int pulse7 = map(RightBackleg.HipAngle, -90,90, Calibration[7]-Limit,Calibration[7]+Limit);  // Hip

  pwm.setPWM(4,0,pulse4);
  pwm.setPWM(5,0,pulse5);
  pwm.setPWM(6,0,pulse6);
  pwm.setPWM(7,0,pulse7);
  
 // BACK LEFT LEG
  int pulse8 = map(LeftBackleg.LowerlegAngle, -90,90, Calibration[8]+Limit,Calibration[8]-Limit);    // Lower leg
  int pulse9 = map(LeftBackleg.UpperlegAngle, -90,90, Calibration[9]+Limit,Calibration[9]-Limit);    // Upper leg 
  int pulse10 = map(LeftBackleg.ShoulderAngle, -90,90, Calibration[10]-Limit,Calibration[10]+Limit);   // Soulder
  int pulse11 = map(LeftBackleg.HipAngle, -90,90, Calibration[11]+Limit,Calibration[11]-Limit);  // Hip
   
  pwm.setPWM(8,0,pulse8);
  pwm.setPWM(9,0,pulse9);
  pwm.setPWM(10,0,pulse10);
  pwm.setPWM(11,0,pulse11);
  
  // FRONT LEFT LEG
  int pulse12 = map(LeftFrontleg.LowerlegAngle, -90,90, Calibration[12]+Limit,Calibration[12]-Limit);  // Lower leg
  int pulse13 = map(LeftFrontleg.UpperlegAngle, -90,90, Calibration[13]+Limit,Calibration[13]-Limit);  // Upper leg
  int pulse14 = map(LeftFrontleg.ShoulderAngle, -90,90, Calibration[14]-Limit,Calibration[14]+Limit);  // Soulder
  int pulse15 = map(LeftFrontleg.HipAngle, -90,90, Calibration[15]-Limit,Calibration[15]+Limit);   // Hip
  
  pwm.setPWM(12,0,pulse12);
  pwm.setPWM(13,0,pulse13);
  pwm.setPWM(14,0,pulse14);
  pwm.setPWM(15,0,pulse15);


 // TESTS
 uint8_t test1;
  test1 = pwm.getPWM(15);

  //pwm.wakeup();
  //pwm.sleep();
}


/****************************************************
 * CODE DATA BLUETOOTH HC-05
 ****************************************************/
// Acces adress of input and manipulate it
void ManualCalibration(){
  // --- HC-05 Blutooth ---
  char data = Serial.read();
      switch(data){
        case 'a':
                CalibAngle += 10;
                break;
        case 'c':
                CalibAngle -= 10;
                break;
        case 'b':
                Calibnumber += 1;
                break;
        case 'd':
                Calibnumber -= 1;
                break;
        }
        Serial.print("  Selected Servo ");
        Serial.print(Calibnumber);
        Serial.print("  Active PWM  ");
        Serial.println(CalibAngle);
        delay(50);
        pwm.setPWM(Calibnumber,0,CalibAngle);
}
