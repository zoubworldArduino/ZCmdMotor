//EXAMPLE OF MOTOR WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : 
//        sudo chmod 666 /dev/ttyS4 if COM4
//        sudo chmod 666 /dev/ttyS24 if COM24
//        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
//        rosrun rosserial_python serial_node.py /dev/ttyS24 & 

//        rostopic list
//        rostopic echo /1/counter 

//        rostopic echo -p /2/counter


// NOTE : WINDOWS 10 with ubuntu 16.04 as subsystem, after ros install do this :
//      sudo apt-get install ros-kinetic-rosserial-windows
//     sudo apt-get install ros-kinetic-rosserial-server

// validated on Captor


#include <Wire.h>
#include <SPI.h>
#include <variant.h>
#include <bootloaders/boot.h>

#if defined(BOARD_ID_Pilo)
#include <Wire.h>
#include <SPI.h>
#include <variant.h>

#include <WireUtility.h>


#define ROS_SERIAL (P_COM3.serial2)
#define ROS_BAUDRATE 57600
#include "ros.h"
ros::NodeHandle  nh;

#elif defined(BOARD_ID_Captor)
#include <Wire.h>
#include <SPI.h>
#include <variant.h>

#include <WireUtility.h>


#define ROS_SERIAL (P_COM0.serial2)
#define ROS_BAUDRATE 57600

#include "ros.h"
ros::NodeHandle  nh;

#else
//#include <Servo.h> 
#include "ros.h"
ros::NodeHandle  nh;
#endif

#include <ZCmdMotor.h>
#include <Zmotor3.h>
#define MySerial P_COM0.serial2
#define PcomSerial MySerial
/*

https://www.youtube.com/watch?v=lujgeInpejY
http://wiki.ros.org/rosjava
https://github.com/rosjava/rosjava_core
http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html

http://www.gurucoding.com/en/raspberry_pi_eclipse/raspberry_pi_cross_compilation_in_eclipse.php
http://www.raspberry-projects.com/pi/programming-in-c/compilers-and-ides/eclipse/create-new-eclipse-project-for-the-rpi

https://playground.arduino.cc/Code/Eclipse

https://www.materiel.net/pc-portable/acer-aspire-a315-21-97ja-144195.html

http://eclipse.baeyens.it/
*/


#define M1_CA P_Encoder[8-1].Pin.IA  
#define M1_CB P_Encoder[8-1].Pin.IB   
#define M1_MP motorBoard.getPin(MOTOR3_P11_PWM)
#define M1_MM motorBoard.getPin(MOTOR3_P11_IO)
#define M1_EN motorBoard.getPin(MOTOR3_P11_EN) 


/*
#define M2_CA  P_Encoder[2-1].Pin.IA  
#define M2_CB  P_Encoder[2-1].Pin.IB  
#define M2_MP  -1 
#define M2_MM  -1 

*/
#define M2_CA  P_Encoder[7-1].Pin.IA  
#define M2_CB  P_Encoder[7-1].Pin.IB  
#define M2_MP motorBoard.getPin(MOTOR3_P7_PWM)  
#define M2_MM  motorBoard.getPin(MOTOR3_P7_IO) 
#define M2_EN  motorBoard.getPin(MOTOR3_P7_EN)  

#define M3_CA  P_Encoder[6-1].Pin.IA  
#define M3_CB  P_Encoder[6-1].Pin.IB  
#define M3_MP motorBoard.getPin(MOTOR3_P12_PWM)  
#define M3_MM  motorBoard.getPin(MOTOR3_P12_IO) 
#define M3_EN  motorBoard.getPin(MOTOR3_P12_EN)  
#define  MyWireMotor WireB 






// called this way, it uses the default address 0x40
Zmotor3 motorBoard = Zmotor3();

//ZEncoder enc(A0,A2,FULL, NULL);
CMDMOTOR cmd1(M1_CA, M1_CB, M1_MP, M1_MM);
CMDMOTOR cmd2(M2_CA, M2_CB, M2_MP, M2_MM);
CMDMOTOR cmd3(M3_CA, M3_CB, M3_MP, M3_MM);

int count=0;
void privateIntHandler1() {
  cmd1.getEncoder()->update();
  count++;
}
void privateIntHandler2() {
  cmd2.getEncoder()->update();
}
void privateIntHandler3() {
  cmd3.getEncoder()->update();
}

// the setup function runs once when you press reset or power the board
void setupCMD() {
  MySerial.print("setup CMD \r\n");
  delay(500);
   cmd1.setPin(M1_CA, M1_CB, M1_MP, M1_MM);
 cmd2.setPin(M2_CA, M2_CB, M2_MP, M2_MM);
 cmd3.setPin(M3_CA, M3_CB, M3_MP, M3_MM);
  cmd1.setup();
  cmd2.setup();
  cmd3.setup();
  
  MySerial.print("setup CMD end \r\n");
  delay(500);
  cmd1.getEncoder()->attachEncoderInt(privateIntHandler1);
  cmd2.getEncoder()->attachEncoderInt(privateIntHandler2);
cmd3.getEncoder()->attachEncoderInt(privateIntHandler3);

cmd1.setup( &nh,	"/1/pwm","/1/speed");
cmd2.setup(&nh,	"/2/pwm","/2/speed");
cmd3.setup(&nh,	"/3/pwm","/3/speed");
 cmd1.getEncoder()->setup( &nh,	"/1/counter");
 cmd2.getEncoder()->setup( &nh,	"/2/counter");
 cmd3.getEncoder()->setup( &nh,	"/3/counter");
 
}


void setupMotorBoard() {
	MyWireMotor.begin();
	MyWireMotor.setClock(10000);
if (0)
{
MySerial.begin(57600);  //115200 //9600
	MySerial.println("Setup");

	volatile int ip = scan(MySerial, MyWireMotor);
	while (ip = scanNext(MySerial, MyWireMotor) != 0);
}
	motorBoard.begin(&MyWireMotor, 0x22, 0x42);
	motorBoard.analogWriteResolution(12);
	setPinExtender(&motorBoard); // connect the board to arduino API.
	// define pin as output

	//put defaul state : enabled low/low
	pinMode(motorBoard.getPin(MOTOR3_EN_0), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_EN_1), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_EN_2), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_EN_3), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_EN_4), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_EN_5), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_EN_6), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_EN_7), OUTPUT);

	digitalWrite(motorBoard.getPin(MOTOR3_EN_0), LOW);
	digitalWrite(motorBoard.getPin(MOTOR3_EN_1), LOW);
	digitalWrite(motorBoard.getPin(MOTOR3_EN_2), LOW);
	digitalWrite(motorBoard.getPin(MOTOR3_EN_3), LOW);
	digitalWrite(motorBoard.getPin(MOTOR3_EN_4), LOW);
	digitalWrite(motorBoard.getPin(MOTOR3_EN_5), LOW);
	digitalWrite(motorBoard.getPin(MOTOR3_EN_6), LOW);
	digitalWrite(motorBoard.getPin(MOTOR3_EN_7), LOW);



	pinMode(motorBoard.getPin(MOTOR3_IO_0), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_IO_1), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_IO_2), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_IO_3), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_IO_4), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_IO_5), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_IO_6), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_IO_7), OUTPUT);

	pinMode(motorBoard.getPin(MOTOR3_PWM_0), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_PWM_1), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_PWM_2), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_PWM_3), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_PWM_4), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_PWM_5), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_PWM_6), OUTPUT);
	pinMode(motorBoard.getPin(MOTOR3_PWM_7), OUTPUT);
//===========TEST MOTOR ==========================
        digitalWrite(M1_EN, HIGH);
	digitalWrite(M2_EN, HIGH);
	digitalWrite(M3_EN, HIGH);
        
	digitalWrite((M1_MM), HIGH);
	digitalWrite((M1_MP), HIGH);

	digitalWrite((M2_MM), HIGH);
	digitalWrite((M2_MP), HIGH);

	digitalWrite((M3_MM), HIGH);
	digitalWrite((M3_MP), HIGH);

	

}
void loopDEBUGMOTOR();
void setup()
{

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)


setupMotorBoard();

loopDEBUGMOTOR();

nh.initNode(); 

setupCMD();

   //wait until you are actually connected
    while (!nh.connected())
    {
      nh.spinOnce();
      delay(10);
    }
     cmd1.stop();
	cmd2.stop();
	cmd3.stop();
        /*
         delay(1000);
    cmd1.setPWMValue(1000);
	cmd2.setPWMValue(1000);
	cmd3.setPWMValue(1000);
     delay(1000);
    
    	cmd1.setPoint(2000);
	cmd2.setPoint(2000);
	cmd3.setPoint(2000);
        int i=0;
 	while(i<1000)
	{
        i++;
        cmd1.loop();
        cmd2.loop();
        cmd3.loop();

delay(10);
}

cmd1.setPoint(-2000);
	cmd2.setPoint(-2000);
	cmd3.setPoint(-2000);
 i=0;
 	while(i<100)
	{
        i++;
        cmd1.loop();
        cmd2.loop();
        cmd3.loop();

delay(10);
}*/
    	cmd1.stop();
	cmd2.stop();
	cmd3.stop();


}

void loop()
{
 cmd1.loop();
  cmd2.loop();
  cmd3.loop();
  
   nh.loginfo("loop()");
   nh.spinOnce();
    delay(200);
}




  
void loopDEBUGMOTOR() {
  // Drive each pin in a 'wave'
 
#define motor motorBoard  
/* this don't work :
      for (int i=0;i<16;i++)
  digitalWrite(motor.getPin(MOTOR3_EN_0+i), HIGH);
  put this :
  */
     pinMode(motor.getPin(MOTOR3_EN_0), OUTPUT);
     pinMode(motor.getPin(MOTOR3_EN_1), OUTPUT);
     pinMode(motor.getPin(MOTOR3_EN_2), OUTPUT);
     pinMode(motor.getPin(MOTOR3_EN_3), OUTPUT);
     pinMode(motor.getPin(MOTOR3_EN_4), OUTPUT);
     pinMode(motor.getPin(MOTOR3_EN_5), OUTPUT);
     pinMode(motor.getPin(MOTOR3_EN_6), OUTPUT);
     pinMode(motor.getPin(MOTOR3_EN_7), OUTPUT);
     
     pinMode(motor.getPin(MOTOR3_IO_0), OUTPUT);
     pinMode(motor.getPin(MOTOR3_IO_1), OUTPUT);
     pinMode(motor.getPin(MOTOR3_IO_2), OUTPUT);
     pinMode(motor.getPin(MOTOR3_IO_3), OUTPUT);
     pinMode(motor.getPin(MOTOR3_IO_4), OUTPUT);
     pinMode(motor.getPin(MOTOR3_IO_5), OUTPUT);
     pinMode(motor.getPin(MOTOR3_IO_6), OUTPUT);
     pinMode(motor.getPin(MOTOR3_IO_7), OUTPUT);
     
     pinMode(motor.getPin(MOTOR3_PWM_0), OUTPUT);
     pinMode(motor.getPin(MOTOR3_PWM_1), OUTPUT);
     pinMode(motor.getPin(MOTOR3_PWM_2), OUTPUT);
     pinMode(motor.getPin(MOTOR3_PWM_3), OUTPUT);
     pinMode(motor.getPin(MOTOR3_PWM_4), OUTPUT);
     pinMode(motor.getPin(MOTOR3_PWM_5), OUTPUT);
     pinMode(motor.getPin(MOTOR3_PWM_6), OUTPUT);
     pinMode(motor.getPin(MOTOR3_PWM_7), OUTPUT);
     /*
    digitalWrite(motor.getPin(MOTOR3_EN_0), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_1), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_2), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_3), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_4), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_5), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_6), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_7), LOW);
    
    
    
       for (int i=0;i<16;i++)
    digitalWrite(motor.getPin(MCP23017_ADDR_BASE<<16)+i, LOW);
   
    for (int i=0;i<16;i++)
    digitalWrite(motor.getPin(MCP23017_ADDR_BASE<<16)+i, HIGH);
  








    delay(1000);  
 
*/
  
    digitalWrite(motor.getPin(MOTOR3_IO_0), LOW);//4
    digitalWrite(motor.getPin(MOTOR3_IO_1), LOW);//5
    digitalWrite(motor.getPin(MOTOR3_IO_2), LOW);//7
    digitalWrite(motor.getPin(MOTOR3_IO_3), LOW);//6
    digitalWrite(motor.getPin(MOTOR3_IO_4), LOW);
    digitalWrite(motor.getPin(MOTOR3_IO_5), LOW);
    digitalWrite(motor.getPin(MOTOR3_IO_6), LOW);
    digitalWrite(motor.getPin(MOTOR3_IO_7), LOW);//M3

    digitalWrite(motor.getPin(MOTOR3_PWM_0), LOW);
    digitalWrite(motor.getPin(MOTOR3_PWM_1), LOW);
    digitalWrite(motor.getPin(MOTOR3_PWM_2), LOW);
    digitalWrite(motor.getPin(MOTOR3_PWM_3), LOW);
    digitalWrite(motor.getPin(MOTOR3_PWM_4), LOW);
    digitalWrite(motor.getPin(MOTOR3_PWM_5), LOW);
    digitalWrite(motor.getPin(MOTOR3_PWM_6), LOW);
    digitalWrite(motor.getPin(MOTOR3_PWM_7), LOW);
    
    digitalWrite(motor.getPin(MOTOR3_EN_0), HIGH);
    digitalWrite(motor.getPin(MOTOR3_EN_1), HIGH);
    digitalWrite(motor.getPin(MOTOR3_EN_2), HIGH);
    digitalWrite(motor.getPin(MOTOR3_EN_3), HIGH);
    digitalWrite(motor.getPin(MOTOR3_EN_4), HIGH);
    
    digitalWrite(motor.getPin(MOTOR3_EN_5), HIGH);
    digitalWrite(motor.getPin(MOTOR3_EN_6), HIGH);
    digitalWrite(motor.getPin(MOTOR3_EN_7), HIGH);
   
/*
    digitalWrite(motor.getPin(MOTOR3_PWM_0), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_1), HIGH);//c
    digitalWrite(motor.getPin(MOTOR3_PWM_2), HIGH);//b
    digitalWrite(motor.getPin(MOTOR3_PWM_3), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_4), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_5), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_6), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_7), HIGH);//M3



    digitalWrite(motor.getPin(MOTOR3_EN_0), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_1), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_2), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_3), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_5), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_6), LOW);
    digitalWrite(motor.getPin(MOTOR3_EN_7), LOW);//M3
    
    
    digitalWrite(motor.getPin(MOTOR3_IO_4), HIGH);
   
    delay(1000);  
    digitalWrite(motor.getPin(MOTOR3_IO_0), HIGH);
    digitalWrite(motor.getPin(MOTOR3_IO_1), HIGH);
    digitalWrite(motor.getPin(MOTOR3_IO_2), HIGH);
    digitalWrite(motor.getPin(MOTOR3_IO_3), HIGH);
    digitalWrite(motor.getPin(MOTOR3_IO_5), HIGH);
    digitalWrite(motor.getPin(MOTOR3_IO_6), HIGH);
    digitalWrite(motor.getPin(MOTOR3_IO_7), HIGH);

    digitalWrite(motor.getPin(MOTOR3_PWM_0), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_1), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_2), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_3), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_4), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_5), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_6), HIGH);
    digitalWrite(motor.getPin(MOTOR3_PWM_7), HIGH);
    delay(1000);
    
    analogWrite(motor.getPin(MOTOR3_PWM_0), 200);
    analogWrite(motor.getPin(MOTOR3_PWM_1), 500);
    analogWrite(motor.getPin(MOTOR3_PWM_2), 1000);
    analogWrite(motor.getPin(MOTOR3_PWM_3), 1500);
    analogWrite(motor.getPin(MOTOR3_PWM_4), 2000);
    analogWrite(motor.getPin(MOTOR3_PWM_5), 2500);
    analogWrite(motor.getPin(MOTOR3_PWM_6), 3000);
    analogWrite(motor.getPin(MOTOR3_PWM_7), 3500);  
  */  

}
 