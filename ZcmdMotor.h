
/*==================================================================================================*/
/*==================================================================================================*/
/*==================================================================================================*/


#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <ZEncoder.h>


#ifdef ROS_USED 
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#endif 


#define MIN(a,b) ((a<b)?a:b)
#define MAX(a,b) ((a<b)?b:a)

class CMDMOTOR {

  int pinINCA;
  int pinINCB;
  int pinMP;
  int pinMM;
public:

  double point, Input, Output;
  double Kp , Ki , Kd ; //speed
//  double Kp=2, Ki=3, Kd=0.001;//distance
      // double Kp=10, Ki=0.1, Kd=0.1;//distance
  double dpointMax ;
  double newPoint;
  PID * pid;
  PID_ATune *aTune;
  //GroveEncoder *Encoder;
  void setPoint(int point);
  int getPoint();
  PID * getPID();
  ZEncoder * getEncoder();
  CMDMOTOR(int INCA, int INCB, int MP, int MM);
  void setPin(int INCA, int INCB, int MP, int MM);
  	/** setup the refresh rate of the topic
	*/
  void setRefreshRateUs(uint32_t intervalTime //!< duration between 2 topic in Micro Seconde
  );
  void loop();
  void setup();
   #ifdef ROS_USED 
    void setup( ros::NodeHandle * myNodeHandle,	const char   *	topicPWM,	const char   *	topicSPEED); 
#endif
  void setPWMValue(signed int td);
  void stop();
  void setSerialDebug(HardwareSerial * SerialDebug);
    boolean enabled;///don't use

private:
  #ifdef ROS_USED 
    ros::NodeHandle  *nh;
    std_msgs::Int16 pwm_msg;//speed //deltaD//D
    std_msgs::Int32 speed_msg;//speed //deltaD//D
    ros::Subscriber<std_msgs::Int16> *subPWM;
    ros::Subscriber<std_msgs::Int32> *subSpeed;
        
#endif
  ZEncoder * encoder;
  HardwareSerial * SerialDebug;

  boolean tuning ;
double aTuneStep, aTuneNoise, aTuneStartValue;
unsigned int aTuneLookBack;
void changeAutoTune();
byte ATuneModeRemember;

void AutoTuneHelper(boolean start);
  void Callback(int newValue);
  signed char olddirection ;
  signed char direction ;
};