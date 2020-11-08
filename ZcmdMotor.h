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
	int pinEN;
        boolean inverted=false; ///don't use
public:
        bool isForcing();
	double point, Input, Output;
	double Kp , Ki , Kd,K0; //speed
//  double Kp=2, Ki=3, Kd=0.001;//distance
	// double Kp=10, Ki=0.1, Kd=0.1;//distance
	double dpointMax;
	double newPoint;
	PID * pid;
	PID_ATune *aTune;
	//GroveEncoder *Encoder;
	void setPoint(signed int point);
	void setSpeed(signed int point);
	int getPoint();
	PID * getPID();
        PID * getPIDPosition();
	ZEncoder * getEncoder();
	CMDMOTOR(int INCA, int INCB, int MP, int MM);
	void setPin(int INCA, int INCB, int MP, int MM);
	void setPin(int INCA, int INCB, int MP, int MM, int EN,boolean invert);
	/** setup the refresh rate of the topic
	 */
	void setRefreshRateUs(uint32_t intervalTime //!< duration between 2 topic in Micro Seconde
	);
	void loop();

	void calib();
	void setup();
#ifdef ROS_USED 
	void setup( ros::NodeHandle * myNodeHandle, const char * topicPWM, const char * topicSPEED);
#endif
	void setPWMValue(signed int td);
	void stop();
	void setSerialDebug(HardwareSerial * SerialDebug);
	boolean enabled; ///don't use
 
void SetPwmLimits(signed int Min,signed int Max);
	
        void SetSpeedLimits(double Min, double Max);
        boolean isLengthReached();
        void setLength(signed int point);
        /** reset length computation, in case of force */
	void stopLength();
	
private:
 
  double length;//distance done
  double position;//current pos n tick
  bool LengthEnabled;
  PID * pidpos;
  boolean  LengthReached=false;// indicate if job is done
  int preverror=1;//identify the previous error up to the target
	
  
  void testPID(signed int speed, unsigned int responsetime);
#ifdef ROS_USED 
	ros::NodeHandle *nh;
	std_msgs::Int16 pwm_msg; //speed //deltaD//D
	std_msgs::Int32 speed_msg;//speed //deltaD//D
	ros::Subscriber<std_msgs::Int16> *subPWM;
	ros::Subscriber<std_msgs::Int32> *subSpeed;

#endif
	ZEncoder * encoder;
	HardwareSerial * SerialDebug;

	boolean tuning;
	double aTuneStep, aTuneNoise, aTuneStartValue;
	unsigned int aTuneLookBack;
	void changeAutoTune();
	byte ATuneModeRemember;

	void AutoTuneHelper(boolean start);
	void Callback(int newValue);
	signed char olddirection;
	signed char direction;
};