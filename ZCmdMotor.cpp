
#include <ZcmdMotor.h>
//#define DEBUG(a) a
#define DEBUG(a) {}
#include <assert.h>



/*==================================================================================================*/
/*==================================================================================================*/
/*==================================================================================================*/



void CMDMOTOR::setSerialDebug(HardwareSerial * mySerialDebug)
{
	SerialDebug=mySerialDebug;
}
int CMDMOTOR::getPoint()
{
	return point;
}
void CMDMOTOR::setPoint(int mypoint) {

	enabled=true;

#ifdef SMOOTH
	newPoint = mypoint;
	if ((newPoint - point) > 0) {
		point = point + MIN(dpointMax, (newPoint - point));
	} else {
		point = point - MIN(dpointMax, (point - newPoint));
	}
#else
	point=mypoint;
#endif
}
void CMDMOTOR::Callback(int newValue) {
	SerialDebug->print("D:");
	SerialDebug->print(newValue, HEX);
	SerialDebug->print("\r\n");
}



void CMDMOTOR::AutoTuneHelper(boolean start)
{
	if(start)
		ATuneModeRemember = pid->GetMode();
	else
		pid->SetMode(ATuneModeRemember);
}

void CMDMOTOR::changeAutoTune()
{
	if(!tuning)
	{
		//Set the output to the desired starting frequency.
		Output=aTuneStartValue;
		aTune->SetNoiseBand(aTuneNoise);
		aTune->SetOutputStep(aTuneStep);
		aTune->SetLookbackSec((int)aTuneLookBack);
		AutoTuneHelper(true);
		tuning = true;
	}
	else
	{ //cancel autotune
		aTune->Cancel();
		tuning = false;
		AutoTuneHelper(false);
	}
}

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
CMDMOTOR::CMDMOTOR(int INCA, int INCB, int MP, int MM) {
#ifdef ROS_USED 
	nh=0;
#endif
        K0=0;
	Kp = 4000.0/371029.0;// 10% of error
	Ki =4000.0/371029.0*10;//;// 4000.0/371029.0;//30;//0.05
	Kd =Kp/1000;//0.0005; //0.0005
	olddirection = 1;
	direction = -1;
	dpointMax = 50;
	point=0; Input=1; Output=0;
	pid = new PID(&Input, &Output, &point, Kp, Ki, Kd, DIRECT);
	setRefreshRateUs(20000);
	assert(pid!=0);// heap issue.
	// pid->SetSampleTime(10);
	aTune  = new PID_ATune(&Input, &Output);
	assert(aTune!=0);// heap issue.
	tuning = false;
	enabled=false;
	aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
	aTuneLookBack=20;
	SerialDebug=0;
	ATuneModeRemember=2;
	/*
 if (!SerialDebug->enabled())
 {
 SerialDebug->begin(9600);  //1000000 , 115200
 }*/
	if (SerialDebug) {
		SerialDebug->print("CMDMOTOR(");
		SerialDebug->print((signed int) INCA);
		SerialDebug->print(",");
		SerialDebug->print((signed int) INCA);
		SerialDebug->print(",");
		SerialDebug->print((signed int) MP);
		SerialDebug->print(",");
		SerialDebug->print((signed int) MM);
		SerialDebug->print(") : ");
	}
	setPin(INCA, INCB, MP, MM);

}
/** setup the refresh rate of the topic
 */
void CMDMOTOR::setRefreshRateUs(uint32_t intervalTime //!< duration between 2 topic in Micro Seconde
)
{
	if(getEncoder()!=0) 
		getEncoder()->setRefreshRateUs(intervalTime);
	if(pid!=0) 
		pid->SetSampleTime(intervalTime/1000);
}



void CMDMOTOR::testPID(signed int speed, unsigned int responsetime)
{
  stop();
  signed int encp2=0;
  signed int encp=getEncoder()->getSpeed();//flush a computation
   while(encp!=0) // wait speed update to 0
   { wdt_clr();
 encp=getEncoder()->getSpeed();
   }
  getPID()->SetTunings( Kp,  Ki,  Kd);
  getPID()->Initialize();
   
  setPoint(speed);
  DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }
  encp=getEncoder()->getSpeed();
  {DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }}
  encp2=getEncoder()->getSpeed();
   stop();delay(100);
  while(getEncoder()->getSpeed()!=0) // wait speed update to 0
  wdt_clr();
}
/**
tuning methodes :
 Zeigler-Nichols (ZN) 
 Tyreus-Luyben or Rivera, Morari and Skogestad.
*/
void CMDMOTOR::calib() 
{
  int i=0;
  // find k0ff kpff : feed forward parameter
  stop();
  signed int pwm0=-1;
  while(i>pwm0)
  {// search again the K0, on bad motoreductor, these is several blocking point
      pwm0=i;
    setPWMValue(pwm0/2);//apply a small pwm to go on block point
    delay(100);getEncoder()->getSpeed();
    int timeout=1000;wdt_clr();
    while(getEncoder()->getSpeed()!=0 && (timeout-->0))
      delay(1);
    for(i=pwm0/2;(i<4096) && (getEncoder()->getSpeed()==0);i++)
    {
      setPWMValue(i);delay(1);
    }  
  }
    pwm0=i;
  signed int enc0=getEncoder()->getSpeed();  
  signed int pwm1=4096;
  setPWMValue(pwm1);
  wdt_clr();delay(700);wdt_clr();
  signed int enc1=getEncoder()->getSpeed();
   stop();delay(100); wdt_clr();
  double K0ff=pwm0;
  double Kpff=pwm1-pwm0;
  Kpff=Kpff/(enc1-enc0);
  getPID()->setFeedForward(K0ff,Kpff);
  wdt_clr();
  // estimate kp PID parameter
 double Kp=Kpff/8;//kp~kpff
 
 
  unsigned int responsetime=getPID()->getSampleTime()/1000*25;
 
  signed int encp=getEncoder()->getSpeed();//flush a computation
  signed int encp2=0;
   while(encp!=0) // wait speed update to 0
   { wdt_clr();
 encp=getEncoder()->getSpeed();
   }
   ////////////////////////////////////////////////////
   
   
    setPWMValue(pwm1*8/10);
   unsigned long t0=micros();
    while(encp<enc1*5/100) // wait speed update to 0
   { wdt_clr();
 encp=getEncoder()->getSpeed();
 delay(1);
   }
    unsigned long t1=micros();
     while(encp2<enc1*64/100) // wait speed update to 0
   { wdt_clr();
 encp2=getEncoder()->getSpeed();
 delay(1);
   }
    
      unsigned long t2=micros();
 stop();delay(100);
    unsigned long L=t1-t0;
       unsigned long T=t2-t1;
       
   Kp=1.2*T/L;
   Ki=2.0*L/1000000.0;
   Kd=0.5*L/1000000.0;
   Kp*=Kpff;
     Ki*=Kpff;
   Kd*=Kpff;
    getPID()->SetTunings( Kp,  Ki,  Kd);
   
    /*
   responsetime=1000;
    {
     signed int speed=enc1/10;// adjust at low speed 5%
testPID( speed,  responsetime);

speed=enc1/2;// adjust at low speed 5%
testPID( speed,  responsetime);
}
*/
  /* 
   //////////////////////////////////////
 signed int speed=enc1/20;// adjust at low speed 5%
while (encp<speed*8/10)
{
  Kp*=2;
  getPID()->SetTunings( Kp,  0,  0);
  getPID()->Initialize();
   
  setPoint(speed);
  DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }
  encp=getEncoder()->getSpeed();
  {DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }}
  encp2=getEncoder()->getSpeed();
   stop();delay(100);
  while(getEncoder()->getSpeed()!=0) // wait speed update to 0
  wdt_clr();
}


stop();
 Kp/=2;
speed=enc1/2;// adjust at high speed 50%
while(encp!=0)
 encp=getEncoder()->getSpeed();
while (encp<speed*8/10)
{
  Kp*=2;
  getPID()->SetTunings( Kp,  0,  0);
  getPID()->Initialize();
   
  setPoint(speed);
  DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }
  encp=getEncoder()->getSpeed();
  { DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }}
  encp2=getEncoder()->getSpeed();
   stop(); delay(100);
  while(getEncoder()->getSpeed()!=0) // wait speed update to 0
  wdt_clr();
}
// define Ki

Ki=Kp/10;// Ki~Kp/5 in 1st approch.
stop();
speed=enc1/10;// adjust at low speed 10%
while(encp!=0)
 encp2=encp=getEncoder()->getSpeed();

while (abs(encp2-speed)>(speed*3)/100)
{
  Ki*=2;
  getPID()->SetTunings( Kp,  Ki,  0);
  getPID()->Initialize();
   
  setPoint(speed);
  DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }
  encp=getEncoder()->getSpeed();
  {
  DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }
  }
  encp2=getEncoder()->getSpeed();
   stop(); delay(100);
  while(getEncoder()->getSpeed()!=0) // wait speed update to 0
  wdt_clr();
}

speed=enc1/2;// adjust at high speed 50%
while(encp!=0)
 encp2=encp=getEncoder()->getSpeed();
Ki/=2;
while (abs(encp2-speed)>(speed*3)/100)
{
  Ki*=2;
  getPID()->SetTunings( Kp,  Ki,  0);
  getPID()->Initialize();
   
  setPoint(speed);
  DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }
  encp=getEncoder()->getSpeed();
  {
  DELAYLOOP( responsetime) 
  {
   loop();
   delay(1);
   wdt_clr();
  }
  }
  encp2=getEncoder()->getSpeed();
   stop(); delay(100);
  while(getEncoder()->getSpeed()!=0) // wait speed update to 0
  wdt_clr();
}

*/

}
char fifoin=0;
char fifoout=0;
#define SizeFIFO 10
signed int dist[SizeFIFO];
signed int  speed[SizeFIFO];
boolean disten=false;
boolean CMDMOTOR::addPath(signed int dist1,signed inst speed1) 
{
  if (((fifoin+1)%SizeFIFO)==fifoout)
    return false;
  
  dist[fifoin]=dist1;
  speed[fifoin]=speed1;
  fifoin++;  fifoin=fifoin>=SizeFIFO?0:fifoin;
  if(!disten)
  {
    currentDistTarget=getEncoder()->getValue();
setPoint(speed1);
setDist(dist1);
fifoout++;  fifoout=fifoout>=SizeFIFO?0:fifoout;

  }

}
/** define the distance to do
dist1 is the coder inc step.
*/
void setDist(signed int dist1)
{
  currentDistTarget+=dist1;
disten=true;  
  
}
/** identify the current distance target*/
signed int currentDistTarget=0;

void CMDMOTOR::setPin(int INCA, int INCB, int MP, int MM) 
{
	setPin( INCA,  INCB,  MP,  MM, -1) ;
}
void CMDMOTOR::setPin(int INCA, int INCB, int MP, int MM, int EN) 
{
	pinINCA = INCA;
	pinINCB = INCB;
	pinMP = MP;
	pinMM = MM;
	pinEN = EN;

	pinMode(pinMP, OUTPUT);
	pinMode(pinMM, OUTPUT);
	digitalWrite(pinMP, LOW);
	digitalWrite(pinMM, LOW);


	pinMode(pinEN, OUTPUT);
	digitalWrite(pinEN, HIGH); 
	if(encoder==0)
	{
		if  (!((pinINCA==-1) &&(pinINCB==-1) ))
		{
			encoder = new ZEncoder(pinINCA, pinINCB, QUARTER, NULL); //FULL, QUARTER
			assert(encoder!=0);// heap issue.
		}
	}
	else
	{
		/*
  encoder->setPin( INCA,  INCB);
		 */
	}
}

ZEncoder * CMDMOTOR::getEncoder() {
	return encoder;
}
PID * CMDMOTOR::getPID() {
	return pid;
}
void CMDMOTOR::setup() {
	if (SerialDebug) {

		SerialDebug->print("CMDMOTOR.setup(");
		SerialDebug->print((signed int) pinINCA);
		SerialDebug->print(",");
		SerialDebug->print((signed int) pinINCB);
		SerialDebug->print(",");
		SerialDebug->print((signed int) pinMP);
		SerialDebug->print(",");
		SerialDebug->print((signed int) pinMM);
		SerialDebug->print(") : ");
	}
	if (!((pinINCA==-1) &&(pinINCB==-1) ))
	{
		encoder = new ZEncoder(pinINCA, pinINCB, QUARTER, NULL); //FULL, QUARTER
		assert(encoder!=0);// heap issue.
	}
	//turn the PID on
	pid->SetMode(MANUAL);

	//pid->SetOutputLimits(-255, 255);

	pid->SetOutputLimits(-14096,14095);
	pid->SetSampleTime(10);  //10ms
	pid->SetMode(AUTOMATIC);
	//  pid.SetControllerDirection(REVERSE );
	if(tuning)
	{
		tuning=false;
		changeAutoTune();
		tuning=true;
	}


}

#define  PWMMAX 4096

void CMDMOTOR::stop()
{
	newPoint=point=0;
	Input=Output=0;
	enabled=false;
	setPWMValue(0);
#if ENABLE_SPEED
	if (getEncoder()!=0)
		getEncoder()->resetSpeed();
#endif
	getPID()->Initialize();
}
/* voltage apply on motor */
void CMDMOTOR::setPWMValue(signed int m1) {
	//cut it only if direction change to prevent from speak during polaryty change on motor
	Output = m1;

	m1 = (m1 > PWMMAX) ? PWMMAX : m1;
	m1 = (m1 < -PWMMAX) ? -PWMMAX : m1;
	if (m1 == 0) {
		//       noInterrupts();
		digitalWrite(pinMM, LOW);
		digitalWrite(pinMP, LOW);
		//     interrupts();
		if (SerialDebug) SerialDebug->print("Mx= STOP,");
	} else if (m1 > 0) {
		//      noInterrupts();
		digitalWrite(pinMM, LOW);
		analogWrite(pinMP, m1);
		//   interrupts();
		if (SerialDebug) 
		{
			SerialDebug->print("Mx= LOW,");
			SerialDebug->print(m1);
			SerialDebug->print(" \r\n");
		}
	} else {
		//  noInterrupts();
		digitalWrite(pinMM, HIGH);
		analogWrite(pinMP, PWMMAX + m1);
		//   interrupts();
		if (SerialDebug) 
		{
			SerialDebug->print("Mx= HIGH,");
			SerialDebug->print(m1);
			SerialDebug->print(" \r\n");
		}
	}

}

void CMDMOTOR::loop() {

#ifdef ROS_USED 
	getEncoder()->loop();
#endif

        
        
  if(disten)
  {
    if ( ((point>0)&& (getEncoder()->getValue()>=currentDistTarget) )
        ||((getEncoder()->getValue()<=currentDistTarget) && (point<0)))
      {// next point
        if(fifoin==fifoout)
        {
          stop();
          disten=false;
        }
        else
        {
          setpoint(speed[fifoout]);
          setDist(setDist[fifoout]);
          fifoout++;  fifoout=fifoout>=SizeFIFO?0:fifoout;
        }
      }
  }



	if (enabled)
	{
#ifdef SMOOTH
		if (newPoint != point) {
			if ((newPoint - point) > 0) {
				point = point + MIN(dpointMax, (newPoint - point));
				/* SerialDebug->print(", MIN(dpointMax,(newPoint-point)=: ");
      SerialDebug->print(MIN(dpointMax, (newPoint - point)));*/
			} else {
				point = point - MIN(dpointMax, (point - newPoint));
				/*  SerialDebug->print(", -MIN(dpointMax,(point-newPoint)=: ");
      SerialDebug->print(MIN(dpointMax, (point - newPoint)));*/
			}
		}
#endif

		/*
  SerialDebug->print(", point=: ");
  SerialDebug->print(point);
  SerialDebug->print("\r\n");*/
		// Input = getEncoder()->getValue();
#if ENABLE_SPEED

		signed int speed = getEncoder()->getSpeed();
#else
		signed int speed = getEncoder()->getDeltaValue();
#endif
		Input = speed;


		int res =0;


		if(tuning)
		{
			byte val = (aTune->Runtime());
			if (val!=0)
			{
				tuning = false;
			}
			if(!tuning)
			{ //we're done, set the tuning parameters
				Kp = aTune->GetKp();
				Ki = aTune->GetKi();
				Kd = aTune->GetKd();
				pid->SetTunings(Kp,Ki,Kd);
				AutoTuneHelper(false);
			}
		}
		else
			res = pid->Compute();

		/*  SerialDebug->print(", newPID=: ");  SerialDebug->print(res);*/
		if(res)// if refresh
			setPWMValue((signed int)Output);

#ifdef PcomPlotter

		PcomPlotter.print(point);
		PcomPlotter.print("\t");
		PcomPlotter.print(Input);

		PcomPlotter.print("\t");
		PcomPlotter.print(Output);
		PcomPlotter.print("\t");
#endif
		/*
   SerialDebug->print(", spd:");
   SerialDebug->print(speed);

   SerialDebug->print(", ms:");
   SerialDebug->print(millis());*/
		/*

   SerialDebug->print(", Kpid: ");
   SerialDebug->print(pid.GetKp());SerialDebug->print(",");
   SerialDebug->print(pid.GetKi());SerialDebug->print(",");
   SerialDebug->print(pid.GetKd());SerialDebug->print(",");
   SerialDebug->print(", GetDirection: ");
   SerialDebug->print(pid.GetDirection());SerialDebug->print(",");

   SerialDebug->print(", GetMode: ");
   SerialDebug->print(pid.GetMode());SerialDebug->print(",");*/


	}
}



#ifdef ROS_USED


CMDMOTOR * myZcmdmotor[8];
static void callbackinstancepwm0( const std_msgs::Int16& pwm_msg)
{	
	myZcmdmotor[0]->enabled=false;
	myZcmdmotor[0]->setPWMValue(-pwm_msg.data);  
}

static void callbackinstancespeed0( const std_msgs::Int32& speed_msg)
{	
	myZcmdmotor[0]->setPoint(-speed_msg.data);  

}

static void callbackinstancepwm1( const std_msgs::Int16& pwm_msg)
{	
	myZcmdmotor[1]->enabled=false;
	myZcmdmotor[1]->setPWMValue(-pwm_msg.data);  
}

static void callbackinstancespeed1( const std_msgs::Int32& speed_msg)
{	
	myZcmdmotor[1]->setPoint(-speed_msg.data);    
}

static void callbackinstancepwm2( const std_msgs::Int16& pwm_msg)
{	
	myZcmdmotor[2]->enabled=false;
	myZcmdmotor[2]->setPWMValue(-pwm_msg.data);  
}

static void callbackinstancespeed2( const std_msgs::Int32& speed_msg)
{	
	myZcmdmotor[2]->setPoint(-speed_msg.data);    
}
static void callbackinstancepwm3( const std_msgs::Int16& pwm_msg)
{	
	myZcmdmotor[3]->enabled=false;
	myZcmdmotor[3]->setPWMValue(-pwm_msg.data);    
}

static void callbackinstancespeed3( const std_msgs::Int32& speed_msg)
{	
	myZcmdmotor[3]->setPoint(speed_msg.data);    
}

static void callbackinstancepwm4( const std_msgs::Int16& pwm_msg)
{	
	myZcmdmotor[4]->enabled=false;
	myZcmdmotor[4]->setPWMValue(-pwm_msg.data);    
}

static void callbackinstancespeed4( const std_msgs::Int32& speed_msg)
{	
	myZcmdmotor[4]->setPoint(speed_msg.data);    
}

static void callbackinstancepwm5( const std_msgs::Int16& pwm_msg)
{	
	myZcmdmotor[5]->enabled=false;
	myZcmdmotor[5]->setPWMValue(-pwm_msg.data);    
}

static void callbackinstancespeed5( const std_msgs::Int32& speed_msg)
{	
	myZcmdmotor[5]->setPoint(speed_msg.data);    
}

static void callbackinstancepwm6( const std_msgs::Int16& pwm_msg)
{	
	myZcmdmotor[6]->enabled=false;
	myZcmdmotor[6]->setPWMValue(-pwm_msg.data);    
}

static void callbackinstancespeed6( const std_msgs::Int32& speed_msg)
{	
	myZcmdmotor[6]->setPoint(speed_msg.data);    
}

static void callbackinstancepwm7( const std_msgs::Int16& pwm_msg)
{	
	myZcmdmotor[7]->enabled=false;
	myZcmdmotor[7]->setPWMValue(-pwm_msg.data);    
}

static void callbackinstancespeed7( const std_msgs::Int32& speed_msg)
{	
	myZcmdmotor[7]->setPoint(speed_msg.data);    
}

static void(*callbackinstancespeed[8])(const std_msgs::Int32& cmd_msg)={
		callbackinstancespeed0,callbackinstancespeed1,callbackinstancespeed2,callbackinstancespeed3,
		callbackinstancespeed4,callbackinstancespeed5,callbackinstancespeed6,callbackinstancespeed7	
};
static void(*callbackinstancepwm[8])(const std_msgs::Int16& cmd_msg)={
		callbackinstancepwm0,callbackinstancepwm1,callbackinstancepwm2,callbackinstancepwm3,
		callbackinstancepwm4,callbackinstancepwm5,callbackinstancepwm6,callbackinstancepwm7	
};
static  int index=0;
/** setup :
  At setup after NodeHandle setup, call this to initialise the topic
 */
void CMDMOTOR::setup( ros::NodeHandle * myNodeHandle,	const char   *	topicPWM,	const char   *	topicSPEED)
{
	assert(index<8);
	myZcmdmotor[index]= this;
	nh=myNodeHandle;
	subPWM=new ros::Subscriber<std_msgs::Int16> (topicPWM, callbackinstancepwm[index]); 
	assert(subPWM!=0);// heap issue.
	subSpeed=new ros::Subscriber<std_msgs::Int32> (topicSPEED, callbackinstancespeed[index]); 
	assert(subSpeed!=0);// heap issue.
	nh->subscribe(*subPWM); 
	nh->subscribe(*subSpeed); 
	DEBUG(nh->loginfo("CMDMOTOR::setup()")); 
	DEBUG(nh->loginfo(topicPWM)); 
	DEBUG(nh->loginfo(topicSPEED)); 
	index++;

}


#endif 
