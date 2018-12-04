
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
  newPoint = mypoint;
  if ((newPoint - point) > 0) {
    point = point + MIN(dpointMax, (newPoint - point));
  } else {
    point = point - MIN(dpointMax, (point - newPoint));
  }

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
 Kp = 20;//1
 Ki = 30;//0.05
 Kd = 0.0005; //0.0005
 olddirection = 1;
 direction = -1;
 dpointMax = 50;
    point=0; Input=1; Output=0;
 pid = new PID(&Input, &Output, &point, Kp, Ki, Kd, DIRECT);
// pid->SetSampleTime(10);
 aTune  = new PID_ATune(&Input, &Output);
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
  pinINCA = INCA;
  pinINCB = INCB;
  pinMP = MP;
  pinMM = MM;

}
void CMDMOTOR::setPin(int INCA, int INCB, int MP, int MM) 
{
    pinINCA = INCA;
  pinINCB = INCB;
  pinMP = MP;
  pinMM = MM;

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
  pinMode(pinMP, OUTPUT);
  pinMode(pinMM, OUTPUT);
  digitalWrite(pinMP, LOW);
  digitalWrite(pinMM, LOW);
  encoder = new ZEncoder(pinINCA, pinINCB, FULL, NULL); //FULL, QUARTER
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
       noInterrupts();
    digitalWrite(pinMM, LOW);
    pinMode(pinMP, OUTPUT);
    digitalWrite(pinMP, LOW);
     interrupts();
  if (SerialDebug) SerialDebug->print("Mx= STOP,");
  } else if (m1 > 0) {
      noInterrupts();
    digitalWrite(pinMM, LOW);
    
    analogWrite(pinMP, m1);
     interrupts();
    if (SerialDebug) 
    {
    SerialDebug->print("Mx= LOW,");
    SerialDebug->print(m1);
    SerialDebug->print(" \r\n");
    }
  } else {
     noInterrupts();
    digitalWrite(pinMM, HIGH);
    analogWrite(pinMP, PWMMAX + m1);
     interrupts();
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
  
  if (enabled)
  {
 
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
  }/*
  SerialDebug->print(", point=: ");
  SerialDebug->print(point);
  SerialDebug->print("\r\n");*/
 // Input = getEncoder()->getValue();
 #if ENABLE_SPEED

  int speed = getEncoder()->getSpeed();
#else
   int speed = getEncoder()->getDeltaValue();
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


CMDMOTOR * myZcmdmotor[4];
static void callbackinstancepwm0( const std_msgs::Int16& pwm_msg)
{	
  myZcmdmotor[0]->enabled=false;
  myZcmdmotor[0]->setPWMValue(-pwm_msg.data);  
}

static void callbackinstancespeed0( const std_msgs::Int16& speed_msg)
{	
  myZcmdmotor[0]->setPoint(-speed_msg.data);  

}

static void callbackinstancepwm1( const std_msgs::Int16& pwm_msg)
{	
  myZcmdmotor[1]->enabled=false;
  myZcmdmotor[1]->setPWMValue(-pwm_msg.data);  
}

static void callbackinstancespeed1( const std_msgs::Int16& speed_msg)
{	
  myZcmdmotor[1]->setPoint(-speed_msg.data);    
}

static void callbackinstancepwm2( const std_msgs::Int16& pwm_msg)
{	
  myZcmdmotor[2]->enabled=false;
  myZcmdmotor[2]->setPWMValue(-pwm_msg.data);  
 }

static void callbackinstancespeed2( const std_msgs::Int16& speed_msg)
{	
  myZcmdmotor[2]->setPoint(-speed_msg.data);    
}
static void callbackinstancepwm3( const std_msgs::Int16& pwm_msg)
{	
  myZcmdmotor[3]->enabled=false;
  myZcmdmotor[3]->setPWMValue(-pwm_msg.data);    
}

static void callbackinstancespeed3( const std_msgs::Int16& speed_msg)
{	
  myZcmdmotor[3]->setPoint(speed_msg.data);    
}
static void(*callbackinstancespeed[4])(const std_msgs::Int16& cmd_msg)={
	callbackinstancespeed0,callbackinstancespeed1,callbackinstancespeed2,callbackinstancespeed3	
	};
static void(*callbackinstancepwm[4])(const std_msgs::Int16& cmd_msg)={
	callbackinstancepwm0,callbackinstancepwm1,callbackinstancepwm2,callbackinstancepwm3	
	};
static  int index=0;
/** setup :
  At setup after NodeHandle setup, call this to initialise the topic
*/
void CMDMOTOR::setup( ros::NodeHandle * myNodeHandle,	const char   *	topicPWM,	const char   *	topicSPEED)
{
  assert(index<4);
  myZcmdmotor[index]= this;
  nh=myNodeHandle;
  subPWM=new ros::Subscriber<std_msgs::Int16> (topicPWM, callbackinstancepwm[index]); 
  subSpeed=new ros::Subscriber<std_msgs::Int16> (topicSPEED, callbackinstancespeed[index]); 
  nh->subscribe(*subPWM); 
  nh->subscribe(*subSpeed); 
  DEBUG(nh->loginfo("CMDMOTOR::setup()")); 
  DEBUG(nh->loginfo(topicPWM)); 
  DEBUG(nh->loginfo(topicSPEED)); 
  index++;
  
}


#endif 
