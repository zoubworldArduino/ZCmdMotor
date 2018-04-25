
#include <ZcmdMotor.h>




/*==================================================================================================*/
/*==================================================================================================*/
/*==================================================================================================*/



void CMDMOTOR::setSerialDebug(Uart * mySerialDebug)
{
  SerialDebug=mySerialDebug;
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
 Kp = 1;//1
 Ki = 1;//0.05
 Kd = 0.00005; //0.0005
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
  enabled=false;
  setPWMValue(0);
  
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
  if (enabled)
  {
  /*
  SerialDebug->print("\r\n");
  SerialDebug->print("\r\n");

  SerialDebug->print(", newPoint=: ");
  SerialDebug->print(newPoint);

  SerialDebug->print(", point=: ");
  SerialDebug->print(point);
  SerialDebug->print(", (newPoint-point)=: ");
  SerialDebug->print((newPoint - point));
*/
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

  int speed = getEncoder()->getSpeed();
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




























































