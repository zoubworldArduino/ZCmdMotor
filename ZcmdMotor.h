


/*==================================================================================================*/
/*==================================================================================================*/
/*==================================================================================================*/


#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <ZEncoder.h>
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
  ZEncoder * getEncoder();
  CMDMOTOR(int INCA, int INCB, int MP, int MM);
  void setPin(int INCA, int INCB, int MP, int MM);
  void loop();
  void setup();
  void setPWMValue(signed int td);
  void stop();
  void setSerialDebug(Uart * SerialDebug);
  
private:
  ZEncoder * encoder;
  Uart * SerialDebug;
  boolean enabled;

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