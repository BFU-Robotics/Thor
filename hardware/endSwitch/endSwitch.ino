#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include <GyverStepper.h>

modbusDevice regBank;
modbusSlave slave;

const int bigRotationSwitchPin = A0;
const float bigRotationGearK = 5;

// Шаговые двигатели
GStepper<STEPPER2WIRE> bigRotationStepper(3200, 8, 9);

void setup()
{
  regBank.setId(1);
  regBank.add(30001);
  regBank.add(30002);
  regBank.add(30003);
  regBank.add(30004);
  regBank.add(30005);
  regBank.add(30006);
  regBank.add(30007);
  regBank.add(30008);
  regBank.add(40001);
  slave._device = &regBank;  
  slave.setBaud(115200);  

  regBank.set(40001, 16000);


  pinMode(bigRotationSwitchPin, INPUT);
  bigRotationStepper.setRunMode(KEEP_SPEED);
  bigRotationStepper.setSpeedDeg(110);
  while(!digitalRead(bigRotationSwitchPin)) {    
    bigRotationStepper.tick();
	}
  bigRotationStepper.reset();

  // дальше например врубаем FOLLOW_POS
  bigRotationStepper.setRunMode(FOLLOW_POS);
  bigRotationStepper.setAcceleration(0);
  bigRotationStepper.setMaxSpeed(5000);
}

int t = 0;

void loop() 
{  
  long LFpos = static_cast<long>((bigRotationStepper.getCurrent()/2546.479089)*100.f);
  regBank.set(30001, (word) (LFpos / 32768) );
  regBank.set(30002, (word) (LFpos % 32768) );

  LFpos = static_cast<int16_t>(0.01f*100.f);
  regBank.set(30003, (word) (LFpos / 32768) );
  regBank.set(30004, (word) (LFpos % 32768) );

  LFpos = static_cast<long>(0.01f * 100.f);
  regBank.set(30005, (word) (LFpos / 32768) );
  regBank.set(30006, (word) (LFpos % 32768) );

  LFpos = static_cast<long>(0.01f * 100.f);
  regBank.set(30007, (word) (LFpos / 32768) );
  regBank.set(30008, (word) (LFpos % 32768) );

  /*static bool dir;
  if (!bigRotationStepper.tick()) {
    dir = !dir;
    bigRotationStepper.setTarget(dir ? -5*1600 : 5*1600);
  }*/

  //bigRotationStepper.setTarget(800);//static_cast<int16_t>(regBank.get(40001) - 16000));
  //motorLF->speed((int16_t)(regBank.get(40001)-32500)/100.f);

  if(!bigRotationStepper.tick())
  {
    bigRotationStepper.setTarget(static_cast<int16_t>(regBank.get(40001) - 16000)/100.f*2546.479089);
  }

  if(t>2)
  {
    
    slave.run();
    t = 0;
  }
  t++;
}

/*

#include <GyverStepper.h>

const int bigRotationSwitchPin = A0;
const float bigRotationGearK = 5;

// Шаговые двигатели
GStepper<STEPPER2WIRE> bigRotationStepper(3200, 8, 9);

void setup() 
{
  Serial.begin(115200);

  pinMode(bigRotationSwitchPin, INPUT);
  bigRotationStepper.setRunMode(KEEP_SPEED);
  bigRotationStepper.setSpeedDeg(100);
  while(!digitalRead(bigRotationSwitchPin)) {    
    bigRotationStepper.tick();
	}
  bigRotationStepper.reset();

  // дальше например врубаем FOLLOW_POS
  bigRotationStepper.setRunMode(FOLLOW_POS);
  bigRotationStepper.setAcceleration(5000);
  bigRotationStepper.setMaxSpeed(5000);
}

void loop() {
  static bool dir;
  if (!bigRotationStepper.tick()) {
    dir = !dir;
    bigRotationStepper.setTarget(dir ? -bigRotationGearK*1600 : bigRotationGearK*1600);
  }
}
*/