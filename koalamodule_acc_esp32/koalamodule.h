#ifndef KOALAMODULE_H
#define KOALAMODULE_H
#include <Arduino.h>
class koalamodule{
  public:
  koalamodule();
  ~koalamodule();
  void attach(short input_EN,short input_STEP, short input_DIR);
  void attach(short input_STEP, short input_DIR);
  void attachSwitch(short input_SW);
  void forward(unsigned long pulse,unsigned int feedrate);
  void backward(unsigned long pulse,unsigned int feedrate);
  void setPulse(unsigned long pulse, unsigned int feedrate);
  void toOrigin();
  void toPosition(float pos,unsigned int feedrate);
  void setPosition(float pos);
  void invertdirection();
  void setAcceleration(unsigned int accel);
  inline void onePulse(unsigned int m_delay);
  void setStepspermm(unsigned int kmp);
  private:
  short m_EN;
  short m_STEP;
  short m_DIR;
  short m_SW;
  unsigned long AbsolutePulse;
  unsigned long AxisOffset;
  unsigned int STEPSPERMM = 100;
  volatile unsigned int m_Acceleration = 10000;
  bool INVERTDIRECTION = 0;
};

#endif