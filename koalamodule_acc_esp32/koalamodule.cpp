//-----------------------------------------------------------------
//  Koalamodule 电控教学案例：Arduino 步进电机加减速库
//  版权所有：杭州赋形智能科技有限公司&喵星考拉
//  支持ESP32 + CNC ShieldV3 控制步进电机
//  支持步进电机1500转运行，在每毫米100脉冲的直线模组上实现600mm/s速度
//-----------------------------------------------------------------

#include "koalamodule.h"
  //快速开平方倒数算法
  float Q_rsqrt(float number){
  long i;
  float y = number;
  i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y*(1.5-(0.5*number*y*y));
  return y;
  }
//类外定义：作用域
  //构造函数
  koalamodule::koalamodule(){};
  //析构函数
  koalamodule::~koalamodule(){};
  //连接步进电机控制引脚
  void koalamodule::attach(short input_EN,short input_STEP, short input_DIR){
    m_EN = input_EN;
    pinMode(m_EN,OUTPUT);
    digitalWrite(m_EN,LOW);
    attach(input_STEP,input_DIR);
  };
  //连接步进电机控制引脚
  void koalamodule::attach(short input_STEP, short input_DIR){
    m_DIR = input_DIR;
    m_STEP = input_STEP;
    pinMode(m_DIR,OUTPUT);
    pinMode(m_STEP,OUTPUT);
  };
  //连接零点限位开关，默认为低电平触发，单片机内部上拉
  void koalamodule::attachSwitch(short input_SW){
    m_SW = input_SW;
    pinMode(m_SW,INPUT_PULLUP);
  };
  //往前走的处理函数
  void koalamodule::forward(unsigned long pulse,unsigned int feedrate){
    digitalWrite(m_DIR,HIGH^INVERTDIRECTION);
    setPulse(pulse,feedrate);
    AbsolutePulse+=pulse;
  };
  //往后走的处理函数
  void koalamodule::backward(unsigned long pulse,unsigned int feedrate){
    digitalWrite(m_DIR,LOW^INVERTDIRECTION);
    setPulse(pulse,feedrate);
    AbsolutePulse-=pulse;
  }; 
  //G28回原点函数
  void koalamodule::toOrigin(){
    forward(200,1500);
    backward(1,1500);
    while(digitalRead(m_SW)==HIGH){
      onePulse(50);
    };
    delay(500);
    forward(2000,1500);
    backward(1,1500);
    while(digitalRead(m_SW)==HIGH){
     onePulse(200);
    };
    AbsolutePulse = 0;
    AxisOffset = 0;
    delay(500);

  };
  //直线运动，相当于G00和G01，函数从这里做入口
  void koalamodule::toPosition(float pos,unsigned int feedrate){
    //计算绝对脉冲数
    unsigned long topulse = pos*STEPSPERMM;
    long requirepulse = topulse - AbsolutePulse + AxisOffset;
    if(requirepulse>0)forward(requirepulse,feedrate);
    else if (requirepulse<0)backward(-requirepulse,feedrate);
    
  };
  //设置当前位置，相当于G92
  void koalamodule::setPosition(float pos){///假设现在的绝对脉冲是1200，输入pos是-20mm
    AxisOffset = AbsolutePulse - pos*STEPSPERMM;

  };
  //翻转电机方向，用于电机回零方向反了的情况
  void koalamodule::invertdirection(){
    INVERTDIRECTION = !INVERTDIRECTION;
  };
  //核心运动控制算法：根据给定速度和加速度发送脉冲
  void koalamodule::setPulse(unsigned long pulse, unsigned int feedrate){
    unsigned long pulse_acc,pulse_dec,pulse_cru,pulse_acc2,cntL,cntH;
    float u = 500000.0*Q_rsqrt(2.0*STEPSPERMM);//加减速的中间数u
    unsigned int dly_acc,dly_cru,delayL,delayH,j;
    static unsigned int acceltable1[100];//第一阶段加速存储delay的表
    static unsigned int acceltable2[100];//第二阶段加速存储delay个数的表
    static unsigned int old_accel=0;
    char i=1;
    //如果加速度发生了改变，需要重算加速度表
    if(old_accel != m_Acceleration){
      //计算第一阶段加速表
      for(int i=1;i<=100;i++){
          acceltable1[i-1] =u*1.0*Q_rsqrt((float)i*m_Acceleration);
      }
      delayL = acceltable1[99]-1;//获取前面的最后一个延时值
      cntL=100;//这个delay值对应第100个脉冲      
      //计算第二阶段加速表
      do{
        delayH = delayL-1;//少1微秒
        cntH = u*1.0*u/delayH/delayH/m_Acceleration;//计算delayH在第几个出现
        acceltable2[i-1] =cntH-cntL;//两者作差得到当前delay值重复次数
        delayL = delayH;//循环赋值
        cntL=cntH;
        i++;
      }while(delayL>0);
      old_accel = m_Acceleration;
      
    }
    //计算加速脉冲数
    pulse_acc = feedrate*1.0/7200*feedrate/m_Acceleration*STEPSPERMM;
    //给加速、匀速、减速脉冲数赋值，如果总脉冲小于两倍加速脉冲则重算脉冲
    if((pulse_acc<<1)>=pulse){
      pulse_acc = pulse>>1;
      pulse_cru = pulse&1;
    }
    else{//脉冲够多，计算加速、匀速、减速脉冲数
      pulse_cru = pulse-(pulse_acc<<1);
    }
    //完成脉冲数计算   
    //计算匀速区间的delay值
    dly_cru = 30000000.0/STEPSPERMM/feedrate;
    //计算第二阶段加速脉冲数
    pulse_acc2= pulse_acc - 100;
    //获取第一个表结尾的脉冲数
    dly_acc = acceltable1[99]-1;
    j=0;
    //计算完成，接下来开始发脉冲
    //如果小于100个脉冲就直接读第一个表  
    if(pulse_acc<=100){
      for(int i=1;i<=pulse_acc;i++){
        digitalWrite(m_STEP,LOW);
        delayMicroseconds(acceltable1[i-1]);
        digitalWrite(m_STEP,HIGH);
        delayMicroseconds(acceltable1[i-1]);
      }
    }
    else{//大于100个脉冲
      //先发完前面100个脉冲
      for(int i=1;i<=100;i++){
        digitalWrite(m_STEP,LOW);
        delayMicroseconds(acceltable1[i-1]);
        digitalWrite(m_STEP,HIGH);
        delayMicroseconds(acceltable1[i-1]);
      }
      //读第二个表计数发脉冲
      unsigned int targetpulse = acceltable2[0];
      while(pulse_acc2>targetpulse)
      { 
        pulse_acc2-=targetpulse;
        for(int k=1;k<=targetpulse;k++){
          digitalWrite(m_STEP,LOW);
          delayMicroseconds(dly_acc);
          digitalWrite(m_STEP,HIGH);
          delayMicroseconds(dly_acc);
        }        
        dly_acc--;
        j++;
        targetpulse = acceltable2[j];
      }
      for(int k=1;k<=pulse_acc2;k++){
         digitalWrite(m_STEP,LOW);
          delayMicroseconds(dly_acc);
          digitalWrite(m_STEP,HIGH);
          delayMicroseconds(dly_acc);
      }
      //加速算法完成后，此处记录了当前的最终延时dly，这个dly的脉冲数pulseacc2，此时的表格情况acceltable
    }//结束pulseacc>100
    //匀速
    if(pulse_cru){
      for(int i=1;i<=pulse_cru;i++){
        digitalWrite(m_STEP,LOW);
        delayMicroseconds(dly_cru);
        digitalWrite(m_STEP,HIGH);
        delayMicroseconds(dly_cru);
      }
    }
    //减速：跟加速反过来
    if(pulse_acc<=100){
        for(int i=pulse_acc;i>=1;i--){
         digitalWrite(m_STEP,LOW);
        delayMicroseconds(acceltable1[i-1]);
        digitalWrite(m_STEP,HIGH);
        delayMicroseconds(acceltable1[i-1]);
      }
    }
    else{
      for(int k=1;k<=pulse_acc2;k++){
          digitalWrite(m_STEP,LOW);
          delayMicroseconds(dly_acc);
          digitalWrite(m_STEP,HIGH);
          delayMicroseconds(dly_acc);
      }
     
      dly_acc++;
      while(j>0){
        for(int k=1;k<=acceltable2[j-1];k++){
           digitalWrite(m_STEP,LOW);
          delayMicroseconds(dly_acc);
          digitalWrite(m_STEP,HIGH);
          delayMicroseconds(dly_acc);
        }
        dly_acc++;
        j--;
      }
      for(unsigned int i=100;i>=1;i--){
        //digitalWrite(m_STEP,!digitalRead(m_STEP));
        digitalWrite(m_STEP,LOW);
        delayMicroseconds(acceltable1[i-1]);
        digitalWrite(m_STEP,HIGH);
        delayMicroseconds(acceltable1[i-1]);
      }
    }   
  }
  //设置加速度，最大65535，完全够用
  void koalamodule::setAcceleration(unsigned int accel){
    if(accel<100)accel=100;
    if(accel>10000)accel=10000;
    m_Acceleration = accel;
  }
  //onePules这个函数调用还是会浪费时间，只在回零和初始化使用
  inline void koalamodule::onePulse(unsigned int m_delay){
    digitalWrite(m_STEP,LOW);
    delayMicroseconds(m_delay);
    digitalWrite(m_STEP,HIGH);
    delayMicroseconds(m_delay);  
  }
  //设置每毫米脉冲数，便于不同的直线模组调用
  void koalamodule::setStepspermm(unsigned int kmp){
    STEPSPERMM=kmp;

 }










