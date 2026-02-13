using namespace vex;
#pragma once

extern int gui();

extern bool auto_started;
extern bool driver_started;

extern void run_auto();
extern int i;

class Button {
  private:
  int x,y,x2,y2;
  public:
  Button(int X1,int Y1,int X2,int Y2);
  bool is_pressed();
};


