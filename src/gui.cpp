
#include "vex.h"
#include "gui_images/rounded-rectangle.h"
#include "gui_images/big-red-rectangle.h"
#include "gui_images/logo.h"
#include "gui_images/full_logo.h"
#include "gui_images/checkmark.h"
#define AUTONS "Skills", "Solo Six WP", "Pos WP", "Solo WP Disrupt", "Solo Eight WP", "Goal Rush", "Goal + Wall Disrupt"
// NUMBER OF ELEMENTS HAVE TO BE SAME IN DESCRIPTION AND AUTON
////////////// if desc is too long, goes out of rectangle box

const char* auton[] = {AUTONS};
//
  //arrows on either side of the screen to increase/decrease the number for auton selector
  //logo
  //auton name + description 
  //port temp + connected
  //private:
int Button::*x,Button::*y,Button::*x2,Button::*y2;
//public:
Button::Button(int X1,int Y1,int X2,int Y2){
  this->x = X1; //point 1 is top left
  this->y = Y1;
  this->x2 = X2; //point 2 is bottom right
  this->y2 = Y2;
}

bool Button::is_pressed(){
  if((Brain.Screen.pressing()) && (Brain.Screen.xPosition() > Button::x) && (Brain.Screen.xPosition() < Button::x2) && (Brain.Screen.yPosition() > Button::y) && (Brain.Screen.yPosition() > Button::y2)){
    return true;
  } else {
    return false;
  }
};


//logo in driver only
bool autonGUI = false;
bool driverGUI = false;
bool auto_started = false;
bool driver_started = false;

void autonBackground(){
    Brain.Screen.clearScreen(transparent);
    Brain.Screen.drawImageFromBuffer(rounded_rectangle, 25,155,sizeof(rounded_rectangle));
    Brain.Screen.drawImageFromBuffer(big_red_rectangle, 25,25,sizeof(big_red_rectangle));
    Brain.Screen.drawImageFromBuffer(logo_white, 260,80,sizeof(logo_white));
}
//
//
int TeamColor = -1; // 1 for red and -1 for blue
int i = 1; // REMEMBER TO CHANGE THE TEAM COLOR TOO (EVEN FOR SKILLS)
//
//

Button auto_increment = Button(25,145,225,50);
Button auto_confirm = Button(250,145,300,50);
void print_auto(const char** autons){
  if(TeamColor == 1){
    Brain.Screen.printAt(32, 185, false, "%d. Red %s", i, autons[i]);
  } else if (TeamColor == -1){
    Brain.Screen.printAt(32, 185, false, "%d. Blue %s", i, autons[i]);
  } else if (i == 0){
    Brain.Screen.printAt(32, 185, false, "%d. %s", i, autons[i]);
  } 
  if(auto_increment.is_pressed()){
    if( i <(sizeof(auton)/4)-1){
        while(Brain.Screen.pressing()) {}
        i++;
    } else if (i ==(sizeof(auton)/4)-1){
        while(Brain.Screen.pressing()) {}
        TeamColor *= -1;
        i=0;
    } 
  Brain.Screen.clearScreen(transparent);
  Brain.Screen.drawImageFromBuffer(rounded_rectangle, 25,155,sizeof(rounded_rectangle));
  Brain.Screen.drawImageFromBuffer(big_red_rectangle, 25,25,sizeof(big_red_rectangle));
  Brain.Screen.drawImageFromBuffer(logo_white, 260,80,sizeof(logo_white));
  }
}

double get_motor_temp(motor Temp){
  if (!Temp.installed()){
    return 0;
  } else {
    return Temp.temperature(celsius);
  }
}

void motor_temps(){
  double leftTemp[]= {get_motor_temp(Left1), get_motor_temp(Left2), get_motor_temp(Left3)};
  double rightTemp[]= {get_motor_temp(Right1), get_motor_temp(Right2), get_motor_temp(Right3)};


  Brain.Screen.printAt(32,50, false, "Left: %2.0f, %2.0f, %2.0f", leftTemp[0], leftTemp[1], leftTemp[2]);
  Brain.Screen.printAt(32,70, false, "Right: %2.0f, %2.0f, %2.0f", rightTemp[0], rightTemp[1], rightTemp[2]);
  Brain.Screen.printAt(32,90, false, "Intake: %2.0f", get_motor_temp(Intake));
  Brain.Screen.printAt(32,110, false, "Lift: %2.0f", get_motor_temp(Wall));
}


void confirm_auto(){
  if (auto_confirm.is_pressed()){
    auto_started = true;
  }
}

void run_auto(){
    switch(i){  
    case 0:
      TeamColor = 1;
      skills();
      break;
    case 1:  // 6 SOLO WP
      if(TeamColor == 1){
        red_solo_six_wp();
      } else if (TeamColor == -1) {
        blue_solo_six_wp();
      }
      break;
    case 2: // POS WP
      if(TeamColor == 1){
        red_pos_wp();
      } else if (TeamColor == -1) {
        blue_pos_wp();
      }
      break;
    case 3: // SOLO WP DISRUPT
      if(TeamColor == 1){
        red_solo_wp_disrupt();
      } else if (TeamColor == -1) {
        blue_solo_wp_disrupt();
      }
      break;
    case 4: // SOLO WP DISRUPT
      if(TeamColor == 1){
        red_solo_seven_wp();
      } else if (TeamColor == -1) {
        blue_solo_seven_wp();
      }
      break;
    case 5: // GOAL RUSH
      if(TeamColor == 1){
        red_goal_rush();
      } else if (TeamColor == -1) {
        blue_goal_rush();
      }
      break;
    case 6: // GOAL RUSH WALL DISRUPT  
      if(TeamColor == 1){
        red_goal_rush_wall_disrupt();
      } else if (TeamColor == -1) {
        blue_goal_rush_wall_disrupt();
      }
      break;
}
}

int gui(){
    task::sleep(3000);
    Brain.Screen.clearScreen(transparent);
    autonBackground();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(mono20);

    print_auto(auton);
    motor_temps();
    while(true){
      print_auto(auton);
      motor_temps();
      task::sleep(20);
    }
}