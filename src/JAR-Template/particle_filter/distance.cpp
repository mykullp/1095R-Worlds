#include "vex.h"

Distance::Distance(int tracker_port, double x, double y) :
    Sensor(distance((tracker_port))),
    x(x),
    y(y)
{};

double Distance::get_distance(){
    return Sensor.objectDistance(inches);
}

std::vector<double> Distance::get_center_vector(){
    return {hypot(x,y), atan2(y,x)};
}

bool Distance::is_installed(){
    return Sensor.installed();
}

Distance_Track::Distance_Track(Distance Distance_Left, Distance Distance_Right, Distance Distance_Back, int gyro_port) :
    Distance_Left(Distance_Left),
    Distance_Right(Distance_Right),
    Distance_Back(Distance_Back)
    Gyro(vex::inertial(gyro_port))
{};

void Distance_Track::update_position(float Left_position, float Right_position, float Back_position, float orientation_deg){
    std::vector<double> left_vector = Distance_Left.get_center_vector();
}