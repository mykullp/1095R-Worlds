#pragma once
#include "vex.h"

class Distance
{
    private:
        int tracker_port;
        double x;
        double y;
    public: 
        distance Sensor;

        Distance(int tracker_port, double x, double y);

        double get_distance();
        
        std::vector<double> get_center_vector();

        bool is_installed();
};

class Distance_Track
{
private: 

public:
    Distance Distance_Left;
    Distance Distance_Right;
    Distance Distance_Back;
    inertial Gyro;

    Distance_Track(Distance Distance_Left, Distance Distance_Right, Distance Distanace_Back, int gyro_port);

    float X_position;
    float Y_position;
    float orientation_deg;
    
    void update_position(float Left_position, float Right_position, float Back_position, float orientation_deg);
};