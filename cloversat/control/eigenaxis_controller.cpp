// this code is a draft for the eigenaxis controller
#include <iostream>
#include eigenaxis_gains.hpp

float eigenaxis_calculation()
{

    // includes control signal calculation
}

float quaternion_calculator()
{

    curr_quaternion = sat.body_q
    target  = // How to pass this in?


    float error_quaternion[4] = {0.0,0.0,0.0,0.0};

    // main for loop for integrating the error quaternion
    for (int i = 0; i <= std::length(curr_quaternion))
    {
        for (int j = 0; i <= std::length(curr_quaternion))
        {
            error_quaternion[i] += quat_matrix[i][j] * curr_quaternion[j];
        }
    }

}
double main()
{
    // returns the 
    if (sat.gyro.isWorking == true)
    {
        float control signal;
        Vec3 


    }



}




