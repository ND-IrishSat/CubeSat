// this code is a draft for the eigenaxis controller
#include <iostream>
#include <vector>
#include eigenaxis_gains.hpp

using vec = std::vector<float> 
using matrix = std::vector<std::vector<float>>

float eigenaxis_calculation()
{

    // includes control signal calculation
}

float quaternion_calculator(vec curr_quaternion, vec comm_quaternion )
{

    matrix c_matrix = {{q[3], q[2], -q[1], -q[0]},
                                                 {-q[2], q[3], q[0], -q[1]},
                                                 {q[1], -q[0], q[3], -q[2]},
                                                 {q[0], q[1], q[2], q[3]}};

    vec error_quaternion = {0.0,0.0,0.0,0.0};

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
        veccurr_quaternion = self.state.quaternion; // ??? how is self defined in cloversat
        vec err_quaternion = quaternion_calculator(curr_quaternion, comm_quaternion);

        // skew symmetric matrix of angular velocities
        matrix omega_matrix = {{0, -self.w[2], self.w[1]},
                                                         {self.w[2], 0, -self.w[0]},
                                                         {-self.w[1], self.w[0], 0}};

        control_torque_vec = //skew * inertia_tensor * ang_velocity - D(gain matrix)* ang_velocity - K(gain matrix) * q_err



    }



}




