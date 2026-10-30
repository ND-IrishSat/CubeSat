// this code is a draft for the eigenaxis controller
#include <iostream>
#include <vector>
#include eigenaxis_gains.hpp

using vec = std::vector<float> 
using matrix = std::vector<std::vector<float>>

class eigenaxis_test
{
    private:
        std::vector<float> omega;
        std::vector<std::vector<float>> skew;
        double D; // constant gain matrix to multiply angular velocity
        double K; // constant gain matrix to multiply the reference quaternion

    public:
        eigenaxis_test(std::vector<float> omega)
        : omega(omega), skew({{0,0,0},{0,0,0},{0,0,0}}), D(constants::D), K(constants::K) {}

        std::vector<double> operator*(std::vector<std::vector<double>> matrix, std::vector<double> vec)
        {   
            // assume inputs are 3x3 and 3x1
            first_index = matrix[0][0] * vec[0] + matrix[0][1] * vec[1] + matrix[0][2] * vec[2];
            second_index = matrix[1][0] * vec[0] + matrix[1][1] * vec[1] + matrix[1][2] * vec[2];
            thrid_index = matrix[2][0] * vec[0] + matrix[2][1] * vec[1] + matrix[2][2] * vec[2];
            
            return {first_index, second_index, third_index};
        }

        double update_eigenaxis()
        {
            omega = sat.body.angular_velocity // ?????
            skew = {{0, -omega[2], omega[1]}, 
                    {omega[2], 0, -omega[0]},
                    {-omega[1], omega[0], 0}};


            control_signal = -skew * sat.body.J * omega - D * omega - K * q_reference;

        }
};

float eigenaxis_calculation()
{
    // includes control torque signal calculation
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




