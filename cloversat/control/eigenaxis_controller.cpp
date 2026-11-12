#include <iostream>
#include <vector>
#include eigenaxis_gains.hpp

using vec = std::vector<float> 
using matrix = std::vector<std::vector<float>>

template <typename T>

T operator*(const T& m, const T& v)
{   
    // assume inputs are 3x3 and 3x1
    float first_row = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
    float second_row = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
    float thrid_row = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
    
    return {first_row, second_row, third_row};
}

T operator*(const T& m, const T& p)
{
    // compute 0-th row of the resulting matrix
    id_0_0 = m[0][0] * p[0][0] + m[0][1] * p[1][0] + m[0][2] * p[2][0];
    id_0_1 = m[0][0] * p[0][1] + m[0][1] * p[1][1] + m[0][2] * p[2][1];
    id_0_2 = m[0][0] * p[0][2] + m[0][1] * p[1][2] + m[0][2] * p[2][2];

    // 1-st row
    id_1_0 = m[1][0] * p[0][0] + m[1][1] * p[1][0] + m[1][2] * p[2][0];
    id_1_1 = m[1][0] * p[0][1] + m[1][1] * p[1][1] + m[1][2] * p[2][1];
    id_1_2 = m[1][0] * p[0][2] + m[1][1] * p[1][2] + m[1][2] * p[2][2];
    
    // 2-nd row
    id_2_0 = m[2][0] * p[0][0] + m[2][1] * p[1][0] + m[2][2] * p[2][0];
    id_2_1 = m[2][0] * p[0][1] + m[2][1] * p[1][1] + m[2][2] * p[2][1];
    id_2_2 = m[2][0] * p[0][2] + m[2][1] * p[1][2] + m[2][2] * p[2][2];

    matrix result = {{id_0_0, id_0_1, id_0_2},
                     {id_1_0, id_1_1, id_1_2},
                     {id_2_0, id_2_1, id_2_2}};

    return result;
}


class eigenaxis
{
    private:
        vec omega;
        matrix skew;
        matrix  D; // constant gain matrix to multiply angular velocity
        matrix K; // constant gain matrix to multiply the reference quaternion

    public:
        eigenaxis(matrix D_input, matrix K_input)
        : omega(omega), skew({{0,0,0},{0,0,0},{0,0,0}}), D(D_input), K(K_input) {}

        double update_eigenaxis(std::vector<float> omega)
        {
            this.omega = sat.body.angular_velocity // ????? for now, assume we directly get this from the IMU sensors
            this.skew = {{0, -omega[2], omega[1]}, 
                         {omega[2], 0, -omega[0]},
                         {-omega[1], omega[0], 0}};
        
            // J is matrix
            float control_signal = -skew * sat.body.J * omega - D * omega - K * q_reference;

        }
                
        vec quaternion_calculator(vec current_q, vec commanded_q)
        {   
            // define for ease of scripting commanded matrix
            vec q = commanded_q;

            // compute matrix in formula for error quaternion
            matrix commanded_matrix = {{q[3], q[2], -q[1], -q[0]},
                                       {-q[2], q[3], q[0], -q[1]},
                                       {q[1], -q[0], q[3], -q[2]},
                                       {q[0], q[1], q[2], q[3]}};

            vec error_quaternion = eiganaxis::operator*(commanded_matrix, current_q)
        }
};

// 1-D test simulation run
int main()
{

    // testing on operator overloads
    matrix D = {{0, 1, 1},
                {1,3,4},
                {4,1,2}};

                 
    matrix K = {{5, 2, 1},
                {4,3,3},
                {4,6,1}};


    test1d = eigenaxis(D, K);
    vec curr_quat_attitude = {0,0,0,0};
    vec des_quat_attitude = {}; // need conversion for this euler angle to quaternion 
    
    vec state = {theta, omega}
    double dt = 0.05;
    double sim_time = 500;

    // double control_signal = eigenaxis::operator*(eigenaxis::operator*(skew, J), omega)
    //                         + eigenaxis::operator*(eigenaxis::operator*(D_matrix, omega))
    //                         - eigenaxis::operator*(J_matrix, q_error);

    for (double time = 0.0; time < sim_time; t++)
    {
        control_signal = test1d.update_eigenaxis(omega)
        vec ang_accel = skew * J * omega + control_signal;


        test1d.quaternion_calculator(curr_q, comm_q)


        if (curr_quat_attitude - des_quat_attitude)
        {
            // throw std::runtime_error("The target has been reached");
            std::cout << "The desired quaternion attitude has been reached" << std::endl;
            break;
        }

    }
    
    eigenaxis_test::operator*(t1, t2);

    std::cout << "Desired quaternion reference attitude " << des_quat_attitude << std::endl;
    std::cout << "Final quaternion representation is " << curr_quat_attitude << std::endl;

}
