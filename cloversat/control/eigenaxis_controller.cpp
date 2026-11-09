#include <iostream>
#include <vector>
#include eigenaxis_gains.hpp

using vec = std::vector<float> 
using matrix = std::vector<std::vector<float>>

class eigenaxis
{
    private:
        vec omega;
        matrix skew;
        matrix  D; // constant gain matrix to multiply angular velocity
        matrix K; // constant gain matrix to multiply the reference quaternion

    public:
        eigenaxis_test(matrix D_input, matrix K_input)
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

eigenaxis operator*(const matrix& m, const vector& v)
{   
    // assume inputs are 3x3 and 3x1
    float first_row = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
    float second_row = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
    float thrid_row = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
    
    return {first_row, second_row, third_row};
}

eigenaxis operator*(const matrix& m, const matrix& p)
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

int main(){

    // testing on operator overloads
    matrix t1 = {{0, 1, 1},
                 {1,3,4},
                 {4,1,2}};

                 
    matrix t2 = {{5, 2, 1},
                 {4,3,3},
                 {4,6,1}};

    eigenaxis_test::operator*(t1, t2);

    
}
