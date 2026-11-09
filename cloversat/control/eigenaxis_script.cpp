#include eigenaxis_gains.hpp
#include <iostream>
#include <vector>

class eigenaxis{

    private: 
        // define constant gain matrices
        double K;
        double D;

    public:
        eigenaxis(std::vector<std::vector> K_init)


     std::vector<double> operator*(std::vector<std::vector<double>> matrix, std::vector<double> vec)
        {   
            // assume inputs are 3x3 and 3x1
            first_index = matrix[0][0] * vec[0] + matrix[0][1] * vec[1] + matrix[0][2] * vec[2];
            second_index = matrix[1][0] * vec[0] + matrix[1][1] * vec[1] + matrix[1][2] * vec[2];
            thrid_index = matrix[2][0] * vec[0] + matrix[2][1] * vec[1] + matrix[2][2] * vec[2];
            
            return {first_index, second_index, third_index};
        }









};
