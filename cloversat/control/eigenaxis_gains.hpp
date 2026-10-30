#ifndef eigenaxis_gains.hpp
#define eigenaxis_gains.hpp

namespace constants
{
    // feedback gain matrix -- Kmatrix for derivatives 
    std::vector<std::vector<float>> K {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};

    // damping gain matrix - D
    std::vector<std::vector<float>> D {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};

    std::cout << "The matrix definition for D is " << D << std::endl;
}

#endif