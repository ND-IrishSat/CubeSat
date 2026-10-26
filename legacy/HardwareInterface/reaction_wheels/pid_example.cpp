#include <iostream>


class pid_controller
{
    private:
        double kd;
        double kp;
        double ki;

        double integral;

    public:
        pid_controller(double kd, double kp, double ki)
        :   kd(kd), kp(kp), ki(ki), integral(0.0) {}



        double func(double measurement, double target, double dt)
        {
            double error = measurement - target;

            double prop = error * kp;
            double derive = (measurement-target) / dt * kd;
            integral += (measurement - target) *dt

            return measurement + prop + derive + integral

        }


};

// simulation

int main()
{

    double dt = 0.5;
    double sim_time = 100; //s
    double curr_time = 0;
    double measurement =  10;
    double target = 80;

    pid_controller object = pid_controller(.10, 1.0, 10)
    
    while curr_time < sim_time
    {
        double error = measurement - target;

        measurement = object.update(measurement, target, dt);

        if (measurement - target < 1.0e-5)
        {
            break;
        }

    }

    std::cout << 'After sim, new position is: ' << measurement << std::endl;
}
