#include "ure_controllers.hpp"
int main(int argc, char const *argv[])
{
    UreController controller;
    // std::vector<double> v{1.3, 1.3, 1.3, 1.3, 1.3, 1.3};
    std::vector<double> v{0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 100000; i++)
    {
        v[0] += - 1.57 / 5000.0;
        v[1] += - 1.57 / 5000.0;
        v[2] += - 1.57 / 5000.0;
        controller.sendDegreeToJoint(v);
        controller.stepOne();
    }
    std::cout << "end" << std::endl;
    return 0;
}
