#include "initialize.hpp"
#include "robot_config.hpp"
#include "data_structure_convert.hpp"

using namespace KDL;
using namespace RokaeApi;

int main(){
    RokaeApi::Model::MechUnitType robot_type = Model::MechUnitType::SR3_C;
    InitRobot init_robot(robot_type);
    int res = init_robot.CreateModels();
    std::cout<<"res = "<<res<<std::endl;
    return 0;

}