#include "dataPrepare.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_localization");

    ROS_INFO("[%s] Node initialization.", ros::this_node::getName().data());

    global_localization::dataPrepare node(false);
    if(!node.configure()){
        return 1;
    }
    ros::spin();

    return 0;
}
