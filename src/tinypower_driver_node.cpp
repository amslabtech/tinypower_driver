#include "tinypower_driver/tinypower_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tinypower_driver");
    tinypower_driver::TinypowerDriver td;
    td.process();
    return 0;
}
