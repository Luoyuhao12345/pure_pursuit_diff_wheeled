#include "my_pure_pursuit.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pure_pursuit");
  new MyPurePursuit();
  ros::spin();
  return 0;
}

