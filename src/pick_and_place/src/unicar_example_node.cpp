#include <unicar_demo/unicar_example.h>
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float32.h"


using namespace unicar_demo;
using namespace std;

float simulationTime = 0.0;

void simulationTimeCallback(const std_msgs::Float32& simTime){
  simulationTime = simTime.data;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  int steps =10;
  bool write_to_file = true;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("steps", steps, steps);
  pnh.param("write_to_file", write_to_file, write_to_file);

  ros::Subscriber subSimulationTime = nh.subscribe("/simulationTimeTopicName",1,simulationTimeCallback);
  cout << simulationTime << endl;

  PickAndPlaceExample example(nh, plotting, rviz, steps, write_to_file);
  example.run();

//  ros::spin();
}
