/**
 * @file pick_and_place_example_node.cpp
 * @brief Pick and place example node
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
