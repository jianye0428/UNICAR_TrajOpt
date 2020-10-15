/**
 * @file pick_and_place_example.h
 * @brief An example of a robot picking up a box and placing it on a shelf.
 *
 * @author Mathew Powelson
 * @date July 22, 2018
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
#ifndef TESSERACT_ROS_EXAMPLES_PICK_AND_PLACE_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_PICK_AND_PLACE_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_common/types.h>
#include <unicar_demo/example.h>

namespace unicar_demo
{
/**
 * @brief An example of a robot picking up a box and placing it on a shelf leveraging
 * tesseract and trajopt to generate the motion trajectory.
 */
class PickAndPlaceExample : public Example
{
public:
  PickAndPlaceExample(const ros::NodeHandle& nh, bool plotting, bool rviz, int steps, bool write_to_file)
        : Example(plotting, rviz), nh_(nh), steps_(steps), write_to_file_(write_to_file), env_current_revision_(0)
  {
      locator_ = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  }
  ~PickAndPlaceExample() override = default;
  PickAndPlaceExample(const PickAndPlaceExample&) = default;
  PickAndPlaceExample& operator=(const PickAndPlaceExample&) = default;
  PickAndPlaceExample(PickAndPlaceExample&&) = default;
  PickAndPlaceExample& operator=(PickAndPlaceExample&&) = default;

  bool run() override;
//  bool box_set() override;//20200221

private:
  ros::NodeHandle nh_;
  int steps_;
  bool write_to_file_;
  int box_number_;
  //20200219_novisual
  int env_current_revision_;
  tesseract_scene_graph::ResourceLocator::Ptr locator_;
  std::unordered_map<std::string, std::unordered_map<std::string, double>> saved_positions_;

  std::shared_ptr<trajopt::ProblemConstructionInfo> cppMethod(const std::string& start, const std::string& finish);
  void addBox();
  std::unordered_map<std::string, std::unordered_map<std::string, double>> getPredefinedPosition();
  std::vector<double> getPositionVector(const tesseract_kinematics::ForwardKinematics::ConstPtr& kin,
                                        const std::unordered_map<std::string, double>& pos);
  Eigen::VectorXd getPositionVectorXd(const tesseract_kinematics::ForwardKinematics::ConstPtr& kin,
                                      const std::unordered_map<std::string, double>& pos);
};

}  // namespace unicar_demo

#endif  // TESSERACT_ROS_EXAMPLES_PICK_AND_PLACE_EXAMPLE_H