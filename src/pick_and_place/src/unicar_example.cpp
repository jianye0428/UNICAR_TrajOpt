#include <tesseract_common/macros.h>
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <unicar_demo/unicar_example.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <tesseract_geometry/mesh_parser.h>
#include <pick_and_place/JointTrajectoryPointToSim.h>
#include <pick_and_place/JointTrajectoryToSim.h>
#include <pick_and_place/isPosition.h>
#include <std_msgs/Bool.h>
#include <ctime>
#include <chrono>


using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace std;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";
const bool ENABLE_TIME_COST = false;
const bool ENABLE_VELOCITY_COST = false;
const double OFFSET = 0.005;
const double GREIFER_OFFSET = 0.2;

namespace unicar_demo
{
bool PickAndPlaceExample::run()
{
  // Set Log Level
  util::gLogLevel = util::LevelWarn;
  //util::gLogLevel = util::LevelError;

  /////////////
  /// SETUP ///
  /////////////

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string, box_parent_link;
  double box_side, box_x, box_y, box_z;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  nh_.getParam("box_side", box_side);
  nh_.getParam("box_x", box_x);
  nh_.getParam("box_y", box_y);
  nh_.getParam("box_z", box_z);
  nh_.getParam("box_parent_link", box_parent_link);

  // Initialize the environment
  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter =
      std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment());

  if (rviz_)
  {
    // These are used to keep visualization updated
    modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
    get_env_changes_rviz_ =
        nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);

    // Check RViz to make sure nothing has changed
    if (!checkRviz())
      return false;
  }
  //sleep(20);
  // Set the initial state of the robot
  std::unordered_map<std::string, double> joint_states;
  joint_states["drehkranz_joint"] = 0.0;
  joint_states["shoulder_pan_joint"] = 0.0;
  joint_states["shoulder_lift_joint"] = 0.0;
  joint_states["elbow_joint"] = 0.0;
  joint_states["wrist_1_joint"] = 0.0;
  joint_states["wrist_2_joint"] = 0.0;
  joint_states["wrist_3_joint"] = 0.0;
  tesseract_->getEnvironment()->setState(joint_states);

  const std::string link_box_name = "box";
  Link link_box(link_box_name);

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(box_side, box_side, box_side);
  link_box.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_box.collision.push_back(collision);

  Joint joint_box("joint_box");
  joint_box.parent_link_name = box_parent_link;
  joint_box.child_link_name = link_box_name;
  joint_box.type = JointType::FIXED;
  joint_box.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(box_x, box_y, (box_z) + OFFSET);
  tesseract_->getEnvironment()->addLink(std::move(link_box), std::move(joint_box));

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(0))
      return false;
  }


  ////////////
  /// PICK ///
  ////////////

  if (rviz_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  // Choose the manipulator and end effector link
  std::string manip = "Manipulator";
  std::string end_effector = "ee_link";
  std::string greifer_link = "greifer_link";
  std::string wrist_3_link = "wrist_3_link"; 

  // Define the final pose (on top of the box)
  //pick_final_pose
  Eigen::Isometry3d final_pose;
  Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
  final_pose.linear() = orientation.matrix();
  final_pose.translation() += Eigen::Vector3d(box_x, box_y, box_z + box_side/2 +  GREIFER_OFFSET); //coordinate to the world

  // Define the approach pose
  Eigen::Isometry3d approach_pose = final_pose;
  approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.1);

  // Create the problem construction info
  trajopt::ProblemConstructionInfo pci(tesseract_);

  pci.basic_info.n_steps = steps_ * 3;
  pci.basic_info.manip = manip;
  pci.basic_info.dt_lower_lim = 2;    // 1/most time
  pci.basic_info.dt_upper_lim = 100;  // 1/least time
  pci.basic_info.start_fixed = true;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  pci.init_info.type = trajopt::InitInfo::STATIONARY;
  pci.init_info.dt = 0.5;

  // Add a collision cost
  {
    auto collision = std::make_shared<trajopt::CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
    collision->evaluator_type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
    collision->first_step = 1;
    collision->last_step = pci.basic_info.n_steps - 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.005, 50);
    pci.cost_infos.push_back(collision);
  }

  // Add a velocity cost without time to penalize paths that are longer
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->term_type = trajopt::TT_COST;
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cost";
    pci.cost_infos.push_back(jv);
  }

  // Add a velocity cnt with time to insure that robot dynamics are obeyed
  if (ENABLE_VELOCITY_COST)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);

    // Taken from ur10 documentation (radians/s) and scaled by 0.8
    std::vector<double> vel_lower_lim{ 1.71 * -0.8, 2.1 * -0.8,  2.1 * -0.8,
                                      3.14 * -0.8, 3.14 * -0.8, 3.14 * -0.8,
                                      3.14 * -0.8};
    std::vector<double> vel_upper_lim{ 1.71 * 0.8, 2.1 * 0.8,  2.1 * 0.8,
                                      3.14 * 0.8, 3.14 * 0.8, 3.14 * 0.8,
                                      3.14 * 0.8 };

    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 50.0);
    jv->lower_tols = vel_lower_lim;
    jv->upper_tols = vel_upper_lim;
    jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cnt";
    pci.cnt_infos.push_back(jv);
  }

  // Add cartesian pose cnt at the approach point
  {
    Eigen::Quaterniond rotation(approach_pose.linear());
    auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = steps_;
    pose_constraint->xyz = approach_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(steps_);
    pci.cnt_infos.push_back(pose_constraint);
  }

  // Add cartesian pose cnt at the final point
  {
    Eigen::Quaterniond rotation(final_pose.linear());
    auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = 2 * steps_ - 1;
    pose_constraint->xyz = final_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_extra_" + std::to_string(2 * steps_ - 1);
    pci.cnt_infos.push_back(pose_constraint);
  }

  // Add a cost on the total time to complete the pick
  if (ENABLE_TIME_COST)
  {
    std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
    time_cost->name = "time_cost";
    time_cost->coeff = 5.0;
    time_cost->limit = 0.0;
    time_cost->term_type = trajopt::TT_COST;
    pci.cost_infos.push_back(time_cost);
  }

  // Create the pick problem
  trajopt::TrajOptProb::Ptr pick_prob = ConstructProblem(pci);

  // Set the optimization parameters (Most are being left as defaults)
  tesseract_motion_planners::TrajOptPlannerConfig config(pick_prob);
  config.params.max_iter = 200;

  // Create Plot Callback
  if (plotting_)
  {
    config.callbacks.push_back(PlotCallback(*pick_prob, plotter));
  }

  // Create file write callback discarding any of the file's current contents
  std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
  if (write_to_file_)
  {
    std::string path = ros::package::getPath("pick_and_place") + "/config/file_output_pick.csv";
    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, pick_prob));
  }

  // Create the planner and the responses that will store the results
  tesseract_motion_planners::TrajOptMotionPlanner planner;
  tesseract_motion_planners::PlannerResponse planning_response;
  tesseract_motion_planners::PlannerResponse planning_response_place;

  // Set Planner Configuration
  planner.setConfiguration(std::make_shared<tesseract_motion_planners::TrajOptPlannerConfig>(config));

  // Solve problem. Results are stored in the response

  auto start = std::chrono::system_clock::now();
  planner.solve(planning_response);
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << "finished computation at " << std::ctime(&end_time)
            << "elapsed time: " << elapsed_seconds.count() << "s\n";

  if (write_to_file_)
    stream_ptr->close();

  // Plot the resulting trajectory
  //if (plotting_)
    plotter->plotTrajectory(pick_prob->GetKin()->getJointNames(),
                            planning_response.joint_trajectory.trajectory.leftCols(
                                static_cast<long>(pick_prob->GetKin()->getJointNames().size())));

  std::cout << planning_response.joint_trajectory.trajectory << '\n';

  ros::Publisher signal_pick = nh_.advertise<std_msgs::Bool>("isInPickPosition",1,true);
  std_msgs::Bool msg_pick;
  msg_pick.data = true;
  signal_pick.publish(msg_pick);

  /////////////
  /// PLACE ///
  /////////////

  if (rviz_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  // Detach the simulated box from the world and attach to the end effector
  Joint joint_box2("joint_box2");
  joint_box2.parent_link_name = end_effector;
  joint_box2.child_link_name = link_box_name;
  joint_box2.type = JointType::FIXED;
  joint_box2.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box2.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0.0, 0.0, -box_side/2-GREIFER_OFFSET-OFFSET);

  tesseract_->getEnvironment()->moveLink(std::move(joint_box2));
  tesseract_->getEnvironment()->addAllowedCollision(link_box_name, "ee_link", "Never");
  tesseract_->getEnvironment()->addAllowedCollision(link_box_name, "greifer_link", "Never");
  tesseract_->getEnvironment()->addAllowedCollision(link_box_name, "wrist_3_link", "Never");
  tesseract_->getEnvironment()->addAllowedCollision(link_box_name, "wrist_2_link", "Never");
  tesseract_->getEnvironment()->addAllowedCollision(link_box_name, end_effector, "Adjacent");

  if (rviz_)
  {
      // Now update rviz environment
      if (!sendRvizChanges(1))
          return false;
  }
  // Set the current state to the last state of the pick trajectory
  tesseract_->getEnvironment()->setState(planning_response.joint_trajectory.joint_names,
                                         planning_response.joint_trajectory.trajectory.bottomRows(1).transpose());

  // Retreat to the approach pose
  Eigen::Isometry3d retreat_pose = approach_pose;

  // Define some place locations.
  Eigen::Isometry3d position_1;
  //position_1.linear() = Eigen::Quaterniond(1, 0, 0, 0).matrix();
  position_1.linear() = Eigen::Quaterniond(1, 0, 0, 0).matrix();
  position_1.translation() = Eigen::Vector3d(-0.6,-0.6,1.0);// the place position in container

  // Set the target pose in container
  final_pose = position_1;

  // Setup approach for place move
  approach_pose = final_pose;
  //approach_pose.linear() += Eigen::Quaterniond(1,0,0,0).matrix();
  //approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.1);
  approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.1);

  // Create the problem construction info
  trajopt::ProblemConstructionInfo pci_place(tesseract_);

  pci_place.basic_info.n_steps = steps_ *3;
  pci_place.basic_info.manip = manip;
  pci_place.basic_info.dt_lower_lim = 2;    // 1/most time
  pci_place.basic_info.dt_upper_lim = 100;  // 1/least time
  pci_place.basic_info.start_fixed = true;
  pci_place.basic_info.use_time = false;

  // Create Kinematic Object
  pci_place.kin = pci_place.getManipulator(pci_place.basic_info.manip);

  pci_place.init_info.type = trajopt::InitInfo::STATIONARY;
  pci_place.init_info.dt = 0.5;

  // Add a collision cost
  {
    auto collision = std::make_shared<trajopt::CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
    collision->evaluator_type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
    collision->first_step = 1;
    collision->last_step = pci_place.basic_info.n_steps - 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci_place.basic_info.n_steps, 0.005, 50);
    pci_place.cost_infos.push_back(collision);
  }

  // Add a velocity cost without time to penalize paths that are longer
  {
    auto jv = std::make_shared<trajopt::JointVelTermInfo>();
    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->term_type = trajopt::TT_COST;
    jv->first_step = 0;
    jv->last_step = pci_place.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cost";
    pci_place.cost_infos.push_back(jv);
  }

  // Add a velocity cnt with time to insure that robot dynamics are obeyed
  if (ENABLE_VELOCITY_COST)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);

    // Taken from ur10 documentation (radians/s) and scaled by 0.8
    std::vector<double> vel_lower_lim{ 1.71 * -0.8, 2.1 * -0.8,  2.1 * -0.8,
                                      3.14 * -0.8, 3.14 * -0.8, 3.14 * -0.8,
                                      3.14 * -0.8 };
    std::vector<double> vel_upper_lim{ 1.71 * 0.8, 2.1 * 0.8,  2.1 * 0.8,
                                      3.14 * 0.8, 3.14 * 0.8, 3.14 * 0.8,
                                      3.14 * 0.8 };

    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 50.0);
    jv->lower_tols = vel_lower_lim;
    jv->upper_tols = vel_upper_lim;
    jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
    jv->first_step = 0;
    jv->last_step = pci_place.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cnt";
    pci_place.cnt_infos.push_back(jv);
  }

  // Add cartesian pose cnt at the retreat point
  Eigen::Isometry3d interim_pose;
  Eigen::Quaterniond interim_orientation(1.0, 0.0, 0.0, 0.0);
  interim_pose.linear() = orientation.matrix();
  interim_pose.translation() += Eigen::Vector3d(box_x, box_y, box_z + box_side/2 +  0.05); //coordinate to the world

  // Define the approach pose
  Eigen::Isometry3d interim_approach_pose = interim_pose;
  interim_approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.1);

  retreat_pose = interim_approach_pose;
  Eigen::Quaterniond rotation(retreat_pose.linear());
  auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
  pose_constraint->term_type = trajopt::TT_CNT;
  pose_constraint->link = link_box_name;
  pose_constraint->timestep = steps_ - 1;
  pose_constraint->xyz = retreat_pose.translation();
  pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
  pose_constraint->pos_coeffs = Eigen::Vector3d(10, 10, 10);
  pose_constraint->rot_coeffs = Eigen::Vector3d(10, 10, 10);
  pose_constraint->name = "pose_" + std::to_string(steps_ - 1);
  pci_place.cnt_infos.push_back(pose_constraint);

  // Add cartesian pose cnt at the final point
  //changed
  auto pose_constraint_ = std::make_shared<trajopt::CartPoseTermInfo>();
  pose_constraint_->term_type = trajopt::TT_CNT;
  pose_constraint_->link = link_box_name;
  pose_constraint_->timestep = 2 * steps_-1;
  //pose_constraint->xyz = Eigen::Vector3d(-0.8, -1.4, 0.45);
  pose_constraint_->xyz = approach_pose.translation();
  //pose_constraint_->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
  pose_constraint_->wxyz = Eigen::Vector4d(1, 0, 0, 0);
  pose_constraint_->pos_coeffs = Eigen::Vector3d(10, 10, 10);
  pose_constraint_->rot_coeffs = Eigen::Vector3d(10, 10, 10);
  pose_constraint_->name = "pose_extra_" + std::to_string(2 * steps_ -1);
  pci_place.cnt_infos.push_back(pose_constraint_);

  // Add a cost on the total time to complete the pick
  if (ENABLE_TIME_COST)
  {
    std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
    time_cost->name = "time_cost";
    time_cost->coeff = 5.0;
    time_cost->term_type = trajopt::TT_COST;
    pci_place.cost_infos.push_back(time_cost);
  }

  // Create the place problem
  trajopt::TrajOptProb::Ptr place_prob = ConstructProblem(pci_place);

  // Set the optimization parameters
  tesseract_motion_planners::TrajOptPlannerConfig config_place(place_prob);
  config_place.params.max_iter = 200;

  // Create Plot Callback
  if (plotting_)
  {
    config_place.callbacks.push_back(PlotCallback(*place_prob, plotter));
  }

  // Create file write callback discarding any of the file's current contents
  std::shared_ptr<std::ofstream> stream_ptr_place(new std::ofstream);
  if (write_to_file_)
  {
    std::string path = ros::package::getPath("pick_and_place") + "/config/file_output_place.csv";
    stream_ptr_place->open(path, std::ofstream::out | std::ofstream::trunc);
    config_place.callbacks.push_back(trajopt::WriteCallback(stream_ptr_place, place_prob));
  }

  // Set Planner Configuration
  planner.setConfiguration(std::make_shared<tesseract_motion_planners::TrajOptPlannerConfig>(config_place));

  // Solve problem
  planner.solve(planning_response_place);

  if (write_to_file_)
    stream_ptr_place->close();

  // Plot the resulting trajectory
  if (plotting_)
    plotter->plotTrajectory(planning_response_place.joint_trajectory.joint_names,
                            planning_response_place.joint_trajectory.trajectory.leftCols(
                                static_cast<long>(place_prob->GetKin()->getJointNames().size())));

  std::cout << planning_response_place.joint_trajectory.trajectory << '\n';

  ros::Publisher signal_place = nh_.advertise<std_msgs::Bool>("isInPlacePosition",1,true);
  std_msgs::Bool msg_place;
  msg_place.data = true;
  signal_place.publish(msg_place);

  if (rviz_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  ROS_INFO("Done");
  return true;
}
}  // namespace unicar

