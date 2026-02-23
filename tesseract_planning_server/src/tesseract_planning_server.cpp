/**
 * @file tesseract_planning_server.cpp
 * @brief A planning server with a default set of motion planners
 *
 * @author Levi Armstrong
 * @date August 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_planning_server/tesseract_planning_server.h>

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization_profiles.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_cache.h>

#include <tesseract_common/any_poly.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/stopwatch.h>

#include <tesseract_common/cereal_serialization.h>
#include <tesseract_command_language/cereal_serialization.h>
#include <tesseract_common/serialization.h>

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/utils.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_server.h>

using tesseract::command_language::InstructionPoly;
using tesseract::common::Serialization;
using tesseract::task_composer::TaskComposerDataStorage;

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";
static const std::string MLT_DEFAULT_NAMESPACE = "MinLengthTask";
static const std::string ISP_DEFAULT_NAMESPACE = "IterativeSplineParameterizationTask";
static const std::string DCC_DEFAULT_NAMESPACE = "DiscreteContactCheckTask";

namespace tesseract_planning_server
{
const std::string TesseractPlanningServer::DEFAULT_GET_MOTION_PLAN_ACTION = "tesseract_get_motion_plan";

TesseractPlanningServer::TesseractPlanningServer(rclcpp::Node::SharedPtr node,
                                                 const std::string& robot_description,
                                                 std::string name)
  : node_(std::move(node))
  , monitor_(std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node_, robot_description, std::move(name)))
  , environment_cache_(std::make_shared<tesseract::environment::DefaultEnvironmentCache>(monitor_->getEnvironment()))
  , profiles_(std::make_shared<tesseract::common::ProfileDictionary>())
  , planning_server_(std::make_unique<tesseract::task_composer::TaskComposerServer>())
  , motion_plan_server_(rclcpp_action::create_server<tesseract_msgs::action::GetMotionPlan>(
        node_,
        DEFAULT_GET_MOTION_PLAN_ACTION,
        std::bind(&TesseractPlanningServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),  // NOLINT
        std::bind(&TesseractPlanningServer::handle_cancel, this, std::placeholders::_1),                       // NOLINT
        std::bind(&TesseractPlanningServer::onMotionPlanningCallback, this, std::placeholders::_1)))           // NOLINT
  , tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock()))
  , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  ctor();
}

TesseractPlanningServer::TesseractPlanningServer(rclcpp::Node::SharedPtr node,
                                                 tesseract::environment::Environment::UPtr env,
                                                 std::string name)
  : node_(std::move(node))
  , monitor_(std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node_, std::move(env), std::move(name)))
  , environment_cache_(std::make_shared<tesseract::environment::DefaultEnvironmentCache>(monitor_->getEnvironment()))
  , profiles_(std::make_shared<tesseract::common::ProfileDictionary>())
  , planning_server_(std::make_unique<tesseract::task_composer::TaskComposerServer>())
  , motion_plan_server_(rclcpp_action::create_server<tesseract_msgs::action::GetMotionPlan>(
        node_,
        DEFAULT_GET_MOTION_PLAN_ACTION,
        std::bind(&TesseractPlanningServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),  // NOLINT
        std::bind(&TesseractPlanningServer::handle_cancel, this, std::placeholders::_1),                       // NOLINT
        std::bind(&TesseractPlanningServer::onMotionPlanningCallback, this, std::placeholders::_1)))           // NOLINT
  , tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock()))
  , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  ctor();
}

void TesseractPlanningServer::ctor()
{
  loadDefaultPlannerProfiles();
  monitor_->environment().addFindTCPOffsetCallback(
      std::bind(&TesseractPlanningServer::tfFindTCPOffset, this, std::placeholders::_1));  // NOLINT
}

tesseract::environment::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() { return *monitor_; }
const tesseract::environment::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() const
{
  return *monitor_;
}

tesseract::task_composer::TaskComposerServer& TesseractPlanningServer::getTaskComposerServer()
{
  return *planning_server_;
}
const tesseract::task_composer::TaskComposerServer& TesseractPlanningServer::getTaskComposerServer() const
{
  return *planning_server_;
}

tesseract::environment::EnvironmentCache& TesseractPlanningServer::getEnvironmentCache() { return *environment_cache_; }
const tesseract::environment::EnvironmentCache& TesseractPlanningServer::getEnvironmentCache() const
{
  return *environment_cache_;
}

tesseract::common::ProfileDictionary& TesseractPlanningServer::getProfileDictionary() { return *profiles_; }
const tesseract::common::ProfileDictionary& TesseractPlanningServer::getProfileDictionary() const { return *profiles_; }

rclcpp_action::GoalResponse
TesseractPlanningServer::handle_goal(const rclcpp_action::GoalUUID&,                                      // NOLINT
                                     std::shared_ptr<const tesseract_msgs::action::GetMotionPlan::Goal>)  // NOLINT
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TesseractPlanningServer::handle_cancel(                               // NOLINT
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tesseract_msgs::action::GetMotionPlan>>)  // NOLINT
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TesseractPlanningServer::onMotionPlanningCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tesseract_msgs::action::GetMotionPlan>>
        goal_handle)  // NOLINT
{
  RCLCPP_INFO(node_->get_logger(), "Tesseract Planning Server Received Request!");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<tesseract_msgs::action::GetMotionPlan::Result>();

  // Check if process planner exist
  if (!planning_server_->hasTask(goal->request.name))
  {
    result->response.successful = false;
    std::ostringstream oss;
    oss << "Requested task '" << goal->request.name << "' is not supported!\n";
    oss << "   Available Tasks:\n";
    for (const auto& planner : planning_server_->getAvailableTasks())
      oss << "      - " << planner << "\n";
    RCLCPP_ERROR_STREAM(node_->get_logger(), oss.str());
    goal_handle->succeed(result);
    return;
  }

  std::string executor_name = goal->request.executor;
  std::vector<std::string> available_executors = planning_server_->getAvailableExecutors();
  if (executor_name.empty() && !available_executors.empty())
    executor_name = planning_server_->getAvailableExecutors().front();

  // Check if executor exists
  if (!planning_server_->hasExecutor(executor_name))
  {
    result->response.successful = false;
    std::ostringstream oss;
    oss << "Requested executor '" << executor_name << "' is not supported!\n";
    oss << "   Available Executors:\n";
    for (const auto& executor : available_executors)
      oss << "      - " << executor << "\n";
    RCLCPP_ERROR_STREAM(node_->get_logger(), oss.str());
    goal_handle->succeed(result);
    return;
  }

  tesseract::common::AnyPoly planning_input;
  try
  {
    planning_input = Serialization::fromArchiveStringXML<tesseract::common::AnyPoly>(goal->request.input);
  }
  catch (const std::exception& e)
  {
    result->response.successful = false;
    std::ostringstream oss;
    oss << "Failed to deserialize planning input instruction with error: '" << e.what() << "'!\n";
    oss << " Make sure the program was serialized from an AnyPoly type.\n";
    RCLCPP_ERROR_STREAM(node_->get_logger(), oss.str());
    goal_handle->succeed(result);
    return;
  }

  tesseract::environment::Environment::Ptr env = environment_cache_->getCachedEnvironment();

  tesseract::scene_graph::SceneState env_state;
  tesseract_rosutils::fromMsg(env_state.joints, goal->request.environment_state.joint_state);

  env->applyCommands(tesseract_rosutils::fromMsg(goal->request.commands));
  env->setState(env_state.joints);

  // Store the initial state in the response for publishing trajectories
  tesseract::scene_graph::SceneState initial_state = env->getState();
  tesseract_rosutils::toMsg(result->response.initial_state, initial_state.joints);

  // Create solve data storage
  auto data = std::make_unique<TaskComposerDataStorage>();
  data->setData("planning_input", std::move(planning_input));
  data->setData("environment", std::shared_ptr<const tesseract::environment::Environment>(std::move(env)));
  data->setData("profiles", profiles_);

  // Solve
  tesseract::common::Stopwatch stopwatch;
  stopwatch.start();
  tesseract::task_composer::TaskComposerFuture::UPtr plan_future =
      planning_server_->run(goal->request.name, std::move(data), goal->request.dotgraph, executor_name);
  plan_future->wait();  // Wait for results
  stopwatch.stop();

  // Generate DOT Graph if requested
  if (goal->request.dotgraph)
    result->response.dotgraph = planning_server_->getTask(plan_future->context->name)
                                    .getDotgraph(plan_future->context->task_infos->getInfoMap());

  try
  {
    const tesseract::task_composer::TaskComposerNode& task = planning_server_->getTask(plan_future->context->name);
    tesseract::common::AnyPoly results = plan_future->context->data_storage->getData(task.getOutputKeys().get("progra"
                                                                                                              "m"));
    result->response.results = Serialization::toArchiveStringXML<tesseract::command_language::InstructionPoly>(
        results.as<tesseract::command_language::CompositeInstruction>());
  }
  catch (const std::exception& e)
  {
    result->response.successful = false;
    std::ostringstream oss;
    oss << "Failed to get output results from task with error: '" << e.what() << "'!\n";
    RCLCPP_ERROR_STREAM(node_->get_logger(), oss.str());
    goal_handle->succeed(result);
    return;
  }

  result->response.successful = plan_future->context->isSuccessful();
  plan_future->clear();

  RCLCPP_INFO(
      node_->get_logger(), "Tesseract Planning Server Finished Request in %f seconds!", stopwatch.elapsedSeconds());
  goal_handle->succeed(result);
}

void TesseractPlanningServer::loadDefaultPlannerProfiles()
{
  // Add TrajOpt Default Profiles
  profiles_->addProfile(TRAJOPT_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>());
  profiles_->addProfile(TRAJOPT_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>());
  profiles_->addProfile(TRAJOPT_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::TrajOptOSQPSolverProfile>());

  // Add TrajOpt IFOPT Default Profiles
  profiles_->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::TrajOptIfoptDefaultMoveProfile>());
  profiles_->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::TrajOptIfoptDefaultCompositeProfile>());
  profiles_->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::TrajOptIfoptOSQPSolverProfile>());

  // Add Descartes Default Profiles
  profiles_->addProfile(DESCARTES_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::DescartesDefaultMoveProfile<double>>());

  // Add OMPL Default Profiles
  profiles_->addProfile(OMPL_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::OMPLRealVectorMoveProfile>());

  // Add Simple Default Profiles
  profiles_->addProfile(SIMPLE_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::motion_planners::SimplePlannerLVSNoIKMoveProfile>());

  // MinLengthTask calls the SimpleMotionPlanner to generate a seed path with waypoints interpolated in joint space
  profiles_->addProfile(MLT_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::task_composer::MinLengthProfile>());

  // Post hoc collision checking
  profiles_->addProfile(DCC_DEFAULT_NAMESPACE,
                        tesseract::command_language::DEFAULT_PROFILE_KEY,
                        std::make_shared<tesseract::task_composer::ContactCheckProfile>());

  // Time parameterization
  profiles_->addProfile(
      ISP_DEFAULT_NAMESPACE,
      tesseract::command_language::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract::time_parameterization::IterativeSplineParameterizationCompositeProfile>());
  profiles_->addProfile(
      ISP_DEFAULT_NAMESPACE,
      tesseract::command_language::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract::time_parameterization::IterativeSplineParameterizationMoveProfile>());
}

Eigen::Isometry3d TesseractPlanningServer::tfFindTCPOffset(const tesseract::common::ManipulatorInfo& manip_info)
{
  if (manip_info.tcp_offset.index() == 1)
    throw std::runtime_error("tfFindTCPOffset: TCP offset is not a string!");

  if (manip_info.tcp_frame.empty())
    throw std::runtime_error("tfFindTCPOffset: TCP offset is empty!");

  const std::string& tcp_frame = manip_info.tcp_frame;
  const std::string& tcp_name = std::get<0>(manip_info.tcp_offset);

  auto tcp_msg = tf_buffer_->lookupTransform(tcp_frame, tcp_name, tf2::TimePointZero);
  return tf2::transformToEigen(tcp_msg.transform);
}

}  // namespace tesseract_planning_server
