/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifdef BOOST_BIND_NO_PLACEHOLDERS
BOOST_BIND_NO_PLACEHOLDERS
#endif

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/utils.h>

namespace tesseract_monitoring
{
ROSEnvironmentMonitor::ROSEnvironmentMonitor(rclcpp::Node::SharedPtr node,
                                             std::string robot_description,
                                             std::string monitor_namespace)
  : EnvironmentMonitor(std::move(monitor_namespace))
  , node_(node)
  , internal_node_(std::make_shared<rclcpp::Node>("ROSEnvironmentMonitor_internal", node->get_fully_qualified_name()))
  , robot_description_(std::move(robot_description))
  , cb_group_(internal_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  , logger_{ internal_node_->get_logger().get_child(monitor_namespace_ + "_monitor") }
{
  // Initial setup
  std::string urdf_xml_string;
  std::string srdf_xml_string;
  if (!node_->get_parameter(robot_description_, urdf_xml_string))
  {
    RCLCPP_ERROR(logger_, "Failed to find parameter: %s", robot_description_.c_str());
    return;
  }
  if (!node_->get_parameter(robot_description_ + "_semantic", srdf_xml_string))
  {
    RCLCPP_ERROR(logger_, "Failed to find parameter: %s", (robot_description_ + "_semantic").c_str());
    return;
  }

  env_ = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init(urdf_xml_string, srdf_xml_string, locator))
    return;

  if (!initialize())
  {
    RCLCPP_WARN(logger_, "EnvironmentMonitor failed to initialize from URDF and SRDF");
  }

  internal_node_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  internal_node_spinner_ = std::make_shared<std::thread>(std::thread{ [this]() {
    internal_node_executor_->add_node(internal_node_);
    internal_node_executor_->spin();
  } });
}

ROSEnvironmentMonitor::ROSEnvironmentMonitor(rclcpp::Node::SharedPtr node,
                                             tesseract_environment::Environment::Ptr env,
                                             std::string monitor_namespace)
  : EnvironmentMonitor(std::move(env), std::move(monitor_namespace))
  , node_(node)
  , internal_node_(std::make_shared<rclcpp::Node>("ROSEnvironmentMonitor_internal", node->get_fully_qualified_name()))
  , cb_group_(internal_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  , logger_{ internal_node_->get_logger().get_child(monitor_namespace_ + "_monitor") }
{
  if (!initialize())
  {
    RCLCPP_WARN(logger_, "ENV passed to ros env monitor did not initialize");
  }
  internal_node_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  internal_node_spinner_ = std::make_shared<std::thread>(std::thread{ [this]() {
    internal_node_executor_->add_node(internal_node_);
    internal_node_executor_->spin();
  } });
}

ROSEnvironmentMonitor::~ROSEnvironmentMonitor()
{
  stopPublishingEnvironment();
  stopStateMonitor();

  current_state_monitor_.reset();
  env_->removeEventCallback(std::hash<ROSEnvironmentMonitor*>()(this));
  env_ = nullptr;

  shutdown();
}

const std::string& ROSEnvironmentMonitor::getURDFDescription() const { return robot_description_; }

bool ROSEnvironmentMonitor::waitForConnection(std::chrono::duration<double> duration) const
{
  const rclcpp::Time start_time = rclcpp::Clock().now();
  rclcpp::Duration wall_timeout(0, 0);
  if (std::chrono::duration_cast<std::chrono::seconds>(duration).count() == 0)
    wall_timeout = rclcpp::Duration(-1, 0);
  else
    wall_timeout = rclcpp::Duration(duration);

  bool is_connected{ false };
  while (rclcpp::ok())
  {
    is_connected = env_->isInitialized();
    if (is_connected)
      return true;

    if (wall_timeout >= rclcpp::Duration(0, 0))
    {
      const rclcpp::Time current_time = rclcpp::Clock().now();
      if ((current_time - start_time) >= wall_timeout)
        return false;
    }

    rclcpp::Rate(50).sleep();
  }

  return false;
}

void ROSEnvironmentMonitor::shutdown()
{
  monitored_environment_subscriber_.reset();
  get_monitored_environment_changes_client_.reset();
  get_monitored_environment_information_client_.reset();
  modify_monitored_environment_client_.reset();
  modify_environment_service_.reset();
  get_environment_changes_service_.reset();
  get_environment_information_service_.reset();
  save_scene_graph_service_.reset();
  internal_node_executor_->cancel();
  if (internal_node_spinner_->joinable())
    internal_node_spinner_->join();
}

bool ROSEnvironmentMonitor::initialize()
{
  enforce_next_state_update_ = false;

  if (monitor_namespace_.empty())
  {
    throw std::runtime_error("The monitor namespace cannot be empty!");
  }

  if (!env_->isInitialized())
  {
    RCLCPP_WARN(logger_, "Failed to initialize environment monitor, the tesseract is uninitialized!");
    return false;
  }

  publish_environment_frequency_ = 30.0;

  last_update_time_ = last_robot_motion_time_ = node_->now();
  last_robot_state_update_wall_time_ = rclcpp::Clock().now();

  state_update_pending_ = false;
  setStateUpdateFrequency(10);

  const auto& _ph1 = std::placeholders::_1;
  const auto& _ph2 = std::placeholders::_2;

  env_->addEventCallback(std::hash<ROSEnvironmentMonitor*>()(this),
                         std::bind(&ROSEnvironmentMonitor::sceneStateChangedCallback, this, _ph1));

  // Shutdown current services
  modify_environment_service_.reset();
  get_environment_changes_service_.reset();
  get_environment_information_service_.reset();
  save_scene_graph_service_.reset();

  // Create new service
  std::string modify_environment_service = R"(/)" + monitor_namespace_ + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  std::string get_environment_changes_service = R"(/)" + monitor_namespace_ + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  std::string get_environment_information_service =
      R"(/)" + monitor_namespace_ + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
  std::string save_scene_graph_service = R"(/)" + monitor_namespace_ + DEFAULT_SAVE_SCENE_GRAPH_SERVICE;

  modify_environment_service_ = internal_node_->create_service<tesseract_msgs::srv::ModifyEnvironment>(
      modify_environment_service,
      std::bind(&ROSEnvironmentMonitor::modifyEnvironmentCallback, this, _ph1, _ph2),
      rmw_qos_profile_services_default,
      cb_group_);

  get_environment_changes_service_ = internal_node_->create_service<tesseract_msgs::srv::GetEnvironmentChanges>(
      get_environment_changes_service,
      std::bind(&ROSEnvironmentMonitor::getEnvironmentChangesCallback, this, _ph1, _ph2),
      rmw_qos_profile_services_default,
      cb_group_);

  get_environment_information_service_ = internal_node_->create_service<tesseract_msgs::srv::GetEnvironmentInformation>(
      get_environment_information_service,
      std::bind(&ROSEnvironmentMonitor::getEnvironmentInformationCallback, this, _ph1, _ph2),
      rmw_qos_profile_services_default,
      cb_group_);

  save_scene_graph_service_ = internal_node_->create_service<tesseract_msgs::srv::SaveSceneGraph>(
      save_scene_graph_service,
      std::bind(&ROSEnvironmentMonitor::saveSceneGraphCallback, this, _ph1, _ph2),
      rmw_qos_profile_services_default,
      cb_group_);

  return true;
}

void ROSEnvironmentMonitor::sceneStateChangedCallback(const tesseract_environment::Event& event)
{
  if (!monitored_environment_subscriber_ && !current_state_monitor_ &&
      (typeid(event) == typeid(tesseract_environment::SceneStateChangedEvent)))
  {
    last_update_time_ = last_robot_motion_time_ = node_->now();
  }
}

void ROSEnvironmentMonitor::stopPublishingEnvironment()
{
  if (publish_environment_)
  {
    std::unique_ptr<std::thread> copy;
    copy.swap(publish_environment_);
    copy->join();
    stopPublishingEnvironment();
    environment_publisher_.reset();
    RCLCPP_INFO(logger_, "Stopped publishing maintained environment.");
  }
}

void ROSEnvironmentMonitor::startPublishingEnvironment()
{
  if (publish_environment_)
  {
    RCLCPP_ERROR(logger_, "Environment publishing thread already exists!");
    return;
  }
  if (!env_->isInitialized())
  {
    RCLCPP_ERROR(logger_, "Tesseract environment not initialized, can't publish environment!");
    return;
  }

  std::string environment_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
  environment_publisher_ =
      node_->create_publisher<tesseract_msgs::msg::EnvironmentState>(environment_topic, rclcpp::QoS(100));
  RCLCPP_INFO(logger_, "Publishing maintained environment on '%s'", environment_topic.c_str());
  publish_environment_ = std::make_unique<std::thread>([this]() { environmentPublishingThread(); });
}

double ROSEnvironmentMonitor::getEnvironmentPublishingFrequency() const { return publish_environment_frequency_; }

void ROSEnvironmentMonitor::environmentPublishingThread()
{
  RCLCPP_INFO(logger_, "Started environment state publishing thread ...");

  // publish the full planning scene
  tesseract_msgs::msg::EnvironmentState start_msg;
  tesseract_rosutils::toMsg(start_msg, *(env_));

  environment_publisher_->publish(start_msg);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.5)));
  environment_publisher_->publish(start_msg);

  RCLCPP_INFO(logger_, "Published the Tesseract Environment State for: '%s'", start_msg.id.c_str());

  do
  {
    tesseract_msgs::msg::EnvironmentState msg;
    bool publish_msg = false;
    rclcpp::Rate rate(publish_environment_frequency_);
    {
      auto lock_read = env_->lockRead();
      tesseract_rosutils::toMsg(msg, *env_);

      // also publish timestamp of this robot_state
      msg.joint_state.header.stamp = last_robot_motion_time_;
      publish_msg = true;
    }

    if (publish_msg)
    {
      rate.reset();
      environment_publisher_->publish(msg);
      rate.sleep();
    }
  } while (publish_environment_);

  RCLCPP_INFO(logger_, "Stopping env publishing thread");
}

void ROSEnvironmentMonitor::stopMonitoringEnvironment()
{
  get_monitored_environment_changes_client_.reset();
  modify_monitored_environment_client_.reset();
  get_monitored_environment_information_client_.reset();
  monitored_environment_subscriber_.reset();
  RCLCPP_INFO(logger_, "Stopped monitoring environment.");
}

const CurrentStateMonitor& ROSEnvironmentMonitor::getStateMonitor() const { return *current_state_monitor_; }

CurrentStateMonitor& ROSEnvironmentMonitor::getStateMonitor() { return *current_state_monitor_; }

void ROSEnvironmentMonitor::startMonitoringEnvironment(const std::string& monitored_namespace,
                                                       tesseract_environment::MonitoredEnvironmentMode mode)
{
  mode_ = mode;
  std::string monitored_environment_topic = R"(/)" + monitored_namespace + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
  std::string monitored_environment_changes_service =
      R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  std::string monitored_environment_modify_service = R"(/)" + monitored_namespace + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  std::string monitored_environment_information_service =
      R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;

  stopMonitoringEnvironment();

  get_monitored_environment_changes_client_ = internal_node_->create_client<tesseract_msgs::srv::GetEnvironmentChanges>(
      monitored_environment_changes_service, rmw_qos_profile_services_default, cb_group_);
  modify_monitored_environment_client_ = internal_node_->create_client<tesseract_msgs::srv::ModifyEnvironment>(
      monitored_environment_modify_service, rmw_qos_profile_services_default, cb_group_);
  get_monitored_environment_information_client_ =
      internal_node_->create_client<tesseract_msgs::srv::GetEnvironmentInformation>(
          monitored_environment_information_service, rmw_qos_profile_services_default, cb_group_);

  monitored_environment_subscriber_ = internal_node_->create_subscription<tesseract_msgs::msg::EnvironmentState>(
      monitored_environment_topic,
      1000,
      std::bind(&ROSEnvironmentMonitor::newEnvironmentStateCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Monitoring external environment on '%s'", monitored_environment_topic.c_str());
}

double ROSEnvironmentMonitor::getStateUpdateFrequency() const
{
  if (dt_state_update_.nanoseconds() > 0)
    return 1.0 / dt_state_update_.seconds();

  return 0.0;
}

void ROSEnvironmentMonitor::newEnvironmentStateCallback(const tesseract_msgs::msg::EnvironmentState::ConstSharedPtr env)
{
  last_update_time_ = node_->now();

  if (!env_->isInitialized())
  {
    auto get_env_info_req = std::make_shared<tesseract_msgs::srv::GetEnvironmentInformation::Request>();
    get_env_info_req->flags = tesseract_msgs::srv::GetEnvironmentInformation::Request::COMMAND_HISTORY |
                              tesseract_msgs::srv::GetEnvironmentInformation::Request::KINEMATICS_INFORMATION;

    auto gei_result_future = get_monitored_environment_information_client_->async_send_request(get_env_info_req);
    gei_result_future.wait();
    auto gei_res = gei_result_future.get();
    if (!gei_res->success)
    {
      RCLCPP_ERROR_STREAM(logger_, "newEnvironmentStateCallback: Failed to get monitor environment information!");
      return;
    }

    tesseract_environment::Commands commands;
    try
    {
      commands = tesseract_rosutils::fromMsg(gei_res->command_history);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "newEnvironmentStateCallback: Failed to convert command history message, %s!", e.what());
      return;
    }

    if (!env_->init(commands))
    {
      RCLCPP_ERROR_STREAM(logger_, "newEnvironmentStateCallback: Failed to initialize environment!");
      return;
    }

    if (!initialize())
    {
      RCLCPP_WARN(logger_, "newEnvironmentStateCallback: EnvironmentMonitor Failed to initialize!");
    }
  }
  else
  {
    // If the monitored environment has changed then request the changes and apply
    if (static_cast<int>(env->revision) > env_->getRevision())
    {
      auto get_env_changes_req = std::make_shared<tesseract_msgs::srv::GetEnvironmentChanges::Request>();
      get_env_changes_req->revision = static_cast<unsigned long>(env_->getRevision());

      auto gec_result_future = get_monitored_environment_changes_client_->async_send_request(get_env_changes_req);
      gec_result_future.wait();
      auto gec_res = gec_result_future.get();
      if (gec_res->success)
      {
        if (!tesseract_rosutils::processMsg(*env_, gec_res->commands))
        {
          RCLCPP_ERROR_STREAM(logger_, "newEnvironmentStateCallback: Failed to apply monitored environments changes.");
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_, "newEnvironmentStateCallback: Failed to get monitored environments changes.");
      }
    }
    else if (static_cast<int>(env->revision) < env_->getRevision())
    {
      if (mode_ == tesseract_environment::MonitoredEnvironmentMode::DEFAULT)
      {
        // If the monitored environment has a lower revision it is reset and additional changes are requested and
        // applied.
        if (env_->reset())
        {
          if (static_cast<int>(env->revision) > env_->getRevision())
          {
            auto get_env_changes_req = std::make_shared<tesseract_msgs::srv::GetEnvironmentChanges::Request>();
            get_env_changes_req->revision = static_cast<unsigned long>(env_->getRevision());

            auto gec_result_future = get_monitored_environment_changes_client_->async_send_request(get_env_changes_req);
            gec_result_future.wait();
            auto gec_res = gec_result_future.get();
            if (gec_res->success)
            {
              if (!tesseract_rosutils::processMsg(*env_, gec_res->commands))
              {
                RCLCPP_ERROR_STREAM(logger_,
                                    "newEnvironmentStateCallback: Failed to apply monitored environments "
                                    "changes.");
              }
            }
            else
            {
              RCLCPP_ERROR_STREAM(logger_,
                                  "newEnvironmentStateCallback: Failed to get monitored environments changes.");
            }
          }
        }
        else
        {
          RCLCPP_ERROR_STREAM(logger_, "newEnvironmentStateCallback: Failed to reset the tesseract object!");
        }
      }
      else if (mode_ == tesseract_environment::MonitoredEnvironmentMode::SYNCHRONIZED)
      {
        // If this has been modified it will push the changes to the monitored environment to keep them in sync
        auto modify_env_req = std::make_shared<tesseract_msgs::srv::ModifyEnvironment::Request>();
        modify_env_req->id = env_->getName();
        modify_env_req->revision = env->revision;
        if (tesseract_rosutils::toMsg(modify_env_req->commands, env_->getCommandHistory(), env->revision))
        {
          auto me_result_future = modify_monitored_environment_client_->async_send_request(modify_env_req);
          me_result_future.wait();
          auto me_res = me_result_future.get();
          if (!me_res->success)
          {
            RCLCPP_ERROR_STREAM(logger_, "newEnvironmentStateCallback: Failed to update monitored environment!");
          }
        }
        else
        {
          RCLCPP_ERROR_STREAM(logger_,
                              "newEnvironmentStateCallback: Failed to convert latest changes to message and "
                              "update "
                              "monitored environment!");
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_, "newEnvironmentStateCallback: Unsupported MonitoredEnvironmentMode!");
      }
    }
  }

  if (!tesseract_rosutils::isMsgEmpty(env->joint_state))
  {
    if (last_robot_motion_time_ != env->joint_state.header.stamp)
    {
      tesseract_rosutils::processMsg(*env_, env->joint_state);
      last_robot_motion_time_ = env->joint_state.header.stamp;
    }
  }

  RCLCPP_DEBUG_STREAM(logger_,
                      "environment update " << fmod(last_update_time_.seconds(), 10.)
                                            << " robot stamp: " << fmod(last_robot_motion_time_.seconds(), 10.));
}

bool ROSEnvironmentMonitor::applyEnvironmentCommandsMessage(
    const std::string& id,
    int revision,
    const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands)
{
  if (!env_ || id != env_->getName() || revision != env_->getRevision())
    return false;

  bool result = true;

  // Update joint state is not a tracked command so need to filter them out.
  std::vector<tesseract_msgs::msg::EnvironmentCommand> filtered_commands;
  std::vector<tesseract_msgs::msg::EnvironmentCommand> update_joint_state_commands;
  for (const auto& cmd : commands)
  {
    if (cmd.command == tesseract_msgs::msg::EnvironmentCommand::UPDATE_JOINT_STATE)
      update_joint_state_commands.push_back(cmd);
    else
      filtered_commands.push_back(cmd);
  }

  if (!filtered_commands.empty())
    result = tesseract_rosutils::processMsg(*env_, filtered_commands);

  if (result)
  {
    for (const auto& cmd : update_joint_state_commands)
    {
      if (tesseract_rosutils::processMsg(*env_, cmd.joint_state))
      {
        last_robot_motion_time_ = node_->now();
      }
      else
      {
        RCLCPP_ERROR(logger_, "Failed to apply UPDATE_JOINT_STATE command!");
        result = false;
      }
    }
  }

  return result;
}

void ROSEnvironmentMonitor::saveSceneGraphCallback(tesseract_msgs::srv::SaveSceneGraph::Request::SharedPtr req,
                                                   tesseract_msgs::srv::SaveSceneGraph::Response::SharedPtr res)
{
  res->success = false;
  if (env_ != nullptr)
  {
    auto lock = env_->lockRead();
    res->success = true;
    env_->getSceneGraph()->saveDOT(req->filepath);
    res->id = env_->getName();
    res->revision = static_cast<unsigned long>(env_->getRevision());
  }
  return;
}

bool ROSEnvironmentMonitor::waitForCurrentState(std::chrono::duration<double> duration)
{
  if (std::chrono::duration_cast<std::chrono::seconds>(duration).count() == 0)
    return false;

  rclcpp::Time t = node_->now();
  rclcpp::Time start = rclcpp::Clock().now();
  auto timeout = rclcpp::Duration::from_seconds(duration.count());

  RCLCPP_DEBUG(logger_, "sync robot state to: %.3fs", fmod(duration.count(), 10.));

  if (current_state_monitor_)
  {
    // Wait for next robot update in state monitor. Those updates are only
    // passed to PSM when robot actually moved!
    enforce_next_state_update_ = true;  // enforce potential updates to be directly applied
    bool success = current_state_monitor_->waitForCurrentState(t, duration.count());
    enforce_next_state_update_ = false;  // back to normal throttling behavior,
                                         // not applying incoming updates
                                         // immediately

    /* If the robot doesn't move, we will never receive an update from CSM in
       planning scene.
       As we ensured that an update, if it is triggered by CSM, is directly
       passed to the scene,
       we can early return true here (if we successfully received a CSM update).
       Otherwise return false. */
    if (success)
      return true;

    RCLCPP_WARN(logger_, "Failed to fetch current robot state.");
    return false;
  }

  // Sometimes there is no state monitor. In this case state updates are
  // received as part of scene updates only.
  // However, scene updates are only published if the robot actually moves.
  // Hence we need a timeout!
  // As publishing planning scene updates is throttled (2Hz by default), a 1s
  // timeout is a suitable default.
  rclcpp::Time prev_robot_motion_time = last_robot_motion_time_;
  while (last_robot_motion_time_ < t &&  // Wait until the state update actually reaches the scene.
         timeout > rclcpp::Duration(0, 0))
  {
    RCLCPP_DEBUG_STREAM(logger_, "last robot motion: " << (t - last_robot_motion_time_).seconds() << " ago");
    timeout = timeout - (rclcpp::Clock().now() - start);  // compute remaining wait_time
  }
  bool success = last_robot_motion_time_ >= t;
  // suppress warning if we received an update at all
  if (!success && prev_robot_motion_time != last_robot_motion_time_)
    RCLCPP_WARN(
        logger_, "Maybe failed to update robot state, time diff: %.3fs", (start - last_robot_motion_time_).seconds());

  RCLCPP_DEBUG_STREAM(logger_,
                      "sync done: robot motion: " << (t - last_robot_motion_time_).seconds()
                                                  << " scene update: " << (t - last_update_time_).seconds());
  return success;
}

void ROSEnvironmentMonitor::startStateMonitor(const std::string& joint_states_topic, bool publish_tf)
{
  stopStateMonitor();
  if (env_)
  {
    if (!current_state_monitor_)
      current_state_monitor_.reset(new CurrentStateMonitor(env_, node_));

    current_state_monitor_->addUpdateCallback(
        std::bind(&ROSEnvironmentMonitor::onJointStateUpdate, this, std::placeholders::_1));
    current_state_monitor_->startStateMonitor(joint_states_topic, publish_tf);

    {
      std::scoped_lock lock(state_pending_mutex_);
      if (dt_state_update_.nanoseconds() > 0)
        publish_ = true;
    }
  }
  else
  {
    RCLCPP_ERROR(logger_, "Cannot monitor robot state because planning scene is not configured");
  }
}

void ROSEnvironmentMonitor::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();

  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  if (state_update_timer_)
    state_update_timer_->cancel();

  publish_ = false;
  {
    std::scoped_lock lock(state_pending_mutex_);
    state_update_pending_ = false;
  }
}

void ROSEnvironmentMonitor::onJointStateUpdate(const sensor_msgs::msg::JointState::ConstSharedPtr /* joint_state */)
{
  rclcpp::Time n = rclcpp::Clock().now();
  rclcpp::Duration dt = n - last_robot_state_update_wall_time_;

  bool update = enforce_next_state_update_;
  {
    std::scoped_lock lock(state_pending_mutex_);

    if (dt < dt_state_update_ && !update)
    {
      state_update_pending_ = true;
    }
    else
    {
      state_update_pending_ = false;
      last_robot_state_update_wall_time_ = n;
      update = true;
    }
  }
  // run the state update with state_pending_mutex_ unlocked
  if (update)
    updateEnvironmentWithCurrentState();
}

void ROSEnvironmentMonitor::updateJointStateTimerCallback()
{
  if (state_update_pending_ && publish_)
  {
    bool update = false;

    rclcpp::Duration dt = rclcpp::Clock().now() - last_robot_state_update_wall_time_;

    {
      // lock for access to dt_state_update_ and state_update_pending_
      std::scoped_lock lock(state_pending_mutex_);
      if (state_update_pending_ && dt >= dt_state_update_)
      {
        state_update_pending_ = false;
        last_robot_state_update_wall_time_ = rclcpp::Clock().now();
        update = true;
        RCLCPP_DEBUG_STREAM(logger_,
                            "performPendingStateUpdate: " << fmod(last_robot_state_update_wall_time_.seconds(), 10));
      }
    }

    // run the state update with state_pending_mutex_ unlocked
    if (update)
    {
      updateEnvironmentWithCurrentState();
      RCLCPP_DEBUG(logger_, "performPendingStateUpdate done");
    }
  }
}

void ROSEnvironmentMonitor::setStateUpdateFrequency(double hz)
{
  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  if (state_update_timer_)
    state_update_timer_->cancel();

  bool update = false;
  if (hz > std::numeric_limits<double>::epsilon())
  {
    std::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = rclcpp::Duration::from_seconds(1.0 / hz);
    state_update_timer_ = node_->create_wall_timer(dt_state_update_.to_chrono<std::chrono::duration<double>>(),
                                                   [this]() { updateJointStateTimerCallback(); });
    publish_ = true;
  }
  else
  {
    publish_ = false;
    std::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = rclcpp::Duration(0, 0);
    if (state_update_pending_)
      update = true;
  }
  RCLCPP_INFO(logger_, "Updating internal planning scene state at most every %lf seconds", dt_state_update_.seconds());

  if (update)
    updateEnvironmentWithCurrentState();
}

void ROSEnvironmentMonitor::updateEnvironmentWithCurrentState()
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) &&
        (node_->now() - current_state_monitor_->getMonitorStartTime()).seconds() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      RCLCPP_WARN_THROTTLE(logger_,
                           *node_->get_clock(),
                           1.0,
                           "The complete state of the robot is not yet known.  Missing %s",
                           missing_str.c_str());
    }

    last_update_time_ = last_robot_motion_time_ = current_state_monitor_->getCurrentStateTime();
    RCLCPP_DEBUG_STREAM(logger_, "robot state update " << fmod(last_robot_motion_time_.seconds(), 10.));

    env_->setState(current_state_monitor_->getCurrentState().joints);
  }
  else
    RCLCPP_ERROR_THROTTLE(
        logger_, *node_->get_clock(), 1.0, "State monitor is not active. Unable to set the planning scene state");
}

void ROSEnvironmentMonitor::setEnvironmentPublishingFrequency(double hz)
{
  publish_environment_frequency_ = hz;
  RCLCPP_DEBUG(logger_, "Maximum frquency for publishing an environment is now %lf Hz", publish_environment_frequency_);
}

void ROSEnvironmentMonitor::modifyEnvironmentCallback(tesseract_msgs::srv::ModifyEnvironment::Request::SharedPtr req,
                                                      tesseract_msgs::srv::ModifyEnvironment::Response::SharedPtr res)
{
  if (req->append)
    res->success = applyEnvironmentCommandsMessage(req->id, env_->getRevision(), req->commands);
  else
    res->success = applyEnvironmentCommandsMessage(req->id, static_cast<int>(req->revision), req->commands);

  res->revision = static_cast<unsigned long>(env_->getRevision());
  return;
}

void ROSEnvironmentMonitor::getEnvironmentChangesCallback(
    tesseract_msgs::srv::GetEnvironmentChanges::Request::SharedPtr req,
    tesseract_msgs::srv::GetEnvironmentChanges::Response::SharedPtr res)
{
  auto lock_read = env_->lockRead();

  if (static_cast<int>(req->revision) > env_->getRevision())
  {
    res->success = false;
    return;
  }

  res->id = env_->getName();
  res->revision = static_cast<unsigned long>(env_->getRevision());
  if (!tesseract_rosutils::toMsg(res->commands, env_->getCommandHistory(), req->revision))
  {
    res->success = false;
    return;
  }

  res->success = true;
  return;
}

void ROSEnvironmentMonitor::getEnvironmentInformationCallback(
    tesseract_msgs::srv::GetEnvironmentInformation::Request::SharedPtr req,
    tesseract_msgs::srv::GetEnvironmentInformation::Response::SharedPtr res)
{
  auto lock_read = env_->lockRead();

  if (!env_->isInitialized())
  {
    res->success = false;
    return;
  }

  res->id = env_->getName();
  res->revision = static_cast<unsigned long>(env_->getRevision());

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::COMMAND_HISTORY)
  {
    if (!tesseract_rosutils::toMsg(res->command_history, env_->getCommandHistory(), 0))
    {
      res->success = false;
      return;
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_LIST)
  {
    for (const auto& link : env_->getSceneGraph()->getLinks())
    {
      tesseract_msgs::msg::Link msg;
      if (!tesseract_rosutils::toMsg(msg, *link))
      {
        res->success = false;
        return;
      }
      res->links.push_back(msg);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_LIST)
  {
    for (const auto& joint : env_->getSceneGraph()->getJoints())
    {
      tesseract_msgs::msg::Joint msg;
      if (!tesseract_rosutils::toMsg(msg, *joint))
      {
        res->success = false;
        return;
      }
      res->joints.push_back(msg);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_NAMES)
  {
    for (const auto& link : env_->getLinkNames())
    {
      res->link_names.push_back(link);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_NAMES)
  {
    for (const auto& joint : env_->getJointNames())
    {
      res->joint_names.push_back(joint);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::ACTIVE_LINK_NAMES)
  {
    for (const auto& link : env_->getActiveLinkNames())
    {
      res->active_link_names.push_back(link);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::ACTIVE_JOINT_NAMES)
  {
    for (const auto& joint : env_->getActiveJointNames())
    {
      res->active_joint_names.push_back(joint);
    }
  }

  tesseract_scene_graph::SceneState state = env_->getState();
  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_TRANSFORMS)
  {
    for (const auto& link_pair : state.link_transforms)
    {
      res->link_transforms.names.push_back(link_pair.first);
      geometry_msgs::msg::Pose pose;
      tesseract_rosutils::toMsg(pose, link_pair.second);
      res->link_transforms.transforms.push_back(pose);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_TRANSFORMS)
  {
    for (const auto& joint_pair : state.joint_transforms)
    {
      res->joint_transforms.names.push_back(joint_pair.first);
      geometry_msgs::msg::Pose pose;
      tesseract_rosutils::toMsg(pose, joint_pair.second);
      res->joint_transforms.transforms.push_back(pose);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::ALLOWED_COLLISION_MATRIX)
  {
    if (!tesseract_rosutils::toMsg(res->allowed_collision_matrix, *env_->getAllowedCollisionMatrix()))
    {
      res->success = false;
      return;
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::KINEMATICS_INFORMATION)
  {
    if (!tesseract_rosutils::toMsg(res->kinematics_information, env_->getKinematicsInformation()))
    {
      res->success = false;
      return;
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_STATES)
  {
    if (!tesseract_rosutils::toMsg(res->joint_states, state.joints))
    {
      res->success = false;
      return;
    }
  }

  res->success = true;
  return;
}

}  // namespace tesseract_monitoring
