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
#include <tesseract_environment/utils.h>
#include <tesseract_rosutils/utils.h>

namespace tesseract_monitoring
{
EnvironmentMonitor::EnvironmentMonitor(
    rclcpp::Node::SharedPtr node,
    const std::string& robot_description,
    std::string monitor_namespace,
    std::string discrete_plugin,
    std::string continuous_plugin
  )
  : node_{node}
  , monitor_namespace_(std::move(monitor_namespace))
  , discrete_plugin_name_(std::move(discrete_plugin))
  , continuous_plugin_name_(std::move(continuous_plugin))
{
  // Initial environment data
  if (! node_->has_parameter(robot_description))
  {
    throw std::runtime_error("Failed to find parameter: '" + robot_description + "'");
  }

  if (! node_->has_parameter(robot_description + "_semantic"))
  {
    throw std::runtime_error("Failed to find parameter: '" + robot_description + "_semantic'");
  }

  std::string urdf_xml = node_->get_parameter(robot_description).as_string();
  std::string srdf_xml = node_->get_parameter(robot_description + "_semantic").as_string();

  env_ = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (! env_->init(urdf_xml, srdf_xml, locator))
  {
    throw std::runtime_error("Failed to initialize environment from URDF and SRDF");
  }

  if (!initialize())
  {
    throw std::runtime_error("EnvironmentMonitor failed to initialize from URDF and SRDF");
  }
}

EnvironmentMonitor::EnvironmentMonitor(
    rclcpp::Node::SharedPtr node,
    tesseract_environment::Environment::Ptr env,
    std::string monitor_namespace,
    std::string discrete_plugin,
    std::string continuous_plugin
  )
  : node_{node}
  , monitor_namespace_(std::move(monitor_namespace))
  , discrete_plugin_name_(std::move(discrete_plugin))
  , continuous_plugin_name_(std::move(continuous_plugin))
  , env_(std::move(env))
{
  if (! initialize())
  {
    throw std::runtime_error("EnvironmentMonitor failed to initialize from tesseract env");
  }
}

EnvironmentMonitor::~EnvironmentMonitor()
{
  stopPublishingEnvironment();
  stopStateMonitor();

  current_state_monitor_.reset();
  env_ = nullptr;

  shutdown();
}

/*
bool EnvironmentMonitor::waitForConnection(rclcpp::Duration timeout) const
{
  const rclcpp::Time start_time = node_->now();
  bool is_connected{ false };
  while (ros::ok())
  {
    {
      auto lock = std::shared_lock(scene_update_mutex_);
      is_connected = env_->isInitialized();
    }
    if (is_connected)
      return true;

    if (wall_timeout >= ros::WallDuration(0))
    {
      const ros::WallTime current_time = ros::WallTime::now();
      if ((current_time - start_time) >= wall_timeout)
        return false;
    }

    ros::WallDuration(0.02).sleep();
    ros::spinOnce();
  }

  return false;
}
*/

void EnvironmentMonitor::shutdown()
{
  //monitored_environment_subscriber_.reset();
  modify_environment_server_.reset();
  get_environment_changes_server_.reset();
  get_environment_information_server_.reset();
  save_scene_graph_server_.reset();
}

bool EnvironmentMonitor::initialize()
{
  enforce_next_state_update_ = false;

  if (monitor_namespace_.empty())
  {
    throw std::runtime_error("The monitor namespace cannot be empty");
  }

  if (! env_->isInitialized())
  {
    throw std::runtime_error("Failed to initalize environment monitor, the tesseract is uninitialized");
  }

  try
  {
    discrete_manager_loader_.reset(new DiscreteContactManagerPluginLoader(
          "tesseract_collision", "tesseract_collision::DiscreteContactManager"));
    for (auto plugin : discrete_manager_loader_->getDeclaredClasses())
    {
      auto fn = [&]() -> tesseract_collision::DiscreteContactManager::Ptr {
        return discrete_manager_loader_->createUniqueInstance(plugin);
      };
      // TODO
      //env_->registerDiscreteContactManager(discrete_manager_loader_->getClassType(plugin), fn);

      RCLCPP_INFO(node_->get_logger(),
          "Discrete Contact Monitor Registered: %s", discrete_manager_loader_->getClassType(plugin).c_str());
    }

    // The tesseract sets a default so it is ok if one is not provided here.
    if (! discrete_plugin_name_.empty())
    {
      if (! discrete_manager_loader_->isClassAvailable(discrete_plugin_name_))
      {
        std::string msg = "\nFailed to set default discrete contact checker plugin: ";
        msg += discrete_plugin_name_ + '\n';
        msg += "  Available Plugins:\n";

        auto available_plugins = discrete_manager_loader_->getDeclaredClasses();
        for (const auto& plugin : available_plugins)
          msg += "    " + plugin + '\n';

        RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
      }
      else
      {
        env_->setActiveDiscreteContactManager(discrete_plugin_name_);
      }
    }

    continuous_manager_loader_.reset(new ContinuousContactManagerPluginLoader(
          "tesseract_collision", "tesseract_collision::ContinuousContactManager"));
    for (auto plugin : continuous_manager_loader_->getDeclaredClasses())
    {
      auto fn = [&]() -> tesseract_collision::ContinuousContactManager::Ptr {
        return continuous_manager_loader_->createUniqueInstance(plugin);
      };
      // TODO
      //  env_->registerContinuousContactManager(continuous_manager_loader_->getClassType(plugin), fn);

      RCLCPP_INFO(node_->get_logger(),
          "Continuous Contact Monitor Registered: %s", continuous_manager_loader_->getClassType(plugin).c_str());
    }

    if (! continuous_plugin_name_.empty())
    {
      if (! continuous_manager_loader_->isClassAvailable(continuous_plugin_name_))
      {
        std::string msg = "\nFailed to set default continuous contact checker plugin: ";
        msg += continuous_plugin_name_ + '\n';
        msg += "  Available Plugins:\n";

        auto available_plugins = continuous_manager_loader_->getDeclaredClasses();
        for (const auto& plugin : available_plugins)
          msg += "    " + plugin + '\n';

        RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
      }
      else
      {
        env_->setActiveContinuousContactManager(continuous_plugin_name_);
      }
    }
  }
  catch (int& /*e*/)
  {
    RCLCPP_ERROR(node_->get_logger().get_child(monitor_namespace_), "Failed to load tesseract contact managers plugin");
    env_.reset();
  }

  publish_environment_frequency_ = 30.0;

  last_update_time_ = last_robot_motion_time_ = node_->now();
  last_robot_state_update_wall_time_ = node_->now();
  dt_state_update_ = rclcpp::Duration::from_seconds(0.1);

  state_update_pending_ = false;
  state_update_timer_ = node_->create_wall_timer(
    dt_state_update_.to_chrono<std::chrono::duration<double>>(),
    [this]()
    {
      updateJointStateTimerCallback();
    }
  );

  const auto& _ph1 = std::placeholders::_1;
  const auto& _ph2 = std::placeholders::_2;

  // Create new service
  std::string modify_environment_service =
    R"(/)" + monitor_namespace_ + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  modify_environment_server_ = node_->create_service<tesseract_msgs::srv::ModifyEnvironment>(
    modify_environment_service,
    std::bind(&EnvironmentMonitor::modifyEnvironmentCallback, this, _ph1, _ph2));

  std::string get_environment_changes_service =
    R"(/)" + monitor_namespace_ + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  get_environment_changes_server_ = node_->create_service<tesseract_msgs::srv::GetEnvironmentChanges>(
    get_environment_changes_service,
    std::bind(&EnvironmentMonitor::getEnvironmentChangesCallback, this, _ph1, _ph2));

  std::string get_environment_information_service =
    R"(/)" + monitor_namespace_ + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
  get_environment_information_server_ = node_->create_service<tesseract_msgs::srv::GetEnvironmentInformation>(
    get_environment_information_service,
    std::bind(&EnvironmentMonitor::getEnvironmentInformationCallback, this, _ph1, _ph2));

  std::string save_scene_graph_service =
    R"(/)" + monitor_namespace_ + DEFAULT_SAVE_SCENE_GRAPH_SERVICE;
  save_scene_graph_server_ = node_->create_service<tesseract_msgs::srv::SaveSceneGraph>(
    save_scene_graph_service,
    std::bind(&EnvironmentMonitor::saveSceneGraphCallback, this, _ph1, _ph2));

  return true;
}

const std::string& EnvironmentMonitor::getName() const { return monitor_namespace_; }

bool EnvironmentMonitor::applyCommand(const tesseract_environment::Command::ConstPtr& command)
{
  bool result = false;
  {
    auto lock = lockEnvironmentWrite();
    result = env_->applyCommand(command);
  }
  triggerEnvironmentUpdateEvent();
  return result;
}

bool EnvironmentMonitor::applyCommands(const tesseract_environment::Commands& commands)
{
  bool result = false;
  {
    auto lock = lockEnvironmentWrite();
    result = env_->applyCommands(commands);
  }
  triggerEnvironmentUpdateEvent();
  return result;
}

tesseract_scene_graph::SceneGraph::ConstPtr EnvironmentMonitor::getSceneGraph() const { return env_->getSceneGraph(); }

tesseract_srdf::KinematicsInformation EnvironmentMonitor::getKinematicsInformation() const
{
  return env_->getKinematicsInformation();
}

tesseract_environment::Environment::Ptr EnvironmentMonitor::getEnvironment() { return env_; }

tesseract_environment::Environment::ConstPtr EnvironmentMonitor::getEnvironment() const { return env_; }

void EnvironmentMonitor::stopPublishingEnvironment()
{
  if (publish_environment_)
  {
    std::unique_ptr<std::thread> copy;
    copy.swap(publish_environment_);
    new_environment_update_condition_.notify_all();
    copy->join();
    stopPublishingEnvironment();
    environment_publisher_.reset();
    RCLCPP_INFO(node_->get_logger().get_child(monitor_namespace_), "Stopped publishing maintained environment.");
  }
}

void EnvironmentMonitor::startPublishingEnvironment()
{
  rclcpp::Logger logger = node_->get_logger().get_child(monitor_namespace_ + "_monitor");
  if (publish_environment_)
  {
    RCLCPP_ERROR(logger, "Environment publishing thread already exists!");
    return;
  }
  if (! env_->isInitialized())
  {
    RCLCPP_ERROR(logger, "Tesseract environment not initialized, can't publish environment!");
    return;
  }

  std::string environment_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
  environment_publisher_ = node_->create_publisher<tesseract_msgs::msg::EnvironmentState>(
      environment_topic, rclcpp::QoS(100));
  RCLCPP_INFO(logger, "Publishing maintained environment on '%s'", environment_topic.c_str());
  publish_environment_ = std::make_unique<std::thread>(
      [this]()
      {
        environmentPublishingThread();
      }
  );
}

double EnvironmentMonitor::getEnvironmentPublishingFrequency() const { return publish_environment_frequency_; }

void EnvironmentMonitor::environmentPublishingThread()
{
  rclcpp::Logger logger = node_->get_logger().get_child(monitor_namespace_ + "_monitor");
  RCLCPP_INFO(logger, "Started environment state publishing thread ...");

  // publish the full planning scene
  tesseract_msgs::msg::EnvironmentState start_msg;
  tesseract_rosutils::toMsg(start_msg, *(env_));

  environment_publisher_->publish(start_msg);
  std::this_thread::sleep_for(std::chrono::duration<double>(1.5));
  environment_publisher_->publish(start_msg);

  RCLCPP_INFO(logger, "Published the Tesseract Environment State for: '%s'", start_msg.id.c_str());

  do
  {
    tesseract_msgs::msg::EnvironmentState msg;
    bool publish_msg = false;
    rclcpp::Rate rate(publish_environment_frequency_);
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
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

  RCLCPP_INFO(logger, "Stopping env publishing thread");
}

CurrentStateMonitor::ConstPtr EnvironmentMonitor::getStateMonitor() const { return current_state_monitor_; }

CurrentStateMonitor::Ptr EnvironmentMonitor::getStateMonitor() { return current_state_monitor_; }

/*
void EnvironmentMonitor::startMonitoringEnvironment(const std::string& monitored_namespace,
                                                    MonitoredEnvironmentMode mode)
{
  monitored_environment_mode_ = mode;
  std::string monitored_environment_topic = R"(/)" + monitored_namespace + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
  std::string monitored_environment_changes_service =
      R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  std::string monitored_environment_modify_service = R"(/)" + monitored_namespace + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  std::string monitored_environment_information_service =
      R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;

  stopMonitoringEnvironment();

  get_monitored_environment_changes_client_ =
      nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(monitored_environment_changes_service);
  modify_monitored_environment_client_ =
      nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(monitored_environment_modify_service);
  get_monitored_environment_information_client_ =
      nh_.serviceClient<tesseract_msgs::GetEnvironmentInformation>(monitored_environment_information_service);

  monitored_environment_subscriber_ =
      nh_.subscribe(monitored_environment_topic, 1000, &EnvironmentMonitor::newEnvironmentStateCallback, this);
  RCLCPP_INFO(node_->get_logger().get_child(monitor_namespace_), "Monitoring external environment on '%s'", monitored_environment_topic.c_str());
}

void EnvironmentMonitor::stopMonitoringEnvironment()
{
  monitored_environment_subscriber_.reset();
  RCLCPP_INFO(node_->get_logger().get_child(monitor_namespace_), "Stopped monitoring environment.");
}
*/

void EnvironmentMonitor::getStateMonitoredTopics(std::vector<std::string>& topics) const
{
  topics.clear();
  if (current_state_monitor_)
  {
    const std::string& t = current_state_monitor_->getMonitoredTopic();
    if (!t.empty())
      topics.push_back(t);
  }
}

double EnvironmentMonitor::getStateUpdateFrequency() const
{
  if (dt_state_update_.nanoseconds() > 0)
    return 1.0 / dt_state_update_.seconds();

  return 0.0;
}

void EnvironmentMonitor::triggerEnvironmentUpdateEvent()
{
  // do not modify update functions while we are calling them
  std::scoped_lock<std::recursive_mutex> lock(update_lock_);

  for (auto& update_callback : update_callbacks_)
    update_callback();

  new_environment_update_condition_.notify_all();
}

/*
void EnvironmentMonitor::newEnvironmentStateCallback(const tesseract_msgs::EnvironmentStateConstPtr& env)
{
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    last_update_time_ = ros::Time::now();

    if (!env_->isInitialized())
    {
      tesseract_msgs::GetEnvironmentInformation res;
      res.request.flags = tesseract_msgs::GetEnvironmentInformation::Request::COMMAND_HISTORY |
                          tesseract_msgs::GetEnvironmentInformation::Request::KINEMATICS_INFORMATION;

      bool status = get_monitored_environment_information_client_.call(res);
      if (!status || !res.response.success)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                               "newEnvironmentStateCallback: Failed to get monitor environment information!");
        return;
      }

      tesseract_environment::Commands commands;
      try
      {
        commands = tesseract_rosutils::fromMsg(res.response.command_history);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(node_->get_logger().get_child(monitor_namespace_),
                        "newEnvironmentStateCallback: Failed to convert command history message, %s!",
                        e.what());
        return;
      }

      if (!env_->init(commands))
      {
        RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_), "newEnvironmentStateCallback: Failed to initialize environment!");
        return;
      }

      if (!initialize())
      {
        RCLCPP_WARN("newEnvironmentStateCallback: EnvironmentMonitor Failed to initialize!");
      }
    }
    else
    {
      // If the monitored environment has changed then request the changes and apply
      if (static_cast<int>(env->revision) > env_->getRevision())
      {
        tesseract_msgs::GetEnvironmentChanges res;
        res.request.revision = static_cast<unsigned long>(env_->getRevision());
        if (get_monitored_environment_changes_client_.call(res))
        {
          if (!tesseract_rosutils::processMsg(*env_, res.response.commands))
          {
            RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                                   "newEnvironmentStateCallback: Failed to apply monitored environments changes.");
          }
        }
        else
        {
          RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                                 "newEnvironmentStateCallback: Failed to get monitored environments changes.");
        }
      }
      else if (static_cast<int>(env->revision) < env_->getRevision())
      {
        if (monitored_environment_mode_ == MonitoredEnvironmentMode::DEFAULT)
        {
          // If the monitored environment has a lower revision it is reset and additional changes are requested and
          // applied.
          if (env_->reset())
          {
            if (static_cast<int>(env->revision) > env_->getRevision())
            {
              tesseract_msgs::GetEnvironmentChanges res;
              res.request.revision = static_cast<unsigned long>(env_->getRevision());
              if (get_monitored_environment_changes_client_.call(res))
              {
                if (!tesseract_rosutils::processMsg(*env_, res.response.commands))
                {
                  RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                                         "newEnvironmentStateCallback: Failed to apply monitored environments "
                                         "changes.");
                }
              }
              else
              {
                RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                                       "newEnvironmentStateCallback: Failed to get monitored environments changes.");
              }
            }
          }
          else
          {
            RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                                   "newEnvironmentStateCallback: Failed to reset the tesseract object!");
          }
        }
        else if (monitored_environment_mode_ == MonitoredEnvironmentMode::SYNCHRONIZED)
        {
          // If this has been modified it will push the changes to the monitored environment to keep them in sync
          tesseract_msgs::ModifyEnvironment res;
          res.request.id = env_->getName();
          res.request.revision = env->revision;
          if (tesseract_rosutils::toMsg(res.request.commands, env_->getCommandHistory(), env->revision))
          {
            bool status = modify_monitored_environment_client_.call(res);
            if (!status || !res.response.success)
            {
              RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                                     "newEnvironmentStateCallback: Failed to update monitored environment!");
            }
          }
          else
          {
            RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                                   "newEnvironmentStateCallback: Failed to convert latest changes to message and "
                                   "update "
                                   "monitored environment!");
          }
        }
        else
        {
          RCLCPP_ERROR_STREAM(node_->get_logger().get_child(monitor_namespace_),
                                 "newEnvironmentStateCallback: Unsupporte MonitoredEnvironmentMode!");
        }
      }
    }

    if (!tesseract_rosutils::isMsgEmpty(env->joint_state))
    {
      if (last_robot_motion_time_ != env->joint_state.header.stamp)
      {
        tesseract_rosutils::processMsg(env_, env->joint_state);
        last_robot_motion_time_ = env->joint_state.header.stamp;
      }
    }

    RCLCPP_DEBUG_STREAM(node_->get_logger().get_child(monitor_namespace_),
                           "environment update " << fmod(last_update_time_.seconds(), 10.)
                                                 << " robot stamp: " << fmod(last_robot_motion_time_.seconds(), 10.));
  }

  triggerEnvironmentUpdateEvent();
}
*/

bool EnvironmentMonitor::applyEnvironmentCommandsMessage(
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

  std::string old_scene_name;
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
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
          RCLCPP_ERROR(node_->get_logger(), "Failed to apply UPDATE_JOINT_STATE command!");
          result = false;
        }
      }
    }
  }

  triggerEnvironmentUpdateEvent();
  return result;
}

void EnvironmentMonitor::saveSceneGraphCallback(
  tesseract_msgs::srv::SaveSceneGraph::Request::SharedPtr req,
  tesseract_msgs::srv::SaveSceneGraph::Response::SharedPtr res)
{
  res->success = !(env_ == nullptr);
  env_->getSceneGraph()->saveDOT(req->filepath);
  res->id = env_->getName();
  res->revision = static_cast<unsigned long>(env_->getRevision());
}

bool EnvironmentMonitor::waitForCurrentState(const rclcpp::Time& t, double wait_time)
{
  if (t.nanoseconds() == 0)
    return false;
  rclcpp::Time start = node_->now();
  auto timeout = rclcpp::Duration::from_seconds(wait_time);

  RCLCPP_DEBUG(node_->get_logger().get_child(monitor_namespace_), "sync robot state to: %.3fs", fmod(t.seconds(), 10.));

  if (current_state_monitor_)
  {
    // Wait for next robot update in state monitor. Those updates are only
    // passed to PSM when robot actually moved!
    enforce_next_state_update_ = true;  // enforce potential updates to be directly applied
    bool success = current_state_monitor_->waitForCurrentState(t, wait_time);
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

    RCLCPP_WARN(node_->get_logger().get_child(monitor_namespace_), "Failed to fetch current robot state.");
    return false;
  }

  // Sometimes there is no state monitor. In this case state updates are
  // received as part of scene updates only.
  // However, scene updates are only published if the robot actually moves.
  // Hence we need a timeout!
  // As publishing planning scene updates is throttled (2Hz by default), a 1s
  // timeout is a suitable default.
  std::shared_lock<std::shared_mutex> lock(scene_update_mutex_);
  rclcpp::Time prev_robot_motion_time = last_robot_motion_time_;
  while (last_robot_motion_time_ < t &&  // Wait until the state update actually reaches the scene.
         timeout > rclcpp::Duration(0,0))
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger().get_child(monitor_namespace_),
        "last robot motion: " << (t - last_robot_motion_time_).seconds() << " ago");
    new_environment_update_condition_.wait_for(lock, std::chrono::nanoseconds(timeout.nanoseconds()));
    timeout = timeout - (node_->now() - start);  // compute remaining wait_time
  }
  bool success = last_robot_motion_time_ >= t;
  // suppress warning if we received an update at all
  if (!success && prev_robot_motion_time != last_robot_motion_time_)
    RCLCPP_WARN(node_->get_logger().get_child(monitor_namespace_),
                   "Maybe failed to update robot state, time diff: %.3fs",
                   (t - last_robot_motion_time_).seconds());

  RCLCPP_DEBUG_STREAM(node_->get_logger().get_child(monitor_namespace_),
                         "sync done: robot motion: " << (t - last_robot_motion_time_).seconds()
                                                     << " scene update: " << (t - last_update_time_).seconds());
  return success;
}

std::shared_lock<std::shared_mutex> EnvironmentMonitor::lockEnvironmentRead() const
{
  return std::shared_lock(scene_update_mutex_);
}
std::unique_lock<std::shared_mutex> EnvironmentMonitor::lockEnvironmentWrite() const
{
  return std::unique_lock(scene_update_mutex_);
}

void EnvironmentMonitor::startStateMonitor(const std::string& joint_states_topic, bool publish_tf)
{
  stopStateMonitor();
  if (env_)
  {
    if (!current_state_monitor_)
      current_state_monitor_.reset(new CurrentStateMonitor(env_, node_));

    current_state_monitor_->addUpdateCallback(
      std::bind(&EnvironmentMonitor::onJointStateUpdate, this, std::placeholders::_1));
    current_state_monitor_->startStateMonitor(joint_states_topic, publish_tf);

    {
      std::scoped_lock lock(state_pending_mutex_);
      // TODO
      /*
      if (dt_state_update_.nanoseconds() > 0)
        state_update_timer_.start();
      */
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger().get_child(monitor_namespace_),
      "Cannot monitor robot state because planning scene is not configured");
  }
}

void EnvironmentMonitor::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();

  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  // TODO
  // state_update_timer_.stop();
  {
    std::scoped_lock lock(state_pending_mutex_);
    state_update_pending_ = false;
  }
}

void EnvironmentMonitor::onJointStateUpdate(const sensor_msgs::msg::JointState& /* joint_state */)
{
  const rclcpp::Time n = node_->now();
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

void EnvironmentMonitor::updateJointStateTimerCallback()
{
  if (state_update_pending_)
  {
    bool update = false;

    rclcpp::Duration dt = node_->now() - last_robot_state_update_wall_time_;

    {
      // lock for access to dt_state_update_ and state_update_pending_
      std::scoped_lock lock(state_pending_mutex_);
      if (state_update_pending_ && dt >= dt_state_update_)
      {
        state_update_pending_ = false;
        last_robot_state_update_wall_time_ = node_->now();
        update = true;
        RCLCPP_DEBUG_STREAM(node_->get_logger().get_child(monitor_namespace_),
                               "performPendingStateUpdate: " << fmod(last_robot_state_update_wall_time_.seconds(), 10));
      }
    }

    // run the state update with state_pending_mutex_ unlocked
    if (update)
    {
      updateEnvironmentWithCurrentState();
      RCLCPP_DEBUG(node_->get_logger().get_child(monitor_namespace_), "performPendingStateUpdate done");
    }
  }
}

void EnvironmentMonitor::setStateUpdateFrequency(double hz)
{
  bool update = false;
  if (hz > std::numeric_limits<double>::epsilon())
  {
    std::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = rclcpp::Duration::from_seconds(1.0 / hz);
    // TODO
    //state_update_timer_.setPeriod(dt_state_update_);
    //state_update_timer_.start();
  }
  else
  {
    // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
    // TODO
    // state_update_timer_.stop();
    std::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = rclcpp::Duration(0,0);
    if (state_update_pending_)
      update = true;
  }
  RCLCPP_INFO(node_->get_logger().get_child(monitor_namespace_),
    "Updating internal planning scene state at most every %lf seconds", dt_state_update_.seconds());

  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::updateEnvironmentWithCurrentState()
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) &&
        (node_->now() - current_state_monitor_->getMonitorStartTime()).seconds() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(monitor_namespace_), *node_->get_clock(),
          1.0, "The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
    }

    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = last_robot_motion_time_ = current_state_monitor_->getCurrentStateTime();
      RCLCPP_DEBUG_STREAM(node_->get_logger().get_child(monitor_namespace_),
        "robot state update " << fmod(last_robot_motion_time_.seconds(), 10.));

      env_->setState(current_state_monitor_->getCurrentState().joints);
    }
    triggerEnvironmentUpdateEvent();
  }
  else
    RCLCPP_ERROR_THROTTLE(node_->get_logger().get_child(monitor_namespace_), *node_->get_clock(),
        1.0, "State monitor is not active. Unable to set the planning scene state");
}

void EnvironmentMonitor::addUpdateCallback(const std::function<void()>& fn)
{
  std::scoped_lock lock(update_lock_);
  if (fn)
    update_callbacks_.push_back(fn);
}

void EnvironmentMonitor::clearUpdateCallbacks()
{
  std::scoped_lock lock(update_lock_);
  update_callbacks_.clear();
}

void EnvironmentMonitor::setEnvironmentPublishingFrequency(double hz)
{
  publish_environment_frequency_ = hz;
  RCLCPP_DEBUG(node_->get_logger().get_child(monitor_namespace_),
                  "Maximum frquency for publishing an environment is now %lf Hz",
                  publish_environment_frequency_);
}

void EnvironmentMonitor::modifyEnvironmentCallback(
  tesseract_msgs::srv::ModifyEnvironment::Request::SharedPtr req,
  tesseract_msgs::srv::ModifyEnvironment::Response::SharedPtr res)
{
  if (req->append)
    res->success = applyEnvironmentCommandsMessage(req->id, env_->getRevision(), req->commands);
  else
    res->success = applyEnvironmentCommandsMessage(req->id, static_cast<int>(req->revision), req->commands);

  res->revision = static_cast<unsigned long>(env_->getRevision());
}

void EnvironmentMonitor::getEnvironmentChangesCallback(
  tesseract_msgs::srv::GetEnvironmentChanges::Request::SharedPtr req,
  tesseract_msgs::srv::GetEnvironmentChanges::Response::SharedPtr res)
{
  auto lock_read = lockEnvironmentRead();

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
}

void EnvironmentMonitor::getEnvironmentInformationCallback(
  tesseract_msgs::srv::GetEnvironmentInformation::Request::SharedPtr req,
  tesseract_msgs::srv::GetEnvironmentInformation::Response::SharedPtr res)
{
  auto lock_read = lockEnvironmentRead();

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
}

}  // namespace tesseract_monitoring
