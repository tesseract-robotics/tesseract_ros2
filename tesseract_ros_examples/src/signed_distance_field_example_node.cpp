/**
 * @file signed_distance_field_example_node.cpp
 * @brief Demonstrates the volumetric SignedDistanceField geometry over ROS 2.
 *
 * Builds an environment containing a link whose collision/visual geometry is a
 * SignedDistanceField (an analytic sphere sampled into a grid) plus a probe sphere,
 * publishes it through the environment monitor (so it renders in RViz as a voxel
 * cloud), and runs a discrete collision check to show the SDF narrowphase reporting
 * the penetration distance. This exercises the full SDF-over-ROS path: creation ->
 * tesseract_rosutils message conversion -> monitor publish -> tesseract_rviz render.
 *
 * @copyright Copyright (c) 2026, Tesseract
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

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tesseract_monitoring/environment_monitor.h>

#include <tesseract/common/resource_locator.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/environment/environment.h>
#include <tesseract/collision/common.h>
#include <tesseract/collision/types.h>
#include <tesseract/collision/discrete_contact_manager.h>

using tesseract::scene_graph::Collision;
using tesseract::scene_graph::Joint;
using tesseract::scene_graph::JointType;
using tesseract::scene_graph::Link;
using tesseract::scene_graph::SceneGraph;
using tesseract::scene_graph::Visual;

/** @brief RViz Example Namespace */
static const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

/** @brief Build a scene graph: base_link -> sdf_link (sphere SDF) and base_link -> probe_link (sphere). */
static SceneGraph::Ptr buildSceneGraph()
{
  auto sg = std::make_shared<SceneGraph>();
  sg->setName("signed_distance_field_example");

  sg->addLink(Link("base_link"));

  // SDF link: f(p) = ||p|| - 0.5 over [-1, 1]^3, sampled at 16 cells per axis.
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  auto sdf = tesseract::geometry::createDiscreteSignedDistanceField(
      sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(16, 16, 16));

  Link sdf_link("sdf_link");
  auto sdf_visual = std::make_shared<Visual>();
  sdf_visual->geometry = sdf;
  sdf_link.visual.push_back(sdf_visual);
  auto sdf_collision = std::make_shared<Collision>();
  sdf_collision->geometry = sdf;
  sdf_link.collision.push_back(sdf_collision);
  sg->addLink(sdf_link);

  Joint sdf_joint("base_to_sdf");
  sdf_joint.type = JointType::FIXED;
  sdf_joint.parent_link_name = "base_link";
  sdf_joint.child_link_name = "sdf_link";
  sg->addJoint(sdf_joint);

  // Probe sphere placed just inside the SDF surface: SDF(0.45,0,0) = -0.05, so with a 0.1 radius
  // the reported contact distance is about -0.15 (penetrating).
  Link probe_link("probe_link");
  auto probe_visual = std::make_shared<Visual>();
  probe_visual->geometry = std::make_shared<tesseract::geometry::Sphere>(0.1);
  probe_link.visual.push_back(probe_visual);
  auto probe_collision = std::make_shared<Collision>();
  probe_collision->geometry = std::make_shared<tesseract::geometry::Sphere>(0.1);
  probe_link.collision.push_back(probe_collision);
  sg->addLink(probe_link);

  Joint probe_joint("base_to_probe");
  probe_joint.type = JointType::FIXED;
  probe_joint.parent_link_name = "base_link";
  probe_joint.child_link_name = "probe_link";
  probe_joint.parent_to_joint_origin_transform.translation().x() = 0.45;
  sg->addJoint(probe_joint);

  return sg;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("signed_distance_field_example_node");

  auto env = std::make_shared<tesseract::environment::Environment>();
  if (!env->init(*buildSceneGraph()))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize environment");
    return 1;
  }

  std::thread spinner{ [node]() { rclcpp::spin(node); } };

  // Publish the environment so RViz can render the SDF (voxel cloud) via tesseract_rviz.
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node, env, EXAMPLE_MONITOR_NAMESPACE);
  monitor->startPublishingEnvironment();

  // Run a discrete collision check between the SDF link and the probe sphere.
  auto manager = env->getDiscreteContactManager();
  if (manager == nullptr)
  {
    RCLCPP_WARN(node->get_logger(), "No discrete contact manager available; skipping collision check");
    spinner.join();
    return 0;
  }
  manager->setActiveCollisionObjects({ "sdf_link", "probe_link" });
  manager->setDefaultCollisionMargin(0.05);
  manager->setCollisionObjectsTransform(env->getState().link_transforms);

  tesseract::collision::ContactResultMap result;
  manager->contactTest(result, tesseract::collision::ContactRequest(tesseract::collision::ContactTestType::CLOSEST));

  tesseract::collision::ContactResultVector contacts;
  result.flattenMoveResults(contacts);

  if (contacts.empty())
  {
    RCLCPP_INFO(node->get_logger(), "SignedDistanceFieldExample: no contacts within margin");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(),
                "SignedDistanceFieldExample: %s vs %s, distance = %f",
                contacts[0].link_names[0].c_str(),
                contacts[0].link_names[1].c_str(),
                contacts[0].distance);
  }

  RCLCPP_INFO(node->get_logger(), "SignedDistanceFieldExample publishing; view 'sdf_link' in RViz. Ctrl-C to exit.");
  spinner.join();
  return 0;
}
