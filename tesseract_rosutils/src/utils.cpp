/**
 * @file utils.cpp
 * @brief Tesseract ROS utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2018, Southwest Research Institute
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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <octomap_msgs/conversions.h>
#include <tesseract_msgs/msg/string_limits_pair.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>
#include <console_bridge/console.h>
#include <filesystem>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/utils.h>

#include <tesseract_common/resource_locator.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/command.h>
#include <tesseract_environment/commands.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_srdf/kinematics_information.h>
#include <tesseract_common/plugin_info.h>

#include <tesseract_environment/utils.h>
#include <tesseract_environment/events.h>
#include <tesseract_environment/environment.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/scene_state.h>

#include <tesseract_srdf/srdf_model.h>
#include <tesseract_srdf/utils.h>

#include <tesseract_collision/core/common.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

#include <tesseract_task_composer/core/task_composer_node_info.h>

const std::string LOGGER_ID{ "tesseract_rosutils_utils" };
namespace tesseract_rosutils
{
std::shared_ptr<tesseract_common::Resource> ROSResourceLocator::locateResource(const std::string& url) const
{
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return nullptr;
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ament_index_cpp::get_package_share_directory(package);

    if (package_path.empty())
    {
      return nullptr;
    }

    mod_url = package_path + mod_url;
  }
  else if (url.find("file://") == 0)
  {
    mod_url.erase(0, strlen("file://"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return nullptr;
    }
  }

  std::filesystem::path mod_url_path(mod_url);
  if (!(mod_url_path.is_absolute() && mod_url_path.has_root_directory()))
    return nullptr;

  return std::make_shared<tesseract_common::SimpleLocatedResource>(
      url, mod_url, std::make_shared<ROSResourceLocator>(*this));
}

template <class Archive>
void ROSResourceLocator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("ResourceLocator",
                                     boost::serialization::base_object<tesseract_common::ResourceLocator>(*this));
}

bool isMsgEmpty(const sensor_msgs::msg::JointState& msg)
{
  return msg.name.empty() && msg.position.empty() && msg.velocity.empty() && msg.effort.empty();
}

bool isMsgEmpty(const tesseract_msgs::msg::TransformMap& msg) { return (msg.names.empty() || msg.transforms.empty()); }

bool isIdentical(const tesseract_scene_graph::Visual& /*visual1*/, const tesseract_scene_graph::Visual& /*visual2*/)
{
  assert(false);
  return false;
}

bool isIdentical(const tesseract_scene_graph::Collision& /*collision1*/,
                 const tesseract_scene_graph::Collision& /*collision2*/)
{
  assert(false);
  return false;
}

bool isIdentical(const tesseract_scene_graph::Link& link1, const tesseract_scene_graph::Link& link2)
{
  if (link1.getName() != link2.getName())
    return false;

  if (link1.collision.size() != link2.collision.size())
    return false;

  for (unsigned i = 0; i < link1.collision.size(); ++i)
  {
    if (!isIdentical(*link1.collision[i], *link2.collision[i]))
      return false;
  }

  // Check Visual
  if (link1.visual.size() != link2.visual.size())
    return false;

  for (unsigned i = 0; i < link1.visual.size(); ++i)
  {
    if (!isIdentical(*link1.visual[i], *link2.visual[i]))
      return false;
  }

  return true;
}

bool fromMsg(Eigen::Isometry3d& pose, const geometry_msgs::msg::Pose& pose_msg)
{
  tf2::fromMsg(pose_msg, pose);
  return true;
}

bool toMsg(geometry_msgs::msg::Pose& pose_msg, const Eigen::Isometry3d& pose)
{
  pose_msg = tf2::toMsg(pose);
  return true;
}

inline bool toMsg(tesseract_msgs::msg::Mesh& mesh_msgs, const tesseract_geometry::Geometry& geometry)
{
  switch (geometry.getType())
  {
    case tesseract_geometry::GeometryType::MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::Mesh&>(geometry);
      const tesseract_common::VectorVector3d& vertices = *(mesh.getVertices());
      mesh_msgs.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        mesh_msgs.vertices[i].x = vertices[i](0);
        mesh_msgs.vertices[i].y = vertices[i](1);
        mesh_msgs.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getFaces());
      mesh_msgs.faces.resize(static_cast<size_t>(faces.size()));
      for (size_t i = 0; i < static_cast<size_t>(faces.size()); ++i)
        mesh_msgs.faces[i] = static_cast<unsigned>(faces[static_cast<unsigned>(i)]);

      if (mesh.getResource() && mesh.getResource()->isFile())
      {
        mesh_msgs.file_path = mesh.getResource()->getFilePath();
      }
      if (mesh_msgs.file_path.empty())
      {
        mesh_msgs.scale[0] = 1;
        mesh_msgs.scale[1] = 1;
        mesh_msgs.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3f& scale = mesh.getScale().cast<float>();
        mesh_msgs.scale[0] = scale.x();
        mesh_msgs.scale[1] = scale.y();
        mesh_msgs.scale[2] = scale.z();
      }

      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::ConvexMesh&>(geometry);

      const tesseract_common::VectorVector3d& vertices = *(mesh.getVertices());
      mesh_msgs.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        mesh_msgs.vertices[i].x = vertices[i](0);
        mesh_msgs.vertices[i].y = vertices[i](1);
        mesh_msgs.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getFaces());
      mesh_msgs.faces.resize(static_cast<size_t>(faces.size()));
      for (size_t i = 0; i < static_cast<size_t>(faces.size()); ++i)
        mesh_msgs.faces[i] = static_cast<unsigned>(faces[static_cast<unsigned>(i)]);

      if (mesh.getResource() && mesh.getResource()->isFile())
      {
        mesh_msgs.file_path = mesh.getResource()->getFilePath();
      }
      if (mesh_msgs.file_path.empty())
      {
        mesh_msgs.scale[0] = 1;
        mesh_msgs.scale[1] = 1;
        mesh_msgs.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3f& scale = mesh.getScale().cast<float>();
        mesh_msgs.scale[0] = scale.x();
        mesh_msgs.scale[1] = scale.y();
        mesh_msgs.scale[2] = scale.z();
      }

      break;
    }
    case tesseract_geometry::GeometryType::SDF_MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::SDFMesh&>(geometry);

      const tesseract_common::VectorVector3d& vertices = *(mesh.getVertices());
      mesh_msgs.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        mesh_msgs.vertices[i].x = vertices[i](0);
        mesh_msgs.vertices[i].y = vertices[i](1);
        mesh_msgs.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getFaces());
      mesh_msgs.faces.resize(static_cast<size_t>(faces.size()));
      for (size_t i = 0; i < static_cast<size_t>(faces.size()); ++i)
        mesh_msgs.faces[i] = static_cast<unsigned>(faces[static_cast<unsigned>(i)]);

      if (mesh.getResource() && mesh.getResource()->isFile())
      {
        mesh_msgs.file_path = mesh.getResource()->getFilePath();
      }
      if (mesh_msgs.file_path.empty())
      {
        mesh_msgs.scale[0] = 1;
        mesh_msgs.scale[1] = 1;
        mesh_msgs.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3f& scale = mesh.getScale().cast<float>();
        mesh_msgs.scale[0] = scale.x();
        mesh_msgs.scale[1] = scale.y();
        mesh_msgs.scale[2] = scale.z();
      }

      break;
    }
    case tesseract_geometry::GeometryType::POLYGON_MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::PolygonMesh&>(geometry);

      const tesseract_common::VectorVector3d& vertices = *(mesh.getVertices());
      mesh_msgs.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        mesh_msgs.vertices[i].x = vertices[i](0);
        mesh_msgs.vertices[i].y = vertices[i](1);
        mesh_msgs.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getFaces());
      mesh_msgs.faces.resize(static_cast<size_t>(faces.size()));
      for (size_t i = 0; i < static_cast<size_t>(faces.size()); ++i)
        mesh_msgs.faces[i] = static_cast<unsigned>(faces[static_cast<unsigned>(i)]);

      if (mesh.getResource() && mesh.getResource()->isFile())
      {
        mesh_msgs.file_path = mesh.getResource()->getFilePath();
      }
      if (mesh_msgs.file_path.empty())
      {
        mesh_msgs.scale[0] = 1;
        mesh_msgs.scale[1] = 1;
        mesh_msgs.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3f& scale = mesh.getScale().cast<float>();
        mesh_msgs.scale[0] = scale.x();
        mesh_msgs.scale[1] = scale.y();
        mesh_msgs.scale[2] = scale.z();
      }

      break;
    }
    default:
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_ID),
                   "Unable to construct primitive shape message for shape of type %d",
                   static_cast<int>(geometry.getType()));
      return false;
    }
  }

  return true;
}

/** \brief Construct the message that corresponds to the shape. Return false on failure. */
bool toMsg(tesseract_msgs::msg::Geometry& geometry_msgs, const tesseract_geometry::Geometry& geometry)
{
  geometry_msgs.uuid = boost::uuids::to_string(geometry.getUUID());
  switch (geometry.getType())
  {
    case tesseract_geometry::GeometryType::SPHERE:
    {
      const auto& sphere = static_cast<const tesseract_geometry::Sphere&>(geometry);

      geometry_msgs.type = tesseract_msgs::msg::Geometry::SPHERE;
      geometry_msgs.sphere_radius = sphere.getRadius();
      break;
    }
    case tesseract_geometry::GeometryType::BOX:
    {
      const auto& box = static_cast<const tesseract_geometry::Box&>(geometry);

      geometry_msgs.type = tesseract_msgs::msg::Geometry::BOX;
      geometry_msgs.box_dimensions[0] = box.getX();
      geometry_msgs.box_dimensions[1] = box.getY();
      geometry_msgs.box_dimensions[2] = box.getZ();
      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      const auto& cylinder = static_cast<const tesseract_geometry::Cylinder&>(geometry);

      geometry_msgs.type = tesseract_msgs::msg::Geometry::CYLINDER;
      geometry_msgs.cylinder_dimensions[0] = cylinder.getRadius();
      geometry_msgs.cylinder_dimensions[1] = cylinder.getLength();
      break;
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      const auto& capsule = static_cast<const tesseract_geometry::Capsule&>(geometry);

      geometry_msgs.type = tesseract_msgs::msg::Geometry::CAPSULE;
      geometry_msgs.capsule_dimensions[0] = capsule.getRadius();
      geometry_msgs.capsule_dimensions[1] = capsule.getLength();
      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      const auto& cone = static_cast<const tesseract_geometry::Cone&>(geometry);

      geometry_msgs.type = tesseract_msgs::msg::Geometry::CONE;
      geometry_msgs.cone_dimensions[0] = cone.getRadius();
      geometry_msgs.cone_dimensions[1] = cone.getLength();
      break;
    }
    case tesseract_geometry::GeometryType::PLANE:
    {
      const auto& plane = static_cast<const tesseract_geometry::Plane&>(geometry);

      geometry_msgs.type = tesseract_msgs::msg::Geometry::PLANE;
      geometry_msgs.plane_coeff[0] = plane.getA();
      geometry_msgs.plane_coeff[1] = plane.getB();
      geometry_msgs.plane_coeff[2] = plane.getC();
      geometry_msgs.plane_coeff[3] = plane.getD();
      break;
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      const auto& octree = static_cast<const tesseract_geometry::Octree&>(geometry);

      geometry_msgs.type = tesseract_msgs::msg::Geometry::OCTREE;
      octomap_msgs::fullMapToMsg(*(octree.getOctree()), geometry_msgs.octomap);
      geometry_msgs.octomap_sub_type.type = static_cast<uint8_t>(octree.getSubType());
      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      geometry_msgs.type = tesseract_msgs::msg::Geometry::MESH;
      toMsg(geometry_msgs.mesh, geometry);
      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      geometry_msgs.type = tesseract_msgs::msg::Geometry::CONVEX_MESH;
      toMsg(geometry_msgs.mesh, geometry);
      break;
    }
    case tesseract_geometry::GeometryType::SDF_MESH:
    {
      geometry_msgs.type = tesseract_msgs::msg::Geometry::SDF_MESH;
      toMsg(geometry_msgs.mesh, geometry);
      break;
    }
    case tesseract_geometry::GeometryType::POLYGON_MESH:
    {
      geometry_msgs.type = tesseract_msgs::msg::Geometry::POLYGON_MESH;
      toMsg(geometry_msgs.mesh, geometry);
      break;
    }
    case tesseract_geometry::GeometryType::COMPOUND_MESH:
    {
      geometry_msgs.type = tesseract_msgs::msg::Geometry::COMPOUND_MESH;
      const auto& compound_mesh = static_cast<const tesseract_geometry::CompoundMesh&>(geometry);
      for (const auto& mesh : compound_mesh.getMeshes())
      {
        tesseract_msgs::msg::Mesh mesh_msg;
        toMsg(mesh_msg, *mesh);
        geometry_msgs.compound_mesh.push_back(mesh_msg);
      }

      const auto compound_mesh_type = compound_mesh.getMeshes().front()->getType();
      if (compound_mesh_type == tesseract_geometry::GeometryType::MESH)
        geometry_msgs.compound_mesh_type = tesseract_msgs::msg::Geometry::MESH;
      else if (compound_mesh_type == tesseract_geometry::GeometryType::CONVEX_MESH)
        geometry_msgs.compound_mesh_type = tesseract_msgs::msg::Geometry::CONVEX_MESH;
      else if (compound_mesh_type == tesseract_geometry::GeometryType::SDF_MESH)
        geometry_msgs.compound_mesh_type = tesseract_msgs::msg::Geometry::SDF_MESH;
      else if (compound_mesh_type == tesseract_geometry::GeometryType::POLYGON_MESH)
        geometry_msgs.compound_mesh_type = tesseract_msgs::msg::Geometry::POLYGON_MESH;
      else
        throw std::runtime_error("Invalid compound mesh type!");

      break;
    }
    default:
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_ID),
                   "Unable to construct primitive shape message for shape of type %d",
                   static_cast<int>(geometry.getType()));
      return false;
    }
  }

  return true;
}

inline std::shared_ptr<tesseract_geometry::PolygonMesh> fromMsg(uint8_t type, const tesseract_msgs::msg::Mesh& mesh_msg)
{
  if (type == tesseract_msgs::msg::Geometry::MESH)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>(mesh_msg.vertices.size());
    auto faces = std::make_shared<Eigen::VectorXi>(mesh_msg.faces.size());

    for (unsigned int i = 0; i < mesh_msg.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(mesh_msg.vertices[i].x, mesh_msg.vertices[i].y, mesh_msg.vertices[i].z);

    for (unsigned int i = 0; i < mesh_msg.faces.size(); ++i)
      (*faces)[static_cast<int>(i)] = static_cast<int>(mesh_msg.faces[i]);

    if (!mesh_msg.file_path.empty())
      return std::make_shared<tesseract_geometry::Mesh>(
          vertices,
          faces,
          std::make_shared<tesseract_common::SimpleLocatedResource>(mesh_msg.file_path, mesh_msg.file_path),
          Eigen::Vector3f(mesh_msg.scale[0], mesh_msg.scale[1], mesh_msg.scale[2]).cast<double>());

    return std::make_shared<tesseract_geometry::Mesh>(vertices, faces);
  }

  if (type == tesseract_msgs::msg::Geometry::CONVEX_MESH)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>(mesh_msg.vertices.size());
    auto faces = std::make_shared<Eigen::VectorXi>(mesh_msg.faces.size());

    for (unsigned int i = 0; i < mesh_msg.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(mesh_msg.vertices[i].x, mesh_msg.vertices[i].y, mesh_msg.vertices[i].z);

    for (unsigned int i = 0; i < mesh_msg.faces.size(); ++i)
      (*faces)[static_cast<int>(i)] = static_cast<int>(mesh_msg.faces[i]);

    if (!mesh_msg.file_path.empty())
      return std::make_shared<tesseract_geometry::ConvexMesh>(
          vertices,
          faces,
          std::make_shared<tesseract_common::SimpleLocatedResource>(mesh_msg.file_path, mesh_msg.file_path),
          Eigen::Vector3f(mesh_msg.scale[0], mesh_msg.scale[1], mesh_msg.scale[2]).cast<double>());

    return std::make_shared<tesseract_geometry::ConvexMesh>(vertices, faces);
  }

  if (type == tesseract_msgs::msg::Geometry::SDF_MESH)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>(mesh_msg.vertices.size());
    auto faces = std::make_shared<Eigen::VectorXi>(mesh_msg.faces.size());

    for (unsigned int i = 0; i < mesh_msg.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(mesh_msg.vertices[i].x, mesh_msg.vertices[i].y, mesh_msg.vertices[i].z);

    for (unsigned int i = 0; i < mesh_msg.faces.size(); ++i)
      (*faces)[static_cast<int>(i)] = static_cast<int>(mesh_msg.faces[i]);

    if (!mesh_msg.file_path.empty())
      return std::make_shared<tesseract_geometry::SDFMesh>(
          vertices,
          faces,
          std::make_shared<tesseract_common::SimpleLocatedResource>(mesh_msg.file_path, mesh_msg.file_path),
          Eigen::Vector3f(mesh_msg.scale[0], mesh_msg.scale[1], mesh_msg.scale[2]).cast<double>());

    return std::make_shared<tesseract_geometry::SDFMesh>(vertices, faces);
  }

  if (type == tesseract_msgs::msg::Geometry::POLYGON_MESH)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>(mesh_msg.vertices.size());
    auto faces = std::make_shared<Eigen::VectorXi>(mesh_msg.faces.size());

    for (unsigned int i = 0; i < mesh_msg.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(mesh_msg.vertices[i].x, mesh_msg.vertices[i].y, mesh_msg.vertices[i].z);

    for (unsigned int i = 0; i < mesh_msg.faces.size(); ++i)
      (*faces)[static_cast<int>(i)] = static_cast<int>(mesh_msg.faces[i]);

    if (!mesh_msg.file_path.empty())
      return std::make_shared<tesseract_geometry::PolygonMesh>(
          vertices,
          faces,
          std::make_shared<tesseract_common::SimpleLocatedResource>(mesh_msg.file_path, mesh_msg.file_path),
          Eigen::Vector3f(mesh_msg.scale[0], mesh_msg.scale[1], mesh_msg.scale[2]).cast<double>());

    return std::make_shared<tesseract_geometry::PolygonMesh>(vertices, faces);
  }

  return nullptr;
}

bool fromMsg(tesseract_geometry::Geometry::Ptr& geometry, const tesseract_msgs::msg::Geometry& geometry_msg)
{
  geometry = nullptr;
  if (geometry_msg.type == tesseract_msgs::msg::Geometry::SPHERE)
  {
    geometry = std::make_shared<tesseract_geometry::Sphere>(geometry_msg.sphere_radius);
  }
  else if (geometry_msg.type == tesseract_msgs::msg::Geometry::BOX)
  {
    geometry = std::make_shared<tesseract_geometry::Box>(
        geometry_msg.box_dimensions[0], geometry_msg.box_dimensions[1], geometry_msg.box_dimensions[2]);
  }
  else if (geometry_msg.type == tesseract_msgs::msg::Geometry::CYLINDER)
  {
    geometry = std::make_shared<tesseract_geometry::Cylinder>(geometry_msg.cylinder_dimensions[0],
                                                              geometry_msg.cylinder_dimensions[1]);
  }
  else if (geometry_msg.type == tesseract_msgs::msg::Geometry::CAPSULE)
  {
    geometry = std::make_shared<tesseract_geometry::Capsule>(geometry_msg.capsule_dimensions[0],
                                                             geometry_msg.capsule_dimensions[1]);
  }
  else if (geometry_msg.type == tesseract_msgs::msg::Geometry::CONE)
  {
    geometry =
        std::make_shared<tesseract_geometry::Cone>(geometry_msg.cone_dimensions[0], geometry_msg.cone_dimensions[1]);
  }
  else if (geometry_msg.type == tesseract_msgs::msg::Geometry::PLANE)
  {
    geometry = std::make_shared<tesseract_geometry::Plane>(geometry_msg.plane_coeff[0],
                                                           geometry_msg.plane_coeff[1],
                                                           geometry_msg.plane_coeff[2],
                                                           geometry_msg.plane_coeff[3]);
  }
  else if (geometry_msg.type == tesseract_msgs::msg::Geometry::MESH ||
           geometry_msg.type == tesseract_msgs::msg::Geometry::CONVEX_MESH ||
           geometry_msg.type == tesseract_msgs::msg::Geometry::SDF_MESH ||
           geometry_msg.type == tesseract_msgs::msg::Geometry::POLYGON_MESH)
  {
    geometry = fromMsg(geometry_msg.type, geometry_msg.mesh);
  }
  else if (geometry_msg.type == tesseract_msgs::msg::Geometry::OCTREE)
  {
    std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(geometry_msg.octomap)));
    auto sub_type = static_cast<tesseract_geometry::OctreeSubType>(geometry_msg.octomap_sub_type.type);
    geometry = std::make_shared<tesseract_geometry::Octree>(om, sub_type);
  }
  else if (geometry_msg.type == tesseract_msgs::msg::Geometry::COMPOUND_MESH)
  {
    std::vector<std::shared_ptr<tesseract_geometry::PolygonMesh>> meshes;
    meshes.reserve(geometry_msg.compound_mesh.size());
    for (const auto& mesh : geometry_msg.compound_mesh)
      meshes.push_back(fromMsg(geometry_msg.compound_mesh_type, mesh));

    geometry = std::make_shared<tesseract_geometry::CompoundMesh>(meshes);
  }

  if (geometry == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_ID),
                 "Unable to construct shape corresponding to shape_msg of type %d",
                 static_cast<int>(geometry_msg.type));
    return false;
  }

  geometry->setUUID(boost::lexical_cast<boost::uuids::uuid>(geometry_msg.uuid));

  return true;
}

bool toMsg(tesseract_msgs::msg::Material& material_msg, const tesseract_scene_graph::Material::Ptr& material)
{
  if (material == nullptr)
  {
    material_msg.empty = true;
    return true;
  }

  material_msg.name = material->getName();
  material_msg.texture_filename = material->texture_filename;
  material_msg.color.r = static_cast<float>(material->color(0));
  material_msg.color.g = static_cast<float>(material->color(1));
  material_msg.color.b = static_cast<float>(material->color(2));
  material_msg.color.a = static_cast<float>(material->color(3));
  return true;
}

bool fromMsg(tesseract_scene_graph::Material::Ptr& material, const tesseract_msgs::msg::Material& material_msg)
{
  if (material_msg.empty)
  {
    material = nullptr;
    return true;
  }

  material = std::make_shared<tesseract_scene_graph::Material>(material_msg.name);
  material->texture_filename = material_msg.texture_filename;
  material->color(0) = static_cast<double>(material_msg.color.r);
  material->color(1) = static_cast<double>(material_msg.color.g);
  material->color(2) = static_cast<double>(material_msg.color.b);
  material->color(3) = static_cast<double>(material_msg.color.a);
  return true;
}

bool toMsg(tesseract_msgs::msg::Inertial& inertial_msg, const tesseract_scene_graph::Inertial::Ptr& inertial)
{
  if (inertial == nullptr)
  {
    inertial_msg.empty = true;
    return true;
  }

  inertial_msg.origin = Eigen::toMsg(inertial->origin);

  inertial_msg.mass = inertial->mass;
  inertial_msg.ixx = inertial->ixx;
  inertial_msg.ixy = inertial->ixy;
  inertial_msg.ixz = inertial->ixz;
  inertial_msg.iyy = inertial->iyy;
  inertial_msg.iyz = inertial->iyz;
  inertial_msg.izz = inertial->izz;

  return true;
}

bool fromMsg(tesseract_scene_graph::Inertial::Ptr& inertial, const tesseract_msgs::msg::Inertial& inertial_msg)
{
  if (inertial_msg.empty)
  {
    inertial = nullptr;
    return true;
  }

  inertial = std::make_shared<tesseract_scene_graph::Inertial>();

  Eigen::fromMsg(inertial_msg.origin, inertial->origin);

  inertial->mass = inertial_msg.mass;
  inertial->ixx = inertial_msg.ixx;
  inertial->ixy = inertial_msg.ixy;
  inertial->ixz = inertial_msg.ixz;
  inertial->iyy = inertial_msg.iyy;
  inertial->iyz = inertial_msg.iyz;
  inertial->izz = inertial_msg.izz;

  return true;
}

bool toMsg(tesseract_msgs::msg::VisualGeometry& visual_msg, const tesseract_scene_graph::Visual& visual)
{
  visual_msg.name = visual.name;
  visual_msg.origin = Eigen::toMsg(visual.origin);
  toMsg(visual_msg.geometry, *(visual.geometry));
  toMsg(visual_msg.material, visual.material);
  return true;
}

bool fromMsg(tesseract_scene_graph::Visual::Ptr& visual, const tesseract_msgs::msg::VisualGeometry& visual_msg)
{
  visual = std::make_shared<tesseract_scene_graph::Visual>();
  visual->name = visual_msg.name;
  Eigen::fromMsg(visual_msg.origin, visual->origin);

  tesseract_geometry::Geometry::Ptr geom;
  fromMsg(geom, visual_msg.geometry);
  visual->geometry = geom;

  fromMsg(visual->material, visual_msg.material);

  return true;
}

bool toMsg(tesseract_msgs::msg::CollisionGeometry& collision_msg, const tesseract_scene_graph::Collision& collision)
{
  collision_msg.name = collision.name;
  collision_msg.origin = Eigen::toMsg(collision.origin);
  toMsg(collision_msg.geometry, *(collision.geometry));
  return true;
}

bool fromMsg(tesseract_scene_graph::Collision::Ptr& collision,
             const tesseract_msgs::msg::CollisionGeometry& collision_msg)
{
  collision = std::make_shared<tesseract_scene_graph::Collision>();
  collision->name = collision_msg.name;
  Eigen::fromMsg(collision_msg.origin, collision->origin);

  tesseract_geometry::Geometry::Ptr geom;
  fromMsg(geom, collision_msg.geometry);
  collision->geometry = geom;

  return true;
}

bool toMsg(tesseract_msgs::msg::Link& link_msg, const tesseract_scene_graph::Link& link)
{
  link_msg.name = link.getName();

  toMsg(link_msg.inertial, link.inertial);

  link_msg.collision.resize(link.collision.size());
  for (size_t i = 0; i < link.collision.size(); ++i)
    toMsg(link_msg.collision[i], *(link.collision[i]));

  link_msg.visual.resize(link.visual.size());
  for (size_t i = 0; i < link.visual.size(); ++i)
    toMsg(link_msg.visual[i], *(link.visual[i]));

  return true;
}

tesseract_scene_graph::Link fromMsg(const tesseract_msgs::msg::Link& link_msg)
{
  tesseract_scene_graph::Link link(link_msg.name);

  fromMsg(link.inertial, link_msg.inertial);

  link.collision.resize(link_msg.collision.size());
  for (size_t i = 0; i < link_msg.collision.size(); ++i)
    fromMsg(link.collision[i], link_msg.collision[i]);

  link.visual.resize(link_msg.visual.size());
  for (size_t i = 0; i < link_msg.visual.size(); ++i)
    fromMsg(link.visual[i], link_msg.visual[i]);

  return link;
}

bool toMsg(tesseract_msgs::msg::JointCalibration& joint_calibration_msg,
           const tesseract_scene_graph::JointCalibration::Ptr& joint_calibration)
{
  if (joint_calibration == nullptr)
  {
    joint_calibration_msg.empty = true;
    return true;
  }

  joint_calibration_msg.reference_position = joint_calibration->reference_position;

  joint_calibration_msg.rising = joint_calibration->rising;

  joint_calibration_msg.falling = joint_calibration->falling;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointCalibration::Ptr& joint_calibration,
             const tesseract_msgs::msg::JointCalibration& joint_calibration_msg)
{
  if (joint_calibration_msg.empty)
  {
    joint_calibration = nullptr;
    return true;
  }
  joint_calibration = std::make_shared<tesseract_scene_graph::JointCalibration>();

  joint_calibration->reference_position = joint_calibration_msg.reference_position;

  joint_calibration->rising = joint_calibration_msg.rising;

  joint_calibration->falling = joint_calibration_msg.falling;

  return true;
}

bool toMsg(tesseract_msgs::msg::JointDynamics& joint_dynamics_msg,
           const tesseract_scene_graph::JointDynamics::Ptr& joint_dynamics)
{
  if (joint_dynamics == nullptr)
  {
    joint_dynamics_msg.empty = true;
    return true;
  }

  joint_dynamics_msg.damping = joint_dynamics->damping;

  joint_dynamics_msg.friction = joint_dynamics->friction;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointDynamics::Ptr& joint_dynamics,
             const tesseract_msgs::msg::JointDynamics& joint_dynamics_msg)
{
  if (joint_dynamics_msg.empty)
  {
    joint_dynamics = nullptr;
    return true;
  }

  joint_dynamics = std::make_shared<tesseract_scene_graph::JointDynamics>();

  joint_dynamics->damping = joint_dynamics_msg.damping;

  joint_dynamics->friction = joint_dynamics_msg.friction;

  return true;
}

bool toMsg(tesseract_msgs::msg::JointLimits& joint_limits_msg,
           const tesseract_scene_graph::JointLimits::Ptr& joint_limits)
{
  if (joint_limits == nullptr)
  {
    joint_limits_msg.empty = true;
    return true;
  }

  joint_limits_msg.lower = joint_limits->lower;

  joint_limits_msg.upper = joint_limits->upper;

  joint_limits_msg.effort = joint_limits->effort;

  joint_limits_msg.velocity = joint_limits->velocity;

  joint_limits_msg.acceleration = joint_limits->acceleration;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointLimits::Ptr& joint_limits,
             const tesseract_msgs::msg::JointLimits& joint_limits_msg)
{
  if (joint_limits_msg.empty)
  {
    joint_limits = nullptr;
    return true;
  }

  joint_limits = std::make_shared<tesseract_scene_graph::JointLimits>();

  joint_limits->lower = joint_limits_msg.lower;

  joint_limits->upper = joint_limits_msg.upper;

  joint_limits->effort = joint_limits_msg.effort;

  joint_limits->velocity = joint_limits_msg.velocity;

  joint_limits->acceleration = joint_limits_msg.acceleration;

  return true;
}

bool toMsg(tesseract_msgs::msg::JointMimic& joint_mimic_msg, const tesseract_scene_graph::JointMimic::Ptr& joint_mimic)
{
  if (joint_mimic == nullptr)
  {
    joint_mimic_msg.empty = true;
    return true;
  }

  joint_mimic_msg.offset = joint_mimic->offset;

  joint_mimic_msg.multiplier = joint_mimic->multiplier;

  joint_mimic_msg.joint_name = joint_mimic->joint_name;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointMimic::Ptr& joint_mimic,
             const tesseract_msgs::msg::JointMimic& joint_mimic_msg)
{
  if (joint_mimic_msg.empty)
  {
    joint_mimic = nullptr;
    return true;
  }

  joint_mimic = std::make_shared<tesseract_scene_graph::JointMimic>();

  joint_mimic->offset = joint_mimic_msg.offset;

  joint_mimic->multiplier = joint_mimic_msg.multiplier;

  joint_mimic->joint_name = joint_mimic_msg.joint_name;

  return true;
}

bool toMsg(tesseract_msgs::msg::JointSafety& joint_safety_msg,
           const tesseract_scene_graph::JointSafety::Ptr& joint_safety)
{
  if (joint_safety == nullptr)
  {
    joint_safety_msg.empty = true;
    return true;
  }

  joint_safety_msg.soft_upper_limit = joint_safety->soft_upper_limit;

  joint_safety_msg.soft_lower_limit = joint_safety->soft_lower_limit;

  joint_safety_msg.k_position = joint_safety->k_position;

  joint_safety_msg.k_velocity = joint_safety->k_velocity;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointSafety::Ptr& joint_safety,
             const tesseract_msgs::msg::JointSafety& joint_safety_msg)
{
  if (joint_safety_msg.empty)
  {
    joint_safety = nullptr;
    return true;
  }

  joint_safety = std::make_shared<tesseract_scene_graph::JointSafety>();

  joint_safety->soft_upper_limit = joint_safety_msg.soft_upper_limit;

  joint_safety->soft_lower_limit = joint_safety_msg.soft_lower_limit;

  joint_safety->k_position = joint_safety_msg.k_position;

  joint_safety->k_velocity = joint_safety_msg.k_velocity;

  return true;
}

bool toMsg(tesseract_msgs::msg::Joint& joint_msg, const tesseract_scene_graph::Joint& joint)
{
  joint_msg.name = joint.getName();
  joint_msg.type = static_cast<unsigned char>(joint.type);

  joint_msg.axis[0] = joint.axis[0];
  joint_msg.axis[1] = joint.axis[1];
  joint_msg.axis[2] = joint.axis[2];

  joint_msg.child_link_name = joint.child_link_name;
  joint_msg.parent_link_name = joint.parent_link_name;

  joint_msg.parent_to_joint_origin_transform = Eigen::toMsg(joint.parent_to_joint_origin_transform);

  bool success = true;
  if (!toMsg(joint_msg.limits, joint.limits))
    success = false;

  if (!toMsg(joint_msg.dynamics, joint.dynamics))
    success = false;

  if (!toMsg(joint_msg.safety, joint.safety))
    success = false;

  if (!toMsg(joint_msg.calibration, joint.calibration))
    success = false;

  if (!toMsg(joint_msg.mimic, joint.mimic))
    success = false;

  return success;
}

tesseract_scene_graph::Joint fromMsg(const tesseract_msgs::msg::Joint& joint_msg)
{
  tesseract_scene_graph::Joint joint(joint_msg.name);

  joint.type = static_cast<tesseract_scene_graph::JointType>(joint_msg.type);

  joint.axis[0] = joint_msg.axis[0];
  joint.axis[1] = joint_msg.axis[1];
  joint.axis[2] = joint_msg.axis[2];

  joint.child_link_name = joint_msg.child_link_name;
  joint.parent_link_name = joint_msg.parent_link_name;

  Eigen::fromMsg(joint_msg.parent_to_joint_origin_transform, joint.parent_to_joint_origin_transform);
  fromMsg(joint.limits, joint_msg.limits);
  fromMsg(joint.dynamics, joint_msg.dynamics);
  fromMsg(joint.safety, joint_msg.safety);
  fromMsg(joint.calibration, joint_msg.calibration);
  fromMsg(joint.mimic, joint_msg.mimic);

  return joint;
}

tesseract_common::PairsCollisionMarginData
fromMsg(const std::vector<tesseract_msgs::msg::ContactMarginPair>& contact_margin_pairs_msg)
{
  tesseract_common::PairsCollisionMarginData contact_margin_pairs;

  for (const auto& pair : contact_margin_pairs_msg)
  {
    tesseract_common::LinkNamesPair lp;
    lp.first = pair.first.first;
    lp.second = pair.first.second;

    contact_margin_pairs.emplace(lp, pair.second);
  }
  return contact_margin_pairs;
}

std::vector<tesseract_msgs::msg::ContactMarginPair>
toMsg(const tesseract_common::PairsCollisionMarginData& contact_margin_pairs)
{
  std::vector<tesseract_msgs::msg::ContactMarginPair> contact_margin_pairs_msg;
  for (const auto& pair : contact_margin_pairs)
  {
    tesseract_msgs::msg::ContactMarginPair cmp;
    cmp.first.first = pair.first.first;
    cmp.first.second = pair.first.second;
    cmp.second = pair.second;
    contact_margin_pairs_msg.push_back(cmp);
  }

  return contact_margin_pairs_msg;
}

tesseract_common::CollisionMarginData fromMsg(const tesseract_msgs::msg::CollisionMarginData& contact_margin_data_msg)
{
  tesseract_common::CollisionMarginPairData pair_data(fromMsg(contact_margin_data_msg.margin_pairs));
  return { contact_margin_data_msg.default_margin, pair_data };
}

tesseract_msgs::msg::CollisionMarginData toMsg(const tesseract_common::CollisionMarginData& contact_margin_data)
{
  tesseract_msgs::msg::CollisionMarginData contact_margin_data_msg;
  contact_margin_data_msg.default_margin = contact_margin_data.getDefaultCollisionMargin();
  for (const auto& pair : contact_margin_data.getCollisionMarginPairData().getCollisionMargins())
  {
    tesseract_msgs::msg::ContactMarginPair cmp;
    cmp.first.first = pair.first.first;
    cmp.first.second = pair.first.second;
    cmp.second = pair.second;
    contact_margin_data_msg.margin_pairs.push_back(cmp);
  }
  return contact_margin_data_msg;
}

tesseract_common::CollisionMarginPairOverrideType
fromMsg(const tesseract_msgs::msg::CollisionMarginPairOverrideType& contact_margin_pair_override_type_msg)
{
  switch (contact_margin_pair_override_type_msg.type)
  {
    case tesseract_msgs::msg::CollisionMarginPairOverrideType::MODIFY:
    {
      return tesseract_common::CollisionMarginPairOverrideType::MODIFY;
    }
    case tesseract_msgs::msg::CollisionMarginPairOverrideType::REPLACE:
    {
      return tesseract_common::CollisionMarginPairOverrideType::REPLACE;
    }
    case tesseract_msgs::msg::CollisionMarginPairOverrideType::NONE:
    {
      return tesseract_common::CollisionMarginPairOverrideType::NONE;
    }
    default:
    {
      throw std::runtime_error("fromMsg: Invalid CollisionMarginPairOverrideType!");
    }
  }
}

tesseract_msgs::msg::CollisionMarginPairOverrideType
toMsg(const tesseract_common::CollisionMarginPairOverrideType& contact_margin_pair_override_type)
{
  tesseract_msgs::msg::CollisionMarginPairOverrideType contact_margin_pair_override_type_msg;
  switch (static_cast<int>(contact_margin_pair_override_type))
  {
    case static_cast<int>(tesseract_collision::CollisionMarginPairOverrideType::MODIFY):
    {
      contact_margin_pair_override_type_msg.type = tesseract_msgs::msg::CollisionMarginPairOverrideType::MODIFY;
      break;
    }
    case static_cast<int>(tesseract_collision::CollisionMarginPairOverrideType::REPLACE):
    {
      contact_margin_pair_override_type_msg.type = tesseract_msgs::msg::CollisionMarginPairOverrideType::REPLACE;
      break;
    }
    case static_cast<int>(tesseract_collision::CollisionMarginPairOverrideType::NONE):
    {
      contact_margin_pair_override_type_msg.type = tesseract_msgs::msg::CollisionMarginPairOverrideType::NONE;
      break;
    }
    default:
    {
      throw std::runtime_error("Unsupported CollisionMarginPairOverrideType");
    }
  }
  return contact_margin_pair_override_type_msg;
}

bool toMsg(std::vector<tesseract_msgs::msg::AllowedCollisionEntry>& acm_msg,
           const tesseract_common::AllowedCollisionMatrix& acm)
{
  for (const auto& entry : acm.getAllAllowedCollisions())
  {
    tesseract_msgs::msg::AllowedCollisionEntry entry_msg;
    entry_msg.link_1 = entry.first.first;
    entry_msg.link_2 = entry.first.second;
    entry_msg.reason = entry.second;
    acm_msg.push_back(entry_msg);
  }

  return true;
}

void toMsg(tesseract_msgs::msg::SceneGraph& scene_graph_msg, const tesseract_scene_graph::SceneGraph& scene_graph)
{
  scene_graph_msg.id = scene_graph.getName();
  scene_graph_msg.root = scene_graph.getRoot();

  for (const auto& link : scene_graph.getLinks())
  {
    tesseract_msgs::msg::Link link_msg;
    toMsg(link_msg, *link);
    scene_graph_msg.links.push_back(link_msg);
    if (!scene_graph.getLinkVisibility(link->getName()))
      scene_graph_msg.invisible_links.push_back(link->getName());

    if (!scene_graph.getLinkCollisionEnabled(link->getName()))
      scene_graph_msg.disabled_collision_links.push_back(link->getName());
  }

  for (const auto& joint : scene_graph.getJoints())
  {
    tesseract_msgs::msg::Joint joint_msg;
    toMsg(joint_msg, *joint);
    scene_graph_msg.joints.push_back(joint_msg);
  }

  toMsg(scene_graph_msg.acm, *scene_graph.getAllowedCollisionMatrix());
}

tesseract_scene_graph::SceneGraph fromMsg(const tesseract_msgs::msg::SceneGraph& scene_graph_msg)
{
  tesseract_scene_graph::SceneGraph g(scene_graph_msg.id);

  for (const auto& link_msg : scene_graph_msg.links)
    g.addLink(fromMsg(link_msg));

  for (const auto& joint_msg : scene_graph_msg.joints)
    g.addJoint(fromMsg(joint_msg));

  g.setRoot(scene_graph_msg.root);

  for (const auto& link_name : scene_graph_msg.invisible_links)
    g.setLinkVisibility(link_name, false);

  for (const auto& link_name : scene_graph_msg.disabled_collision_links)
    g.setLinkCollisionEnabled(link_name, false);

  for (const auto& entry : scene_graph_msg.acm)
    g.getAllowedCollisionMatrix()->addAllowedCollision(entry.link_1, entry.link_2, entry.reason);

  return g;
}

bool toMsg(tesseract_msgs::msg::EnvironmentCommand& command_msg, const tesseract_environment::Command& command)
{
  switch (command.getType())
  {
    case tesseract_environment::CommandType::ADD_LINK:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::ADD_LINK;
      const auto& cmd = static_cast<const tesseract_environment::AddLinkCommand&>(command);
      tesseract_rosutils::toMsg(command_msg.add_link, *(cmd.getLink()));
      command_msg.add_replace_allowed = cmd.replaceAllowed();

      if (cmd.getJoint())
        tesseract_rosutils::toMsg(command_msg.add_joint, *(cmd.getJoint()));

      return true;
    }
    case tesseract_environment::CommandType::MOVE_LINK:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::MOVE_LINK;
      const auto& cmd = static_cast<const tesseract_environment::MoveLinkCommand&>(command);
      tesseract_rosutils::toMsg(command_msg.move_link_joint, *(cmd.getJoint()));
      return true;
    }
    case tesseract_environment::CommandType::MOVE_JOINT:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::MOVE_JOINT;
      const auto& cmd = static_cast<const tesseract_environment::MoveJointCommand&>(command);
      command_msg.move_joint_name = cmd.getJointName();
      command_msg.move_joint_parent_link = cmd.getParentLink();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_LINK:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::REMOVE_LINK;
      const auto& cmd = static_cast<const tesseract_environment::RemoveLinkCommand&>(command);
      command_msg.remove_link = cmd.getLinkName();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_JOINT:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::REMOVE_JOINT;
      const auto& cmd = static_cast<const tesseract_environment::RemoveJointCommand&>(command);
      command_msg.remove_joint = cmd.getJointName();
      return true;
    }
    case tesseract_environment::CommandType::REPLACE_JOINT:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::REPLACE_JOINT;
      const auto& cmd = static_cast<const tesseract_environment::ReplaceJointCommand&>(command);
      tesseract_rosutils::toMsg(command_msg.replace_joint, *(cmd.getJoint()));
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
    {
      assert(false);
      return false;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_ORIGIN;
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointOriginCommand&>(command);
      command_msg.change_joint_origin_name = cmd.getJointName();
      command_msg.change_joint_origin_pose = Eigen::toMsg(cmd.getOrigin());
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED;
      const auto& cmd = static_cast<const tesseract_environment::ChangeLinkCollisionEnabledCommand&>(command);
      command_msg.change_link_collision_enabled_name = cmd.getLinkName();
      command_msg.change_link_collision_enabled_value = cmd.getEnabled();
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::CHANGE_LINK_VISIBILITY;
      const auto& cmd = static_cast<const tesseract_environment::ChangeLinkVisibilityCommand&>(command);
      command_msg.change_link_visibility_name = cmd.getLinkName();
      command_msg.change_link_visibility_value = cmd.getEnabled();
      return true;
    }
    case tesseract_environment::CommandType::MODIFY_ALLOWED_COLLISIONS:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::MODIFY_ALLOWED_COLLISIONS;
      const auto& cmd = static_cast<const tesseract_environment::ModifyAllowedCollisionsCommand&>(command);
      command_msg.modify_allowed_collisions_type = static_cast<uint8_t>(cmd.getModifyType());
      toMsg(command_msg.modify_allowed_collisions, cmd.getAllowedCollisionMatrix());
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK;
      const auto& cmd = static_cast<const tesseract_environment::RemoveAllowedCollisionLinkCommand&>(command);
      command_msg.remove_allowed_collision_link = cmd.getLinkName();
      return true;
    }
    case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::ADD_SCENE_GRAPH;
      const auto& cmd = static_cast<const tesseract_environment::AddSceneGraphCommand&>(command);

      toMsg(command_msg.scene_graph, *cmd.getSceneGraph());
      if (cmd.getJoint() != nullptr)
        toMsg(command_msg.scene_graph_joint, *cmd.getJoint());

      command_msg.scene_graph_prefix = cmd.getPrefix();
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_POSITION_LIMITS:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_POSITION_LIMITS;
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointPositionLimitsCommand&>(command);
      for (const auto& limits : cmd.getLimits())
      {
        tesseract_msgs::msg::StringLimitsPair pair;
        pair.first = limits.first;
        pair.second[0] = limits.second.first;
        pair.second[1] = limits.second.second;
        command_msg.change_joint_position_limits.push_back(pair);
      }

      return true;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_VELOCITY_LIMITS;
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointVelocityLimitsCommand&>(command);
      for (const auto& limits : cmd.getLimits())
      {
        tesseract_msgs::msg::StringDoublePair pair;
        pair.first = limits.first;
        pair.second = limits.second;
        command_msg.change_joint_velocity_limits.push_back(pair);
      }

      return true;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_ACCELERATION_LIMITS;
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointAccelerationLimitsCommand&>(command);
      for (const auto& limits : cmd.getLimits())
      {
        tesseract_msgs::msg::StringDoublePair pair;
        pair.first = limits.first;
        pair.second = limits.second;
        command_msg.change_joint_acceleration_limits.push_back(pair);
      }

      return true;
    }
    case tesseract_environment::CommandType::ADD_KINEMATICS_INFORMATION:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::ADD_KINEMATICS_INFORMATION;
      const auto& cmd = static_cast<const tesseract_environment::AddKinematicsInformationCommand&>(command);
      return toMsg(command_msg.add_kinematics_information, cmd.getKinematicsInformation());
    }
    case tesseract_environment::CommandType::CHANGE_COLLISION_MARGINS:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::CHANGE_COLLISION_MARGINS;
      const auto& cmd = static_cast<const tesseract_environment::ChangeCollisionMarginsCommand&>(command);

      // Process default margin
      std::optional<double> default_margin = cmd.getDefaultCollisionMargin();
      command_msg.has_collision_default_margin = default_margin.has_value();
      if (default_margin.has_value())
        command_msg.collision_default_margin = default_margin.value();

      // Process pair margin data
      command_msg.collision_margin_pair_data = toMsg(cmd.getCollisionMarginPairData().getCollisionMargins());
      command_msg.collision_margin_pair_override_type = toMsg(cmd.getCollisionMarginPairOverrideType());
      return true;
    }
    case tesseract_environment::CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::ADD_CONTACT_MANAGERS_PLUGIN_INFO;
      const auto& cmd = static_cast<const tesseract_environment::AddContactManagersPluginInfoCommand&>(command);
      command_msg.add_contact_managers_plugin_info = toMsg(cmd.getContactManagersPluginInfo());
      return true;
    }
    case tesseract_environment::CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::SET_ACTIVE_DISCRETE_CONTACT_MANAGER;
      const auto& cmd = static_cast<const tesseract_environment::SetActiveDiscreteContactManagerCommand&>(command);
      command_msg.set_active_discrete_contact_manager = cmd.getName();
      return true;
    }
    case tesseract_environment::CommandType::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER;
      const auto& cmd = static_cast<const tesseract_environment::SetActiveContinuousContactManagerCommand&>(command);
      command_msg.set_active_continuous_contact_manager = cmd.getName();
      return true;
    }
    case tesseract_environment::CommandType::ADD_TRAJECTORY_LINK:
    {
      command_msg.command = tesseract_msgs::msg::EnvironmentCommand::ADD_TRAJECTORY_LINK;
      const auto& cmd = static_cast<const tesseract_environment::AddTrajectoryLinkCommand&>(command);
      command_msg.add_trajectory_link_name = cmd.getLinkName();
      command_msg.add_trajectory_link_parent_name = cmd.getParentLinkName();
      toMsg(command_msg.add_trajectory_link_traj, cmd.getTrajectory());
      command_msg.add_trajectory_link_replace_allowed = cmd.replaceAllowed();
      return true;
    }
    default:
    {
      CONSOLE_BRIDGE_logWarn("Unhandled CommandType '%d' in toMsg", command.getType());
    }
  }

  return false;
}

bool toMsg(std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands_msg,
           const tesseract_environment::Commands& commands,
           unsigned long past_revision)
{
  for (unsigned long i = past_revision; i < commands.size(); ++i)
  {
    tesseract_msgs::msg::EnvironmentCommand command_msg;
    if (!tesseract_rosutils::toMsg(command_msg, *(commands[i])))
      return false;

    commands_msg.push_back(command_msg);
  }

  return true;
}

tesseract_environment::Commands fromMsg(const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands_msg)
{
  tesseract_environment::Commands commands;
  commands.reserve(commands_msg.size());
  for (const auto& command : commands_msg)
    commands.push_back(fromMsg(command));

  return commands;
}

tesseract_environment::Command::Ptr fromMsg(const tesseract_msgs::msg::EnvironmentCommand& command_msg)
{
  switch (command_msg.command)
  {
    case tesseract_msgs::msg::EnvironmentCommand::ADD_LINK:
    {
      tesseract_scene_graph::Link link = fromMsg(command_msg.add_link);
      if (command_msg.add_joint.name.empty() || command_msg.add_joint.type == 0)
        return std::make_shared<tesseract_environment::AddLinkCommand>(link, command_msg.add_replace_allowed);

      tesseract_scene_graph::Joint joint = fromMsg(command_msg.add_joint);
      return std::make_shared<tesseract_environment::AddLinkCommand>(link, joint, command_msg.add_replace_allowed);
    }
    case tesseract_msgs::msg::EnvironmentCommand::MOVE_LINK:
    {
      tesseract_scene_graph::Joint j = fromMsg(command_msg.move_link_joint);
      return std::make_shared<tesseract_environment::MoveLinkCommand>(j);
    }
    case tesseract_msgs::msg::EnvironmentCommand::MOVE_JOINT:
    {
      return std::make_shared<tesseract_environment::MoveJointCommand>(command_msg.move_joint_name,
                                                                       command_msg.move_joint_parent_link);
    }
    case tesseract_msgs::msg::EnvironmentCommand::REMOVE_LINK:
    {
      return std::make_shared<tesseract_environment::RemoveLinkCommand>(command_msg.remove_link);
    }
    case tesseract_msgs::msg::EnvironmentCommand::REMOVE_JOINT:
    {
      return std::make_shared<tesseract_environment::RemoveJointCommand>(command_msg.remove_joint);
    }
    case tesseract_msgs::msg::EnvironmentCommand::REPLACE_JOINT:
    {
      tesseract_scene_graph::Joint j = fromMsg(command_msg.replace_joint);
      return std::make_shared<tesseract_environment::ReplaceJointCommand>(j);
    }
    case tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_ORIGIN:
    {
      Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
      if (!fromMsg(pose, command_msg.change_joint_origin_pose))
        throw std::runtime_error("Failed to convert pose message to eigen");

      return std::make_shared<tesseract_environment::ChangeJointOriginCommand>(command_msg.change_joint_origin_name,
                                                                               pose);
    }
    case tesseract_msgs::msg::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED:
    {
      return std::make_shared<tesseract_environment::ChangeLinkCollisionEnabledCommand>(
          command_msg.change_link_collision_enabled_name, command_msg.change_link_collision_enabled_value);
    }
    case tesseract_msgs::msg::EnvironmentCommand::CHANGE_LINK_VISIBILITY:
    {
      return std::make_shared<tesseract_environment::ChangeLinkCollisionEnabledCommand>(
          command_msg.change_link_visibility_name, command_msg.change_link_visibility_value);
    }
    case tesseract_msgs::msg::EnvironmentCommand::MODIFY_ALLOWED_COLLISIONS:
    {
      tesseract_common::AllowedCollisionMatrix acm;
      for (const auto& entry : command_msg.modify_allowed_collisions)
        acm.addAllowedCollision(entry.link_1, entry.link_2, entry.reason);
      return std::make_shared<tesseract_environment::ModifyAllowedCollisionsCommand>(
          acm,
          static_cast<tesseract_environment::ModifyAllowedCollisionsType>(command_msg.modify_allowed_collisions_type));
    }
    case tesseract_msgs::msg::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK:
    {
      return std::make_shared<tesseract_environment::RemoveAllowedCollisionLinkCommand>(
          command_msg.remove_allowed_collision_link);
    }
    case tesseract_msgs::msg::EnvironmentCommand::ADD_SCENE_GRAPH:
    {
      if (command_msg.scene_graph_joint.name.empty() || command_msg.scene_graph_joint.type == 0)
      {
        return std::make_shared<tesseract_environment::AddSceneGraphCommand>(fromMsg(command_msg.scene_graph),
                                                                             command_msg.scene_graph_prefix);
      }
      tesseract_scene_graph::Joint j = fromMsg(command_msg.scene_graph_joint);

      return std::make_shared<tesseract_environment::AddSceneGraphCommand>(
          fromMsg(command_msg.scene_graph), j, command_msg.scene_graph_prefix);
    }
    case tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_POSITION_LIMITS:
    {
      std::unordered_map<std::string, std::pair<double, double>> limits_map;
      for (const auto& limits : command_msg.change_joint_position_limits)
        limits_map[limits.first] = std::make_pair(limits.second[0], limits.second[1]);

      return std::make_shared<tesseract_environment::ChangeJointPositionLimitsCommand>(limits_map);
    }
    case tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_VELOCITY_LIMITS:
    {
      std::unordered_map<std::string, double> limits_map;
      for (const auto& limits : command_msg.change_joint_velocity_limits)
        limits_map[limits.first] = limits.second;

      return std::make_shared<tesseract_environment::ChangeJointVelocityLimitsCommand>(limits_map);
    }
    case tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_ACCELERATION_LIMITS:
    {
      std::unordered_map<std::string, double> limits_map;
      for (const auto& limits : command_msg.change_joint_acceleration_limits)
        limits_map[limits.first] = limits.second;

      return std::make_shared<tesseract_environment::ChangeJointAccelerationLimitsCommand>(limits_map);
    }
    case tesseract_msgs::msg::EnvironmentCommand::ADD_KINEMATICS_INFORMATION:
    {
      tesseract_srdf::KinematicsInformation kin_info;
      fromMsg(kin_info, command_msg.add_kinematics_information);

      return std::make_shared<tesseract_environment::AddKinematicsInformationCommand>(kin_info);
    }
    case tesseract_msgs::msg::EnvironmentCommand::CHANGE_COLLISION_MARGINS:
    {
      tesseract_common::CollisionMarginPairData pair_margin_data(fromMsg(command_msg.collision_margin_pair_data));
      tesseract_common::CollisionMarginPairOverrideType pair_override_type =
          fromMsg(command_msg.collision_margin_pair_override_type);

      if (command_msg.has_collision_default_margin)
      {
        return std::make_shared<tesseract_environment::ChangeCollisionMarginsCommand>(
            command_msg.collision_default_margin, pair_margin_data, pair_override_type);
      }

      return std::make_shared<tesseract_environment::ChangeCollisionMarginsCommand>(pair_margin_data,
                                                                                    pair_override_type);
    }
    case tesseract_msgs::msg::EnvironmentCommand::ADD_CONTACT_MANAGERS_PLUGIN_INFO:
    {
      tesseract_common::ContactManagersPluginInfo info = fromMsg(command_msg.add_contact_managers_plugin_info);
      return std::make_shared<tesseract_environment::AddContactManagersPluginInfoCommand>(info);
    }
    case tesseract_msgs::msg::EnvironmentCommand::SET_ACTIVE_DISCRETE_CONTACT_MANAGER:
    {
      return std::make_shared<tesseract_environment::SetActiveDiscreteContactManagerCommand>(
          command_msg.set_active_discrete_contact_manager);
    }
    case tesseract_msgs::msg::EnvironmentCommand::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER:
    {
      return std::make_shared<tesseract_environment::SetActiveContinuousContactManagerCommand>(
          command_msg.set_active_continuous_contact_manager);
    }
    case tesseract_msgs::msg::EnvironmentCommand::ADD_TRAJECTORY_LINK:
    {
      tesseract_common::JointTrajectory traj = fromMsg(command_msg.add_trajectory_link_traj);
      return std::make_shared<tesseract_environment::AddTrajectoryLinkCommand>(
          command_msg.add_trajectory_link_name,
          command_msg.add_trajectory_link_parent_name,
          traj,
          command_msg.add_trajectory_link_replace_allowed);
    }
    default:
    {
      throw std::runtime_error("Unsupported command type " + std::to_string(command_msg.command));
    }
  }
}

void toMsg(tesseract_msgs::msg::EnvironmentState& state_msg,
           const tesseract_environment::Environment& env,
           bool include_joint_states)
{
  state_msg.id = env.getName();
  state_msg.revision = static_cast<unsigned long>(env.getRevision());

  if (include_joint_states)
  {
    tesseract_scene_graph::SceneState env_state = env.getState();
    toMsg(state_msg.joint_state, env_state.joints);
    toMsg(state_msg.floating_joint_states, env_state.floating_joints);
  }
}

void toMsg(const tesseract_msgs::msg::EnvironmentState::SharedPtr& state_msg,
           const tesseract_environment::Environment& env)
{
  toMsg(*state_msg, env);
}

void toMsg(tesseract_msgs::msg::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj)
{
  traj_msg.uuid = boost::uuids::to_string(traj.uuid);
  traj_msg.description = traj.description;
  for (const auto& js : traj)
  {
    assert(js.joint_names.size() == static_cast<unsigned>(js.position.size()));

    tesseract_msgs::msg::JointState js_msg;
    js_msg.joint_names = js.joint_names;
    js_msg.position.resize(static_cast<size_t>(js.position.size()));
    js_msg.velocity.resize(static_cast<size_t>(js.velocity.size()));
    js_msg.acceleration.resize(static_cast<size_t>(js.acceleration.size()));

    for (int i = 0; i < js.position.size(); ++i)
      js_msg.position[static_cast<size_t>(i)] = js.position(i);

    for (int i = 0; i < js.velocity.size(); ++i)
      js_msg.velocity[static_cast<size_t>(i)] = js.velocity(i);

    for (int i = 0; i < js.acceleration.size(); ++i)
      js_msg.acceleration[static_cast<size_t>(i)] = js.acceleration(i);

    js_msg.time_from_start = rclcpp::Duration::from_seconds(js.time);
    traj_msg.states.push_back(js_msg);
  }
}

tesseract_common::JointTrajectory fromMsg(const tesseract_msgs::msg::JointTrajectory& traj_msg)
{
  tesseract_common::JointTrajectory trajectory;
  trajectory.uuid = boost::lexical_cast<boost::uuids::uuid>(traj_msg.uuid);
  trajectory.description = traj_msg.description;
  for (const auto& js_msg : traj_msg.states)
  {
    assert(js_msg.joint_names.size() == static_cast<unsigned>(js_msg.position.size()));

    tesseract_common::JointState js;
    js.joint_names = js_msg.joint_names;
    js.position.resize(static_cast<long>(js_msg.position.size()));
    js.velocity.resize(static_cast<long>(js_msg.velocity.size()));
    js.acceleration.resize(static_cast<long>(js_msg.acceleration.size()));

    for (std::size_t i = 0; i < js_msg.position.size(); ++i)
      js.position(static_cast<long>(i)) = js_msg.position[i];

    for (std::size_t i = 0; i < js_msg.velocity.size(); ++i)
      js.velocity(static_cast<long>(i)) = js_msg.velocity[i];

    for (std::size_t i = 0; i < js_msg.acceleration.size(); ++i)
      js.acceleration(static_cast<long>(i)) = js_msg.acceleration[i];

    js.time = rclcpp::Duration(js_msg.time_from_start).seconds();
    trajectory.push_back(js);
  }
  return trajectory;
}

bool processMsg(tesseract_environment::Environment& env,
                const sensor_msgs::msg::JointState& joint_state_msg,
                const tesseract_msgs::msg::TransformMap& floating_joint_state_msg)
{
  const bool joint_state_msg_empty = isMsgEmpty(joint_state_msg);
  const bool floating_joint_state_msg_empty = isMsgEmpty(floating_joint_state_msg);

  if (joint_state_msg_empty && floating_joint_state_msg_empty)
    return false;

  tesseract_common::TransformMap floating_joints;
  if (!floating_joint_state_msg_empty)
    fromMsg(floating_joints, floating_joint_state_msg);

  if (!joint_state_msg_empty)
  {
    std::unordered_map<std::string, double> joints;
    for (auto i = 0U; i < joint_state_msg.name.size(); ++i)
      joints[joint_state_msg.name[i]] = joint_state_msg.position[i];
    env.setState(joints, floating_joints);
    return true;
  }

  env.setState(floating_joints);
  return true;
}

bool processMsg(tesseract_environment::Environment& env,
                const std::vector<tesseract_msgs::msg::EnvironmentCommand>& env_command_msg)
{
  tesseract_environment::Commands commands = fromMsg(env_command_msg);
  return env.applyCommands(commands);
}

void toMsg(tesseract_msgs::msg::ContactResult& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result,
           const rclcpp::Time& stamp)
{
  contact_result_msg.stamp = stamp;
  contact_result_msg.distance = contact_result.distance;
  contact_result_msg.type_id[0] = static_cast<unsigned char>(contact_result.type_id[0]);
  contact_result_msg.type_id[1] = static_cast<unsigned char>(contact_result.type_id[1]);
  contact_result_msg.link_names[0] = contact_result.link_names[0];
  contact_result_msg.link_names[1] = contact_result.link_names[1];
  contact_result_msg.shape_id[0] = static_cast<size_t>(contact_result.shape_id[0]);
  contact_result_msg.shape_id[1] = static_cast<size_t>(contact_result.shape_id[1]);
  contact_result_msg.subshape_id[0] = static_cast<size_t>(contact_result.subshape_id[0]);
  contact_result_msg.subshape_id[1] = static_cast<size_t>(contact_result.subshape_id[1]);
  contact_result_msg.normal.x = contact_result.normal[0];
  contact_result_msg.normal.y = contact_result.normal[1];
  contact_result_msg.normal.z = contact_result.normal[2];
  contact_result_msg.nearest_points[0].x = contact_result.nearest_points[0][0];
  contact_result_msg.nearest_points[0].y = contact_result.nearest_points[0][1];
  contact_result_msg.nearest_points[0].z = contact_result.nearest_points[0][2];
  contact_result_msg.nearest_points[1].x = contact_result.nearest_points[1][0];
  contact_result_msg.nearest_points[1].y = contact_result.nearest_points[1][1];
  contact_result_msg.nearest_points[1].z = contact_result.nearest_points[1][2];
  contact_result_msg.nearest_points_local[0].x = contact_result.nearest_points_local[0][0];
  contact_result_msg.nearest_points_local[0].y = contact_result.nearest_points_local[0][1];
  contact_result_msg.nearest_points_local[0].z = contact_result.nearest_points_local[0][2];
  contact_result_msg.nearest_points_local[1].x = contact_result.nearest_points_local[1][0];
  contact_result_msg.nearest_points_local[1].y = contact_result.nearest_points_local[1][1];
  contact_result_msg.nearest_points_local[1].z = contact_result.nearest_points_local[1][2];
  toMsg(contact_result_msg.transform[0], contact_result.transform[0]);
  toMsg(contact_result_msg.transform[1], contact_result.transform[1]);
  contact_result_msg.cc_time[0] = contact_result.cc_time[0];
  contact_result_msg.cc_time[1] = contact_result.cc_time[1];
  toMsg(contact_result_msg.cc_transform[0], contact_result.cc_transform[0]);
  toMsg(contact_result_msg.cc_transform[1], contact_result.cc_transform[1]);

  if (contact_result.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
    contact_result_msg.cc_type[0] = tesseract_msgs::msg::ContactResult::CCTYPE_TIME0;
  else if (contact_result.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
    contact_result_msg.cc_type[0] = tesseract_msgs::msg::ContactResult::CCTYPE_TIME1;
  else if (contact_result.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Between)
    contact_result_msg.cc_type[0] = tesseract_msgs::msg::ContactResult::CCTYPE_BETWEEN;
  else
    contact_result_msg.cc_type[0] = tesseract_msgs::msg::ContactResult::CCTYPE_NONE;

  if (contact_result.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
    contact_result_msg.cc_type[1] = tesseract_msgs::msg::ContactResult::CCTYPE_TIME0;
  else if (contact_result.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
    contact_result_msg.cc_type[1] = tesseract_msgs::msg::ContactResult::CCTYPE_TIME1;
  else if (contact_result.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Between)
    contact_result_msg.cc_type[1] = tesseract_msgs::msg::ContactResult::CCTYPE_BETWEEN;
  else
    contact_result_msg.cc_type[1] = tesseract_msgs::msg::ContactResult::CCTYPE_NONE;
}

void toMsg(tesseract_msgs::msg::ContactResult& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result)
{
  toMsg(contact_result_msg, contact_result, rclcpp::Clock{ RCL_ROS_TIME }.now());
}

void toMsg(const tesseract_msgs::msg::ContactResult::SharedPtr& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result)
{
  toMsg(*contact_result_msg, contact_result);
}

void toMsg(const tesseract_msgs::msg::ContactResult::SharedPtr& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result,
           const rclcpp::Time& stamp)
{
  toMsg(*contact_result_msg, contact_result, stamp);
}

tesseract_msgs::msg::KinematicsPluginInfo toMsg(const tesseract_common::KinematicsPluginInfo& info)
{
  tesseract_msgs::msg::KinematicsPluginInfo msg;
  msg.search_paths.insert(msg.search_paths.begin(), info.search_paths.begin(), info.search_paths.end());
  msg.search_libraries.insert(msg.search_libraries.begin(), info.search_libraries.begin(), info.search_libraries.end());
  for (const auto& pair : info.fwd_plugin_infos)
  {
    tesseract_msgs::msg::GroupsKinematicPlugins pair_msg;
    pair_msg.group = pair.first;
    pair_msg.plugin_container.default_plugin = pair.second.default_plugin;
    pair_msg.plugin_container.plugins = toMsg(pair.second.plugins);
    msg.group_fwd_plugins.push_back(pair_msg);
  }

  for (const auto& pair : info.inv_plugin_infos)
  {
    tesseract_msgs::msg::GroupsKinematicPlugins pair_msg;
    pair_msg.group = pair.first;
    pair_msg.plugin_container.default_plugin = pair.second.default_plugin;
    pair_msg.plugin_container.plugins = toMsg(pair.second.plugins);
    msg.group_inv_plugins.push_back(pair_msg);
  }
  return msg;
}

tesseract_msgs::msg::ContactManagersPluginInfo toMsg(const tesseract_common::ContactManagersPluginInfo& info)
{
  tesseract_msgs::msg::ContactManagersPluginInfo msg;
  msg.search_paths.insert(msg.search_paths.begin(), info.search_paths.begin(), info.search_paths.end());
  msg.search_libraries.insert(msg.search_libraries.begin(), info.search_libraries.begin(), info.search_libraries.end());
  msg.discrete_plugin_container.default_plugin = info.discrete_plugin_infos.default_plugin;
  msg.discrete_plugin_container.plugins = toMsg(info.discrete_plugin_infos.plugins);
  msg.continuous_plugin_container.default_plugin = info.continuous_plugin_infos.default_plugin;
  msg.continuous_plugin_container.plugins = toMsg(info.continuous_plugin_infos.plugins);
  return msg;
}

std::vector<tesseract_msgs::msg::StringPluginInfoPair> toMsg(const tesseract_common::PluginInfoMap& info_map)
{
  std::vector<tesseract_msgs::msg::StringPluginInfoPair> msg;
  for (const auto& pair : info_map)
  {
    tesseract_msgs::msg::StringPluginInfoPair pair_msg;
    pair_msg.first = pair.first;
    pair_msg.second = toMsg(pair.second);
    msg.push_back(pair_msg);
  }
  return msg;
}

tesseract_msgs::msg::PluginInfo toMsg(const tesseract_common::PluginInfo& info)
{
  tesseract_msgs::msg::PluginInfo msg;
  msg.class_name = info.class_name;

  if (info.config)
  {
    YAML::Emitter out;
    out << info.config;
    msg.config = out.c_str();
  }

  return msg;
}

bool toMsg(geometry_msgs::msg::PoseArray& pose_array, const tesseract_common::VectorIsometry3d& transforms)
{
  for (const auto& transform : transforms)
  {
    geometry_msgs::msg::Pose pose;
    pose = Eigen::toMsg(transform);
    pose_array.poses.push_back(pose);
  }

  return true;
}

tesseract_msgs::msg::ChainGroup toMsg(tesseract_srdf::ChainGroups::const_reference group)
{
  tesseract_msgs::msg::ChainGroup g;
  g.name = group.first;
  g.chains.reserve(group.second.size());
  for (const auto& pair : group.second)
  {
    tesseract_msgs::msg::StringPair chain;
    chain.first = pair.first;
    chain.second = pair.second;
    g.chains.push_back(chain);
  }
  return g;
}

tesseract_msgs::msg::GroupsJointStates toMsg(tesseract_srdf::GroupJointStates::const_reference group)
{
  tesseract_msgs::msg::GroupsJointStates g;
  g.name = group.first;

  g.joint_states.reserve(group.second.size());
  for (const auto& gs : group.second)
  {
    tesseract_msgs::msg::GroupsJointState gjs;
    gjs.name = gs.first;
    gjs.joint_state.reserve(gs.second.size());
    for (const auto& s : gs.second)
    {
      tesseract_msgs::msg::StringDoublePair js;
      js.first = s.first;
      js.second = s.second;
      gjs.joint_state.push_back(js);
    }
    g.joint_states.push_back(gjs);
  }

  return g;
}

tesseract_msgs::msg::GroupsTCPs toMsg(tesseract_srdf::GroupTCPs::const_reference group)
{
  tesseract_msgs::msg::GroupsTCPs g;
  g.name = group.first;

  g.tcps.reserve(group.second.size());
  for (const auto& gs : group.second)
  {
    tesseract_msgs::msg::GroupsTCP gtcp;
    gtcp.name = gs.first;
    toMsg(gtcp.tcp, gs.second);
    g.tcps.push_back(gtcp);
  }
  return g;
}

bool toMsg(tesseract_msgs::msg::KinematicsInformation& kin_info_msg,
           const tesseract_srdf::KinematicsInformation& kin_info)
{
  kin_info_msg.group_names.insert(
      kin_info_msg.group_names.end(), kin_info.group_names.begin(), kin_info.group_names.end());

  kin_info_msg.chain_groups.reserve(kin_info.chain_groups.size());
  for (const auto& group : kin_info.chain_groups)
  {
    tesseract_msgs::msg::ChainGroup g;
    g.name = group.first;
    g.chains.reserve(group.second.size());
    for (const auto& chain : group.second)
    {
      tesseract_msgs::msg::StringPair pair;
      pair.first = chain.first;
      pair.second = chain.second;
      g.chains.push_back(pair);
    }

    kin_info_msg.chain_groups.push_back(g);
  }

  kin_info_msg.joint_groups.reserve(kin_info.joint_groups.size());
  for (const auto& group : kin_info.joint_groups)
  {
    tesseract_msgs::msg::JointGroup g;
    g.name = group.first;
    g.joints.reserve(group.second.size());
    for (const auto& joint_name : group.second)
      g.joints.push_back(joint_name);

    kin_info_msg.joint_groups.push_back(g);
  }

  kin_info_msg.link_groups.reserve(kin_info.link_groups.size());
  for (const auto& group : kin_info.link_groups)
  {
    tesseract_msgs::msg::LinkGroup g;
    g.name = group.first;
    g.links.reserve(group.second.size());
    for (const auto& link_name : group.second)
      g.links.push_back(link_name);

    kin_info_msg.link_groups.push_back(g);
  }

  kin_info_msg.group_joint_states.reserve(kin_info.group_states.size());
  for (const auto& group : kin_info.group_states)
    kin_info_msg.group_joint_states.push_back(toMsg(group));

  kin_info_msg.group_tcps.reserve(kin_info.group_tcps.size());
  for (const auto& group : kin_info.group_tcps)
    kin_info_msg.group_tcps.push_back(toMsg(group));

  // Load kinematics plugins
  kin_info_msg.kinematics_plugin_info = toMsg(kin_info.kinematics_plugin_info);

  return true;
}

bool fromMsg(tesseract_srdf::KinematicsInformation& kin_info,
             const tesseract_msgs::msg::KinematicsInformation& kin_info_msg)
{
  kin_info.group_names.insert(kin_info_msg.group_names.begin(), kin_info_msg.group_names.end());

  for (const auto& group : kin_info_msg.chain_groups)
  {
    tesseract_srdf::ChainGroup chain_group;
    for (const auto& pair : group.chains)
      chain_group.emplace_back(pair.first, pair.second);

    kin_info.chain_groups[group.name] = chain_group;
  }

  for (const auto& group : kin_info_msg.joint_groups)
    kin_info.joint_groups[group.name] = group.joints;

  for (const auto& group : kin_info_msg.link_groups)
    kin_info.link_groups[group.name] = group.links;

  for (const auto& group : kin_info_msg.group_joint_states)
  {
    for (const auto& state : group.joint_states)
    {
      tesseract_srdf::GroupsJointState joint_state;
      joint_state.reserve(state.joint_state.size());
      for (const auto& js : state.joint_state)
        joint_state[js.first] = js.second;

      kin_info.group_states[group.name][state.name] = joint_state;
    }
  }

  for (const auto& group : kin_info_msg.group_tcps)
  {
    for (const auto& pose : group.tcps)
    {
      Eigen::Isometry3d tcp{ Eigen::Isometry3d::Identity() };
      fromMsg(tcp, pose.tcp);

      kin_info.group_tcps[group.name][pose.name] = tcp;
    }
  }

  // Load kinematics plugins
  kin_info.kinematics_plugin_info = fromMsg(kin_info_msg.kinematics_plugin_info);

  return true;
}

tesseract_common::KinematicsPluginInfo fromMsg(const tesseract_msgs::msg::KinematicsPluginInfo& info_msg)
{
  tesseract_common::KinematicsPluginInfo info;
  info.search_paths.insert(info_msg.search_paths.begin(), info_msg.search_paths.end());
  info.search_libraries.insert(info_msg.search_libraries.begin(), info_msg.search_libraries.end());

  for (const auto& pair : info_msg.group_fwd_plugins)
  {
    tesseract_common::PluginInfoContainer container;
    container.default_plugin = pair.plugin_container.default_plugin;
    container.plugins = fromMsg(pair.plugin_container.plugins);
    info.fwd_plugin_infos[pair.group] = container;
  }

  for (const auto& pair : info_msg.group_inv_plugins)
  {
    tesseract_common::PluginInfoContainer container;
    container.default_plugin = pair.plugin_container.default_plugin;
    container.plugins = fromMsg(pair.plugin_container.plugins);
    info.inv_plugin_infos[pair.group] = container;
  }

  return info;
}

tesseract_common::ContactManagersPluginInfo fromMsg(const tesseract_msgs::msg::ContactManagersPluginInfo& info_msg)
{
  tesseract_common::ContactManagersPluginInfo info;
  info.search_paths.insert(info_msg.search_paths.begin(), info_msg.search_paths.end());
  info.search_libraries.insert(info_msg.search_libraries.begin(), info_msg.search_libraries.end());
  info.discrete_plugin_infos.default_plugin = info_msg.discrete_plugin_container.default_plugin;
  info.discrete_plugin_infos.plugins = fromMsg(info_msg.discrete_plugin_container.plugins);
  info.continuous_plugin_infos.default_plugin = info_msg.continuous_plugin_container.default_plugin;
  info.continuous_plugin_infos.plugins = fromMsg(info_msg.continuous_plugin_container.plugins);

  return info;
}

tesseract_common::PluginInfoMap fromMsg(const std::vector<tesseract_msgs::msg::StringPluginInfoPair>& info_map_msg)
{
  tesseract_common::PluginInfoMap info_map;
  for (const auto& pair : info_map_msg)
    info_map[pair.first] = fromMsg(pair.second);

  return info_map;
}

tesseract_common::PluginInfo fromMsg(const tesseract_msgs::msg::PluginInfo& info_msg)
{
  tesseract_common::PluginInfo info;
  info.class_name = info_msg.class_name;

  if (!info_msg.config.empty())
    info.config = YAML::Load(info_msg.config);

  return info;
}

bool toMsg(tesseract_msgs::msg::TransformMap& transform_map_msg, const tesseract_common::TransformMap& transform_map)
{
  transform_map_msg.names.reserve(transform_map.size());
  transform_map_msg.transforms.reserve(transform_map.size());
  for (const auto& pair : transform_map)
  {
    transform_map_msg.names.push_back(pair.first);
    geometry_msgs::msg::Pose pose;
    if (!toMsg(pose, pair.second))
      return false;

    transform_map_msg.transforms.push_back(pose);
  }
  return true;
}

bool fromMsg(tesseract_common::TransformMap& transform_map, const tesseract_msgs::msg::TransformMap& transform_map_msg)
{
  if (transform_map_msg.names.size() != transform_map_msg.transforms.size())
    return false;

  for (std::size_t i = 0; i < transform_map_msg.names.size(); ++i)
  {
    Eigen::Isometry3d pose;
    if (fromMsg(pose, transform_map_msg.transforms.at(i)))
      return false;

    transform_map[transform_map_msg.names.at(i)] = pose;
  }

  return true;
}

bool toMsg(sensor_msgs::msg::JointState& joint_state_msg, const std::unordered_map<std::string, double>& joint_state)
{
  joint_state_msg.header.stamp = rclcpp::Clock{ RCL_ROS_TIME }.now();
  joint_state_msg.name.reserve(joint_state.size());
  joint_state_msg.position.reserve(joint_state.size());
  for (const auto& pair : joint_state)
  {
    joint_state_msg.name.push_back(pair.first);
    joint_state_msg.position.push_back(pair.second);
  }
  return true;
}

bool fromMsg(std::unordered_map<std::string, double>& joint_state, const sensor_msgs::msg::JointState& joint_state_msg)
{
  if (joint_state_msg.name.size() != joint_state_msg.position.size())
    return false;

  for (std::size_t i = 0; i < joint_state_msg.name.size(); ++i)
    joint_state[joint_state_msg.name.at(i)] = joint_state_msg.position.at(i);

  return true;
}

bool toMsg(std::vector<tesseract_msgs::msg::StringDoublePair>& joint_state_map_msg,
           const std::unordered_map<std::string, double>& joint_state)
{
  for (const auto& s : joint_state)
  {
    tesseract_msgs::msg::StringDoublePair js;
    js.first = s.first;
    js.second = s.second;
    joint_state_map_msg.push_back(js);
  }
  return true;
}

bool fromMsg(std::unordered_map<std::string, double>& joint_state,
             const std::vector<tesseract_msgs::msg::StringDoublePair>& joint_state_map_msg)
{
  for (const auto& s : joint_state_map_msg)
    joint_state[s.first] = s.second;

  return true;
}

bool toMsg(tesseract_msgs::msg::Environment& environment_msg,
           const tesseract_environment::Environment& env,
           bool include_joint_states)
{
  bool success = true;
  if (include_joint_states)
  {
    tesseract_scene_graph::SceneState env_state = env.getState();
    success = success && toMsg(environment_msg.joint_states, env_state.joints);
    success = success && toMsg(environment_msg.floating_joint_states, env_state.floating_joints);
  }

  success = success && tesseract_rosutils::toMsg(environment_msg.command_history, env.getCommandHistory(), 0);

  return success;
}

bool toMsg(tesseract_msgs::msg::Environment& environment_msg,
           const tesseract_environment::Environment::ConstPtr& env,
           bool include_joint_states)
{
  return toMsg(environment_msg, *env, include_joint_states);
}

tesseract_environment::Environment::UPtr fromMsg(const tesseract_msgs::msg::Environment& environment_msg)
{
  tesseract_environment::Commands commands;
  try
  {
    commands = tesseract_rosutils::fromMsg(environment_msg.command_history);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_ID),
                 "fromMsg(Environment): Failed to convert command history message: %s!",
                 e.what());
    return nullptr;
  }

  if (commands.empty())
    return nullptr;

  auto env = std::make_unique<tesseract_environment::Environment>();
  if (!env->init(commands))  // TODO: Get state solver
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_ID), "fromMsg(Environment): Failed to initialize environment!");
    return nullptr;
  }

  auto env_state = std::make_shared<tesseract_scene_graph::SceneState>();
  if (!tesseract_rosutils::fromMsg(env_state->joints, environment_msg.joint_states))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_ID), "fromMsg(Environment): Failed to get joint states");
    return nullptr;
  }
  if (!tesseract_rosutils::fromMsg(env_state->floating_joints, environment_msg.floating_joint_states))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_ID), "fromMsg(Environment): Failed to get floating joint states");
    return nullptr;
  }
  env->setState(env_state->joints, env_state->floating_joints);

  return env;
}

bool toMsg(tesseract_msgs::msg::TaskComposerNodeInfo& node_info_msg,
           const tesseract_planning::TaskComposerNodeInfo& node_info)
{
  using namespace tesseract_planning;
  node_info_msg.name = node_info.name;
  node_info_msg.uuid = boost::uuids::to_string(node_info.uuid);
  node_info_msg.inbound_edges.reserve(node_info.inbound_edges.size());
  for (const auto& edge : node_info.inbound_edges)
    node_info_msg.inbound_edges.push_back(boost::uuids::to_string(edge));
  node_info_msg.outbound_edges.reserve(node_info.outbound_edges.size());
  for (const auto& edge : node_info.outbound_edges)
    node_info_msg.outbound_edges.push_back(boost::uuids::to_string(edge));

  for (const auto& pair : node_info.input_keys.data())
  {
    tesseract_msgs::msg::TaskComposerKey key_msg;
    key_msg.port = pair.first;
    if (pair.second.index() == 0)
    {
      key_msg.keys.push_back(std::get<std::string>(pair.second));
    }
    else
    {
      for (const auto& key : std::get<std::vector<std::string>>(pair.second))
        key_msg.keys.push_back(key);
    }
    node_info_msg.input_keys.push_back(key_msg);
  }

  for (const auto& pair : node_info.output_keys.data())
  {
    tesseract_msgs::msg::TaskComposerKey key_msg;
    key_msg.port = pair.first;
    if (pair.second.index() == 0)
    {
      key_msg.type_index = 0;
      key_msg.keys.push_back(std::get<std::string>(pair.second));
    }
    else
    {
      key_msg.type_index = 1;
      for (const auto& key : std::get<std::vector<std::string>>(pair.second))
        key_msg.keys.push_back(key);
    }
    node_info_msg.output_keys.push_back(key_msg);
  }

  node_info_msg.return_value = node_info.return_value;
  node_info_msg.status_code = node_info_msg.status_code;
  node_info_msg.status_message = node_info_msg.status_message;
  node_info_msg.elapsed_time = node_info.elapsed_time;

  return true;
}

tesseract_planning::TaskComposerNodeInfo fromMsg(const tesseract_msgs::msg::TaskComposerNodeInfo& node_info_msg)
{
  using namespace tesseract_planning;
  tesseract_planning::TaskComposerNodeInfo node_info;
  node_info.name = node_info_msg.name;
  node_info.uuid = boost::lexical_cast<boost::uuids::uuid>(node_info_msg.uuid);
  node_info.inbound_edges.reserve(node_info_msg.inbound_edges.size());
  for (const auto& edge : node_info_msg.inbound_edges)
    node_info.inbound_edges.push_back(boost::lexical_cast<boost::uuids::uuid>(edge));
  node_info.outbound_edges.reserve(node_info_msg.outbound_edges.size());
  for (const auto& edge : node_info_msg.outbound_edges)
    node_info.outbound_edges.push_back(boost::lexical_cast<boost::uuids::uuid>(edge));

  for (const auto& key_msg : node_info_msg.input_keys)
  {
    if (key_msg.type_index == 0)
      node_info.input_keys.add(key_msg.port, key_msg.keys.front());
    else
      node_info.input_keys.add(key_msg.port, key_msg.keys);
  }

  for (const auto& key_msg : node_info_msg.output_keys)
  {
    if (key_msg.type_index == 0)
      node_info.output_keys.add(key_msg.port, key_msg.keys.front());
    else
      node_info.output_keys.add(key_msg.port, key_msg.keys);
  }

  node_info.return_value = node_info_msg.return_value;
  node_info.status_code = node_info_msg.status_code;
  node_info.status_message = node_info_msg.status_message;
  node_info.elapsed_time = node_info_msg.elapsed_time;

  return node_info;
}

trajectory_msgs::msg::JointTrajectory toMsg(const tesseract_common::JointTrajectory& joint_trajectory,
                                            const tesseract_scene_graph::SceneState& initial_state)
{
  trajectory_msgs::msg::JointTrajectory result;
  std::vector<std::string> joint_names;
  std::map<std::string, int> joint_names_indices;
  trajectory_msgs::msg::JointTrajectoryPoint last_point;
  for (const auto& joint_state : joint_trajectory)
  {
    for (const auto& joint : joint_state.joint_names)
    {
      if (std::find(joint_names.begin(), joint_names.end(), joint) == joint_names.end())
      {
        joint_names.push_back(joint);
        joint_names_indices.insert({ joint, joint_names.size() - 1 });
      }
    }
  }
  Eigen::VectorXd initial_points = initial_state.getJointValues(joint_names);
  last_point.positions =
      std::vector<double>(initial_points.data(), initial_points.data() + initial_points.rows() * initial_points.cols());
  result.joint_names = joint_names;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
  for (const auto& joint : joint_trajectory)
  {
    trajectory_msgs::msg::JointTrajectoryPoint current_point;
    current_point.positions = last_point.positions;
    current_point.velocities = std::vector<double>(joint_names.size(), 0);
    current_point.accelerations = std::vector<double>(joint_names.size(), 0);
    current_point.effort = std::vector<double>(joint_names.size(), 0);
    current_point.time_from_start = rclcpp::Duration::from_seconds(joint.time);
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(joint.joint_names.size()); j++)
    {
      auto joint_index = static_cast<std::size_t>(joint_names_indices[joint.joint_names[static_cast<std::size_t>(j)]]);
      if (joint.position.size() > 0)
        current_point.positions[joint_index] = joint.position[j];
      if (joint.velocity.size() > 0)
        current_point.velocities[joint_index] = joint.velocity[j];
      if (joint.acceleration.size() > 0)
        current_point.accelerations[joint_index] = joint.acceleration[j];
      if (joint.effort.size() > j)
        current_point.effort[joint_index] = joint.effort[j];
    }
    last_point = current_point;
    points.push_back(current_point);
  }
  result.points = points;
  return result;
}

tesseract_common::JointTrajectory fromMsg(const trajectory_msgs::msg::JointTrajectory& joint_trajectory_msg)
{
  tesseract_common::JointTrajectory joint_trajectory;
  joint_trajectory.reserve(joint_trajectory_msg.points.size());
  for (const auto& state_msg : joint_trajectory_msg.points)
  {
    tesseract_common::JointState state;
    state.joint_names = joint_trajectory_msg.joint_names;
    state.position = Eigen::Map<const Eigen::VectorXd>(state_msg.positions.data(),
                                                       static_cast<Eigen::Index>(state_msg.positions.size()));
    state.velocity = Eigen::Map<const Eigen::VectorXd>(state_msg.velocities.data(),
                                                       static_cast<Eigen::Index>(state_msg.velocities.size()));
    state.acceleration = Eigen::Map<const Eigen::VectorXd>(state_msg.accelerations.data(),
                                                           static_cast<Eigen::Index>(state_msg.accelerations.size()));
    state.effort =
        Eigen::Map<const Eigen::VectorXd>(state_msg.effort.data(), static_cast<Eigen::Index>(state_msg.effort.size()));
    state.time = state_msg.time_from_start.sec;
    joint_trajectory.push_back(state);
  }
  return joint_trajectory;
}

void toTransformMsgs(const std::shared_ptr<tesseract_environment::Environment>& env,
                     const rclcpp::Time& stamp,
                     std::vector<geometry_msgs::msg::TransformStamped>& transforms,
                     std::vector<geometry_msgs::msg::TransformStamped>& static_transforms)
{
  const auto& scene_graph = env->getSceneGraph();
  const auto& active_joints = env->getActiveJointNames();

  for (const auto& joint : scene_graph->getJoints())
  {
    const auto& tf = env->getRelativeLinkTransform(joint->parent_link_name, joint->child_link_name);
    // Convert link transform to TransformStamped message
    auto transform_msg = tf2::eigenToTransform(tf);
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = joint->parent_link_name;
    transform_msg.child_frame_id = joint->child_link_name;

    // Add to appropriate collection based on whether it's static (connected by a fixed joint) or dynamic
    if ((std::find(active_joints.begin(), active_joints.end(), joint->getName()) == active_joints.end()))
    {
      static_transforms.push_back(transform_msg);
    }
    else
    {
      transforms.push_back(transform_msg);
    }
  }
}

}  // namespace tesseract_rosutils

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_rosutils::ROSResourceLocator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_rosutils::ROSResourceLocator)
