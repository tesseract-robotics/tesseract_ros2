#ifndef TESSERACT_RVIZ_TYPES_H
#define TESSERACT_RVIZ_TYPES_H

#include <rviz_rendering/objects/point_cloud.hpp>
#include <tesseract_geometry/impl/octree.h>

namespace tesseract_rviz
{
enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2
};

enum OctreeVoxelColorMode
{
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
};

struct OctreeDataContainer
{
  std::shared_ptr<rviz_rendering::PointCloud> point_cloud;
  std::vector<rviz_rendering::PointCloud::Point> points;
  float size;
  tesseract_geometry::Octree::SubType shape_type;
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_TYPES_H
