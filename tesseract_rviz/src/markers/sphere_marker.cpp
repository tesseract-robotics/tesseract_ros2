/**
 * @file sphere_marker.cpp
 * @brief Sphere marker
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#include <tesseract_rviz/markers/sphere_marker.h>
#include <tesseract_rviz/markers/marker_selection_handler.h>
#include <tesseract_rviz/markers/utils.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/selection_manager.hpp>

#include <rviz_rendering/objects/shape.hpp>

#include <OgreSceneNode.h>
#include <OgreMatrix3.h>

namespace tesseract_rviz
{
SphereMarker::SphereMarker(const std::string& ns,
                           const int id,
                           rviz_common::DisplayContext* context,
                           Ogre::SceneNode* parent_node,
                           float radius)
  : MarkerBase(ns, id, context, parent_node), shape_(nullptr), scale_(Ogre::Vector3(1, 1, 1)), radius_(radius)
{
  shape_ = new rviz_rendering::Shape(rviz_rendering::Shape::Sphere, context_->getSceneManager(), scene_node_);
  setScale(scale_);

  handler_.reset(new MarkerSelectionHandler(this, MarkerID(ns_, id_), context_));
  handler_->addTrackedObjects(shape_->getRootNode());
}

SphereMarker::~SphereMarker() { delete shape_; }

void SphereMarker::setScale(Ogre::Vector3 scale)
{
  scale_ = scale;
  shape_->setScale(radius_ * scale_);
}

Ogre::Vector3 SphereMarker::getScale() const { return scale_; }

void SphereMarker::setRadius(float radius)
{
  radius_ = radius;
  shape_->setScale(radius_ * scale_);
}

float SphereMarker::getRadius() const { return radius_; }

void SphereMarker::setColor(Ogre::ColourValue color) { shape_->setColor(color.r, color.g, color.b, color.a); }

std::set<Ogre::MaterialPtr> SphereMarker::getMaterials()
{
  std::set<Ogre::MaterialPtr> materials;
  extractMaterials(shape_->getEntity(), materials);
  return materials;
}

}  // namespace tesseract_rviz
