/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TESSERACT_RVIZ_MARKERS_MARKER_SELECTION_HANDLER_H
#define TESSERACT_RVIZ_MARKERS_MARKER_SELECTION_HANDLER_H
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"

namespace tesseract_rviz
{
class MarkerBase;
typedef std::pair<std::string, int32_t> MarkerID;

class MarkerSelectionHandler : public rviz_common::interaction::SelectionHandler
{
public:
  using Ptr = std::shared_ptr<MarkerSelectionHandler>;
  using ConstPtr = std::shared_ptr<const MarkerSelectionHandler>;

  MarkerSelectionHandler(const MarkerBase* marker, MarkerID id, rviz_common::DisplayContext* context);
  virtual ~MarkerSelectionHandler();

  virtual void setPosition(const Ogre::Vector3& position);
  virtual void setOrientation(const Ogre::Quaternion& orientation);
  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

private:
  const MarkerBase* marker_;
  QString marker_id_;
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_INTERACTIVE_MARKER_MARKER_SELECTION_HANDLER_H
