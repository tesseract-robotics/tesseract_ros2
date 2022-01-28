/**
 * @file environment_cache.cpp
 * @copyright Copyright (c) 2022, Southwest Research Institute
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

#include <tesseract_planning_server/environment_cache.h>

namespace tesseract_planning_server
{

ROSProcessEnvironmentCache::ROSProcessEnvironmentCache(tesseract_monitoring::EnvironmentMonitor::ConstPtr env)
  : environment_(std::move(env))
{
}

void ROSProcessEnvironmentCache::setCacheSize(long size)
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  cache_size_ = static_cast<std::size_t>(size);
}

long ROSProcessEnvironmentCache::getCacheSize() const { return static_cast<long>(cache_size_); }

void ROSProcessEnvironmentCache::refreshCache() const
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract_environment::Environment::UPtr env;
  {
    auto lock = environment_->lockEnvironmentRead();
    int rev = environment_->getEnvironment()->getRevision();
    if (rev != cache_env_revision_ || cache_.empty())
    {
      env = environment_->getEnvironment()->clone();
      cache_env_revision_ = rev;
    }
  }

  if (env != nullptr)
  {
    cache_.clear();
    for (std::size_t i = 0; i < cache_size_; ++i)
      cache_.emplace_back(env->clone());
  }
  else if (cache_.size() <= 2)
  {
    for (std::size_t i = (cache_.size() - 1); i < cache_size_; ++i)
      cache_.emplace_back(cache_.front()->clone());
  }
}

tesseract_environment::Environment::UPtr ROSProcessEnvironmentCache::getCachedEnvironment() const
{
  // This is to make sure the cached items are updated if needed
  refreshCache();

  tesseract_scene_graph::SceneState current_state;
  {
    auto lock = environment_->lockEnvironmentRead();
    current_state = environment_->getEnvironment()->getState();
  }

  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract_environment::Environment::UPtr env = std::move(cache_.back());

  // Update to the current joint values
  env->setState(current_state.joints);

  cache_.pop_back();

  return env;
}

}
