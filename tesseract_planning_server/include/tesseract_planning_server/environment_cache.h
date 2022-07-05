/**
 * @file environment_cache.h
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

#include <tesseract_process_managers/core/process_environment_cache.h>
#include <tesseract_monitoring/environment_monitor.h>

namespace tesseract_planning_server
{
class ROSProcessEnvironmentCache : public tesseract_planning::EnvironmentCache
{
public:
  using Ptr = std::shared_ptr<ROSProcessEnvironmentCache>;
  using ConstPtr = std::shared_ptr<const ROSProcessEnvironmentCache>;

  ROSProcessEnvironmentCache(tesseract_monitoring::EnvironmentMonitor::ConstPtr env);

  /**
   * @brief Set the cache size used to hold tesseract objects for motion planning
   * @param size The size of the cache.
   */
  void setCacheSize(long size) override final;

  /**
   * @brief Get the cache size used to hold tesseract objects for motion planning
   * @return The size of the cache.
   */
  long getCacheSize() const override final;

  /** @brief If the environment has changed it will rebuild the cache of tesseract objects */
  void refreshCache() const override final;

  /**
   * @brief This will pop a Tesseract object from the queue
   * @details This will first call refreshCache to ensure it has an updated tesseract then proceed
   */
  tesseract_environment::Environment::UPtr getCachedEnvironment() const override final;

protected:
  /** @brief The tesseract_object used to create the cache */
  tesseract_monitoring::EnvironmentMonitor::ConstPtr environment_;

  /** @brief The assigned cache size */
  std::size_t cache_size_{ 5 };

  /** @brief The environment revision number at the time the cache was populated */
  mutable int cache_env_revision_{ 0 };

  /** @brief A vector of cached Tesseract objects */
  mutable std::deque<tesseract_environment::Environment::UPtr> cache_;

  /** @brief The mutex used when reading and writing to cache_ */
  mutable std::shared_mutex cache_mutex_;
};

}  // namespace tesseract_planning_server
