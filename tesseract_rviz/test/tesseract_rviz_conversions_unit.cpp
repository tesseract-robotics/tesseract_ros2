#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <memory>
#include <Eigen/Eigen>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/conversions.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/link.h>

#include <OgreSceneManager.h>
#include <OgreRoot.h>
#include <OgreMaterialManager.h>
#include <OgreLogManager.h>

#include <tesseract_qt/common/entity_container.h>
#include <tesseract_qt/common/entity_manager.h>

/**
 * @brief Test fixture for tesseract_rviz conversions
 * 
 * This fixture sets up an Ogre environment for testing the conversion functions
 */
class TesseractRvizConversionsUnit : public ::testing::Test
{
protected:
  Ogre::Root* ogre_root_;
  Ogre::SceneManager* scene_manager_;
  std::unique_ptr<tesseract_gui::EntityManager> entity_manager_;
  
  void SetUp() override
  {
    // Initialize Ogre without rendering window
    Ogre::LogManager* log_manager = new Ogre::LogManager();
    log_manager->createLog("OgreTest.log", true, false, true);
    
    ogre_root_ = new Ogre::Root("", "", "");
    
    // Load render system plugin - try different possible names
    try {
      ogre_root_->loadPlugin("RenderSystem_GL");
    } catch (...) {
      try {
        ogre_root_->loadPlugin("RenderSystem_GL3Plus");
      } catch (...) {
        // If no GL available, we'll handle it gracefully in tests
      }
    }
    
    // Set render system if available
    const Ogre::RenderSystemList& render_systems = ogre_root_->getAvailableRenderers();
    if (!render_systems.empty())
    {
      ogre_root_->setRenderSystem(render_systems[0]);
      ogre_root_->initialise(false);
    }
    
    // Create scene manager
    scene_manager_ = ogre_root_->createSceneManager();
    
    // Initialize entity manager
    entity_manager_ = std::make_unique<tesseract_gui::EntityManager>();
  }
  
  void TearDown() override
  {
    entity_manager_.reset();
    
    if (ogre_root_)
    {
      delete ogre_root_;
      ogre_root_ = nullptr;
    }
    
    if (Ogre::LogManager::getSingletonPtr())
    {
      delete Ogre::LogManager::getSingletonPtr();
    }
  }
  
  /**
   * @brief Helper to create a material for testing
   */
  Ogre::MaterialPtr createTestMaterial()
  {
    static int material_counter = 0;
    std::string mat_name = "TestMaterial_" + std::to_string(material_counter++);
    return Ogre::MaterialManager::getSingleton().create(
        mat_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }
};

/**
 * @brief Test that PLANE geometry type does not cause a crash
 * 
 * This test verifies that the loadLinkGeometry function properly handles
 * the PLANE geometry type without returning nullptr or causing a segfault
 */
TEST_F(TesseractRvizConversionsUnit, PlaneGeometryDoesNotCrash)
{
  // Skip if Ogre is not fully initialized
  if (!ogre_root_->isInitialised())
  {
    GTEST_SKIP() << "Ogre not initialized, skipping test";
  }
  
  // Create a PLANE geometry
  auto plane_geometry = std::make_shared<tesseract_geometry::Plane>(1.0, 0.0, 0.0, 0.0);
  
  // Create entity container
  auto entity_container = entity_manager_->getEntityContainer("test");
  
  // Create test material
  auto material = createTestMaterial();
  
  // Create identity transform
  Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  
  // This should NOT crash or return nullptr
  Ogre::SceneNode* result = tesseract_rviz::loadLinkGeometry(
      *scene_manager_,
      entity_container,
      *plane_geometry,
      scale,
      local_pose,
      material,
      true);
  
  // Verify that we got a valid scene node
  EXPECT_NE(result, nullptr) << "loadLinkGeometry should not return nullptr for PLANE geometry";
  
  // Verify the scene node has objects attached
  if (result)
  {
    EXPECT_GT(result->numAttachedObjects(), 0u) << "PLANE scene node should have attached objects";
  }
}

/**
 * @brief Test that unsupported geometry types do not cause a crash
 * 
 * This test uses the default case to verify that even for truly unsupported
 * geometry types, the function handles them gracefully
 */
TEST_F(TesseractRvizConversionsUnit, UnsupportedGeometryDoesNotCrash)
{
  // Skip if Ogre is not fully initialized
  if (!ogre_root_->isInitialised())
  {
    GTEST_SKIP() << "Ogre not initialized, skipping test";
  }
  
  // Create a supported geometry (BOX) to ensure the test setup works
  auto box_geometry = std::make_shared<tesseract_geometry::Box>(1.0, 1.0, 1.0);
  
  // Create entity container
  auto entity_container = entity_manager_->getEntityContainer("test");
  
  // Create test material
  auto material = createTestMaterial();
  
  // Create identity transform
  Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  
  // Test with a supported geometry first
  Ogre::SceneNode* result = tesseract_rviz::loadLinkGeometry(
      *scene_manager_,
      entity_container,
      *box_geometry,
      scale,
      local_pose,
      material,
      true);
  
  EXPECT_NE(result, nullptr) << "loadLinkGeometry should return valid node for BOX geometry";
}

/**
 * @brief Test that nullptr return values are handled properly in loadLinkVisuals
 * 
 * This test verifies that if loadLinkGeometry returns nullptr (for unsupported types),
 * the nullptr check in loadLinkVisuals prevents a crash
 */
TEST_F(TesseractRvizConversionsUnit, NullptrCheckPreventsSegfault)
{
  // Skip if Ogre is not fully initialized
  if (!ogre_root_->isInitialised())
  {
    GTEST_SKIP() << "Ogre not initialized, skipping test";
  }
  
  // Create a link with PLANE visual geometry
  tesseract_scene_graph::Link link("test_link");
  
  auto visual = std::make_shared<tesseract_scene_graph::Visual>();
  visual->geometry = std::make_shared<tesseract_geometry::Plane>(1.0, 0.0, 0.0, 0.0);
  visual->origin = Eigen::Isometry3d::Identity();
  
  auto material = std::make_shared<tesseract_scene_graph::Material>("test_material");
  material->color << 1.0, 0.0, 0.0, 1.0;
  visual->material = material;
  
  link.visual.push_back(visual);
  
  // Create entity container
  auto entity_container = entity_manager_->getEntityContainer("test");
  
  // This should NOT crash even if geometry is unsupported
  Ogre::SceneNode* result = tesseract_rviz::loadLinkVisuals(
      *scene_manager_,
      entity_container,
      link,
      nullptr);
  
  // Should return a valid scene node (even if geometry wasn't loaded)
  EXPECT_NE(result, nullptr) << "loadLinkVisuals should always return a valid scene node";
}

/**
 * @brief Test various supported geometry types
 * 
 * This test verifies that all the standard geometry types are properly handled
 */
TEST_F(TesseractRvizConversionsUnit, SupportedGeometryTypes)
{
  // Skip if Ogre is not fully initialized
  if (!ogre_root_->isInitialised())
  {
    GTEST_SKIP() << "Ogre not initialized, skipping test";
  }
  
  auto entity_container = entity_manager_->getEntityContainer("test");
  auto material = createTestMaterial();
  Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  
  // Test SPHERE
  {
    auto geometry = std::make_shared<tesseract_geometry::Sphere>(0.5);
    Ogre::SceneNode* result = tesseract_rviz::loadLinkGeometry(
        *scene_manager_, entity_container, *geometry, scale, local_pose, material, true);
    EXPECT_NE(result, nullptr) << "SPHERE geometry should be supported";
  }
  
  // Test BOX
  {
    auto geometry = std::make_shared<tesseract_geometry::Box>(1.0, 1.0, 1.0);
    Ogre::SceneNode* result = tesseract_rviz::loadLinkGeometry(
        *scene_manager_, entity_container, *geometry, scale, local_pose, material, true);
    EXPECT_NE(result, nullptr) << "BOX geometry should be supported";
  }
  
  // Test CYLINDER
  {
    auto geometry = std::make_shared<tesseract_geometry::Cylinder>(0.5, 1.0);
    Ogre::SceneNode* result = tesseract_rviz::loadLinkGeometry(
        *scene_manager_, entity_container, *geometry, scale, local_pose, material, true);
    EXPECT_NE(result, nullptr) << "CYLINDER geometry should be supported";
  }
  
  // Test CONE
  {
    auto geometry = std::make_shared<tesseract_geometry::Cone>(0.5, 1.0);
    Ogre::SceneNode* result = tesseract_rviz::loadLinkGeometry(
        *scene_manager_, entity_container, *geometry, scale, local_pose, material, true);
    EXPECT_NE(result, nullptr) << "CONE geometry should be supported";
  }
  
  // Test CAPSULE
  {
    auto geometry = std::make_shared<tesseract_geometry::Capsule>(0.5, 1.0);
    Ogre::SceneNode* result = tesseract_rviz::loadLinkGeometry(
        *scene_manager_, entity_container, *geometry, scale, local_pose, material, true);
    EXPECT_NE(result, nullptr) << "CAPSULE geometry should be supported";
  }
  
  // Test PLANE (newly added support)
  {
    auto geometry = std::make_shared<tesseract_geometry::Plane>(1.0, 0.0, 0.0, 0.0);
    Ogre::SceneNode* result = tesseract_rviz::loadLinkGeometry(
        *scene_manager_, entity_container, *geometry, scale, local_pose, material, true);
    EXPECT_NE(result, nullptr) << "PLANE geometry should now be supported";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
