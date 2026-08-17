#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <string>
#include <Eigen/Eigen>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMaterialManager.h>
#include <OgreLogManager.h>
#include <OgreRenderWindow.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/conversions.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/scene_graph/link.h>

#include <tesseract_qt/common/entity_container.h>
#include <tesseract_qt/common/entity_manager.h>

/**
 * @brief Fixture providing a headless Ogre root backed by a hidden render window
 *
 * Tests using this fixture skip when no GL render system can be brought up, which is the
 * normal case in CI. Anything that must run in CI belongs in a fixture-free test.
 */
class TesseractRvizConversionsUnit : public ::testing::Test
{
protected:
  std::unique_ptr<Ogre::LogManager> log_manager_;
  std::unique_ptr<Ogre::Root> ogre_root_;
  Ogre::RenderWindow* render_window_{ nullptr };
  Ogre::SceneManager* scene_manager_{ nullptr };
  // EntityManager derives from enable_shared_from_this, so getEntityContainer() throws unless
  // the manager is owned by a shared_ptr
  std::shared_ptr<tesseract::gui::EntityManager> entity_manager_;

  /** @brief Ogre hands a bare plugin name to dlopen, whose search path never covers the vendored
   *         plugin directory, so the render system must be loaded by resolved full path */
  bool loadRenderSystemPlugin()
  {
#ifdef TESSERACT_RVIZ_OGRE_PLUGIN_DIR
    for (const char* plugin : { "RenderSystem_GL", "RenderSystem_GL3Plus" })
    {
      const std::filesystem::path path =
          std::filesystem::path(TESSERACT_RVIZ_OGRE_PLUGIN_DIR) / (std::string(plugin) + ".so");
      if (!std::filesystem::exists(path))
        continue;

      try
      {
        ogre_root_->loadPlugin(path.string());
        return true;
      }
      catch (const Ogre::Exception& e)
      {
        Ogre::LogManager::getSingleton().logMessage("Could not load " + path.string() + ": " + e.getDescription());
      }
    }
#endif
    return false;
  }

  void SetUp() override
  {
    // The Ogre singletons are process wide, so a second fixture must not create a second LogManager
    if (Ogre::LogManager::getSingletonPtr() == nullptr)
    {
      log_manager_ = std::make_unique<Ogre::LogManager>();
      log_manager_->createLog("OgreTest.log", true, false, true);
    }

    ogre_root_ = std::make_unique<Ogre::Root>("", "", "");
    entity_manager_ = std::make_shared<tesseract::gui::EntityManager>();

    if (!loadRenderSystemPlugin())
      return;

    const Ogre::RenderSystemList& render_systems = ogre_root_->getAvailableRenderers();
    if (render_systems.empty())
      return;

    ogre_root_->setRenderSystem(render_systems[0]);
    ogre_root_->initialise(false);

    // Mesh generation needs a hardware buffer manager, which the GL render system only creates
    // along with its first render context
    try
    {
      Ogre::NameValuePairList params;
      params["hidden"] = "true";
      render_window_ = ogre_root_->createRenderWindow("TesseractRvizConversionsUnit", 32, 32, false, &params);
    }
    catch (const Ogre::Exception& e)
    {
      Ogre::LogManager::getSingleton().logMessage("Could not create a render window: " + e.getDescription());
      return;
    }

    scene_manager_ = ogre_root_->createSceneManager();
  }

  void TearDown() override
  {
    entity_manager_.reset();
    scene_manager_ = nullptr;
    render_window_ = nullptr;
    ogre_root_.reset();
    log_manager_.reset();
  }

  bool ogreReady() const { return render_window_ != nullptr && scene_manager_ != nullptr; }

  /** @brief Name the material after the running test so no process-wide counter is needed */
  static Ogre::MaterialPtr createTestMaterial()
  {
    const std::string name =
        std::string("TestMaterial_") + ::testing::UnitTest::GetInstance()->current_test_info()->name();
    return Ogre::MaterialManager::getSingleton().create(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }
};

/** @brief An unsupported geometry must be skipped, while the supported ones still render */
TEST_F(TesseractRvizConversionsUnit, UnsupportedGeometryIsSkipped)  // NOLINT
{
  if (!ogreReady())
    GTEST_SKIP() << "No Ogre render system available";

  tesseract::scene_graph::Link link("test_link");

  // POLYGON_MESH is the only geometry type loadLinkGeometry's switch does not handle
  auto unsupported = std::make_shared<tesseract::scene_graph::Visual>();
  unsupported->geometry = std::make_shared<tesseract::geometry::PolygonMesh>(
      std::make_shared<tesseract::common::VectorVector3d>(), std::make_shared<Eigen::VectorXi>());
  unsupported->origin = Eigen::Isometry3d::Identity();
  link.visual.push_back(unsupported);

  // Pairing it with a supported visual is what makes the test fail without the null guard: the
  // unsupported one must contribute no child, the supported one exactly one. A raw triangle MESH
  // is used rather than a BOX because it needs no installed ogre_media asset.
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(0.0, 0.0, 0.0);
  vertices->emplace_back(1.0, 0.0, 0.0);
  vertices->emplace_back(0.0, 1.0, 0.0);
  auto faces = std::make_shared<Eigen::VectorXi>(4);
  *faces << 3, 0, 1, 2;

  auto supported = std::make_shared<tesseract::scene_graph::Visual>();
  supported->geometry = std::make_shared<tesseract::geometry::Mesh>(vertices, faces);
  supported->origin = Eigen::Isometry3d::Identity();
  link.visual.push_back(supported);

  auto entity_container = entity_manager_->getEntityContainer("test");

  Ogre::SceneNode* result = tesseract_rviz::loadLinkVisuals(*scene_manager_, *entity_container, link, nullptr);
  ASSERT_NE(result, nullptr) << "loadLinkVisuals must always return a valid scene node";
  EXPECT_EQ(result->numChildren(), 1U) << "only the supported visual may contribute a child node";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
