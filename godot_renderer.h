#pragma once

// Define this macro so glad.h will be included in shader_gles3.h
#define GLES3_INCLUDE_H "glad/glad.h"

#include "core/error_list.h"
#include "core/register_core_types.h"
#include "drivers/gles3/rasterizer_gles3.h"
#include "drivers/register_driver_types.h"
#include "drivers/unix/dir_access_unix.h"
#include "drivers/unix/file_access_unix.h"
#include "os/thread_dummy.h"
#include "os_dummy.h"
#include "project_settings.h"
#include "quick_hull.h"
#include "scene/main/scene_tree.h"
#include "scene/register_scene_types.h"
#include "servers/register_server_types.h"
#include "servers/visual/visual_server_raster.h"

// TL;DR: GLFW has to be included here, after rasterizer_gles3.
// glfw needs to be included *after* glad.h, which is
// included by shader_gles3.h in rasterizer.gles3.h
#include <GLFW/glfw3.h>

class GodotRenderer {
  int window_width, window_height;
  //TODO: Use nullptr when C++11 is enabled
  OS_Dummy os_;
  GLFWwindow *window_ = NULL;
  VisualServer *visual_server = NULL;
  ARVRServer *arvr_server = NULL;
  ProjectSettings *globals = NULL;

public:
  GodotRenderer(int window_width, int window_height)
    : window_width(window_width), window_height(window_height), os_(window_width, window_height) {}

  void Initialize() {
    InitGLContext();
    InitGodot();
  }

  void Draw() {
    VSG::scene->update_dirty_instances(); //update scene stuff
    VSG::viewport->draw_viewports();
    VSG::scene->render_probes();
  }

  void MainLoop() {
    do {
      glClear(GL_COLOR_BUFFER_BIT);
      Draw();
      // TODO: modify this for render_to_texture
      glfwSwapBuffers(window_);
      glfwPollEvents();
    } while (glfwGetKey(window_, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
         glfwWindowShouldClose(window_) == 0);
  }

  void Cleanup() {
    Cleanup_godot();
    CleanupGL();
  }

  GLFWwindow *const get_glfw_window() const { return window_; }

private:
  // TODO: modify this for render_to_texture
  Error InitGLContext() {
    if (!glfwInit()) {
      fprintf(stderr, "Failed to initialize GLFW\n");
      return FAILED;
    }
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

    // Open a window_ and create its OpenGL context
    window_ = glfwCreateWindow(window_width,
        window_height, "Godot Render window_", NULL, NULL);
    if (window_ == NULL) {
      fprintf(stderr, "Failed to open GLFW window_.\n");
      glfwTerminate();
      return FAILED;
    }
    glfwMakeContextCurrent(window_); // Initialize GLEW
    glfwSetInputMode(window_, GLFW_STICKY_KEYS, GL_TRUE);
  }

  void InitGodot() {
    // Main::setup()
    RID_OwnerBase::init_rid();

    /// THESE are needed for ObjectDB operations
    ThreadDummy::make_default();
    SemaphoreDummy::make_default();
    MutexDummy::make_default();
    RWLockDummy::make_default();
    IPDummy::make_default();

    // Do we need this? Can we just pass resources directly from Drake?
    FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_RESOURCES);
    FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_USERDATA);
    FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_FILESYSTEM);
    //FileAccessBufferedFA<FileAccessUnix>::make_default();
    //TODO: Change to DirAccessOSX for osx platfom
    DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_RESOURCES);
    DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_USERDATA);
    DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_FILESYSTEM);
    // ============================

    ClassDB::init();

    register_core_types();
    register_core_driver_types();

    // Needed for GLOBAL_DEF in VisualServer and Rasterizer
    globals = memnew(ProjectSettings);
    register_core_settings(); //here globals is present

    //////////////////////////////////////
    // Main setup2
    ////////////////////////////////////////////////////////////////

    RasterizerGLES3::register_config();
    RasterizerGLES3::make_current();

    visual_server = memnew(VisualServerRaster);
    visual_server->init();

    //=====================================
    register_server_types();
    Color clear = GLOBAL_DEF("rendering/environment/default_clear_color", Color(0.3, 0.3, 0.3));
    VisualServer::get_singleton()->set_default_clear_color(clear);

    register_scene_types();

    arvr_server = memnew(ARVRServer); // Needed for VisualServer::draw(), in VisualServerViewport::draw_viewports()
  }

  void Cleanup_godot() {
    // Cleanup
    unregister_scene_types();
    unregister_server_types();

    //memdelete(main_loop);
    visual_server->finish();
    memdelete(visual_server);

    if (arvr_server)
      memdelete(arvr_server);

    if (globals)
      memdelete(globals);

    unregister_core_driver_types();
    unregister_core_types();
  }

  void CleanupGL() {
    glfwTerminate();
  }
};
