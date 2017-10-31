/*************************************************************************/
/*  godot_x11.cpp                                                        */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2017 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2017 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/
#include <limits.h>
#include <locale.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "core/register_core_types.h"
#include "drivers/register_driver_types.h"
#include "os_dummy.h"
#include "scene/main/scene_tree.h"
#include "scene/register_scene_types.h"
#include "servers/visual/visual_server_raster.h"
#include "servers/register_server_types.h"
#include "drivers/gles3/rasterizer_gles3.h"
#include "project_settings.h"
#include "quick_hull.h"
#include "os/thread_dummy.h"

static ProjectSettings *globals = NULL;

#define OBJECT_COUNT 50

	RID test_cube;
	RID instance;
	RID camera;
	RID viewport;
	RID light;
	RID scenario;

	struct InstanceInfo {

		RID instance;
		Transform base;
		Vector3 rot_axis;
	};

	List<InstanceInfo> instances;

	float ofs;
	bool quit;

	void init() {

		print_line("INITIALIZING TEST RENDER");
		VisualServer *vs = VisualServer::get_singleton();
		test_cube = vs->get_test_cube();
		scenario = vs->scenario_create();

		Vector<Vector3> vts;

		vts.push_back(Vector3(1, 1, 1));
		vts.push_back(Vector3(1, -1, 1));
		vts.push_back(Vector3(-1, 1, 1));
		vts.push_back(Vector3(-1, -1, 1));
		vts.push_back(Vector3(1, 1, -1));
		vts.push_back(Vector3(1, -1, -1));
		vts.push_back(Vector3(-1, 1, -1));
		vts.push_back(Vector3(-1, -1, -1));

		Geometry::MeshData md;
		Error err = QuickHull::build(vts, md);
		print_line("ERR: " + itos(err));
		test_cube = vs->mesh_create();
		vs->mesh_add_surface_from_mesh_data(test_cube, md);

		List<String> cmdline = OS::get_singleton()->get_cmdline_args();
		int object_count = OBJECT_COUNT;
		if (cmdline.size() > 0 && cmdline[cmdline.size() - 1].to_int()) {
			object_count = cmdline[cmdline.size() - 1].to_int();
		};

		for (int i = 0; i < object_count; i++) {

			InstanceInfo ii;

			ii.instance = vs->instance_create2(test_cube, scenario);

			ii.base.translate(Math::random(-20, 20), Math::random(-20, 20), Math::random(-20, 18));
			ii.base.rotate(Vector3(0, 1, 0), Math::randf() * Math_PI);
			ii.base.rotate(Vector3(1, 0, 0), Math::randf() * Math_PI);
			vs->instance_set_transform(ii.instance, ii.base);

			ii.rot_axis = Vector3(Math::random(-1, 1), Math::random(-1, 1), Math::random(-1, 1)).normalized();

			instances.push_back(ii);
		}

		camera = vs->camera_create();

		// 		vs->camera_set_perspective( camera, 60.0,0.1, 100.0 );

		viewport = vs->viewport_create();
		Size2i screen_size = OS::get_singleton()->get_window_size();
		vs->viewport_set_size(viewport, screen_size.x, screen_size.y);
		vs->viewport_attach_to_screen(viewport, Rect2(Vector2(), screen_size));
		vs->viewport_set_active(viewport, true);
		vs->viewport_attach_camera(viewport, camera);
		vs->viewport_set_scenario(viewport, scenario);
		vs->camera_set_transform(camera, Transform(Basis(), Vector3(0, 3, 30)));
		vs->camera_set_perspective(camera, 60, 0.1, 1000);

		/*
		RID lightaux = vs->light_create( VisualServer::LIGHT_OMNI );
		vs->light_set_var( lightaux, VisualServer::LIGHT_VAR_RADIUS, 80 );
		vs->light_set_var( lightaux, VisualServer::LIGHT_VAR_ATTENUATION, 1 );
		vs->light_set_var( lightaux, VisualServer::LIGHT_VAR_ENERGY, 1.5 );
		light = vs->instance_create( lightaux );
		*/
		RID lightaux;

		lightaux = vs->light_create(VisualServer::LIGHT_DIRECTIONAL);
		//vs->light_set_color( lightaux, VisualServer::LIGHT_COLOR_AMBIENT, Color(0.0,0.0,0.0) );
		vs->light_set_color(lightaux, Color(1.0, 1.0, 1.0));
		//vs->light_set_shadow( lightaux, true );
		light = vs->instance_create2(lightaux, scenario);
		Transform lla;
		//lla.set_look_at(Vector3(),Vector3(1,-1,1),Vector3(0,1,0));
		lla.set_look_at(Vector3(), Vector3(-0.000000, -0.836026, -0.548690), Vector3(0, 1, 0));

		vs->instance_set_transform(light, lla);

		lightaux = vs->light_create(VisualServer::LIGHT_OMNI);
		//vs->light_set_color( lightaux, VisualServer::LIGHT_COLOR_AMBIENT, Color(0.0,0.0,1.0) );
		vs->light_set_color(lightaux, Color(1.0, 1.0, 0.0));
		vs->light_set_param(lightaux, VisualServer::LIGHT_PARAM_RANGE, 4);
		vs->light_set_param(lightaux, VisualServer::LIGHT_PARAM_ENERGY, 8);
		//vs->light_set_shadow( lightaux, true );
		//light = vs->instance_create( lightaux );

		ofs = 0;
		quit = false;
	}

int main(int argc, char *argv[]) {

  OS_Dummy os;
// Main::setup()
	RID_OwnerBase::init_rid();

  /// THESE are needed for ObjectDB operations
	ThreadDummy::make_default();
	SemaphoreDummy::make_default();
	MutexDummy::make_default();
  RWLockDummy::make_default();
  IPDummy::make_default();
  // ============================

	ClassDB::init();

	register_core_types();
	register_core_driver_types();

  // Needed for GLOBAL_DEF in VisualServer and Rasterizer
  globals = memnew(ProjectSettings);
  register_core_settings(); //here globals is present

  OS::VideoMode current_videomode = os.get_default_video_mode();

//////////////////////////////////////
// Main setup2
  if( !glfwInit() )
  {
      fprintf( stderr, "Failed to initialize GLFW\n" );
      return -1;
  }
  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

  // Open a window and create its OpenGL context
  GLFWwindow* window; // (In the accompanying source code, this variable is global for simplicity)
  window = glfwCreateWindow( current_videomode.width, current_videomode.height, "Tutorial 01", NULL, NULL);
  if( window == NULL ){
      fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
      glfwTerminate();
      return -1;
  }
  glfwMakeContextCurrent(window); // Initialize GLEW
//glewExperimental=true; // Needed in core profile
//if (glewInit() != GLEW_OK) {
    //fprintf(stderr, "Failed to initialize GLEW\n");
    //return -1;
//}

  glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
////////////////////////////////////////////////////////////////

  RasterizerGLES3::register_config();
	RasterizerGLES3::make_current();

	VisualServer* visual_server = memnew(VisualServerRaster);
  visual_server->init();

  //=====================================
  register_server_types();
	Color clear = GLOBAL_DEF("rendering/environment/default_clear_color", Color(0.3, 0.3, 0.3));
	VisualServer::get_singleton()->set_default_clear_color(clear);

  register_scene_types();

  ARVRServer* arvr_server = memnew(ARVRServer); // Needed for VisualServer::draw(), in VisualServerViewport::draw_viewports()

  init(); // This create the cubes and add them to the visual server
  std::cout << "before main loop..." << std::endl;

  do {
    //glClear(GL_COLOR_BUFFER_BIT);
	  std::cout << "_____________________________\n" << std::flush;
    //VisualServer::get_singleton()->draw(); // flush visual commands

    VSG::scene->update_dirty_instances(); //update scene stuff
    VSG::viewport->draw_viewports();
    VSG::scene->render_probes();

	  std::cout << "*****************************\n" << std::flush;
	  glfwSwapBuffers(window);
	  glfwPollEvents();
  } while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
      glfwWindowShouldClose(window) == 0);

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

  glfwTerminate();

  return 0;//os.get_exit_code();
}
