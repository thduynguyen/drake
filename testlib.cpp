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
#include "os_x11.h"
#include "X11/Xutil.h"
#include "scene/main/scene_tree.h"
#include "servers/visual/visual_server_raster.h"
#include "scene/register_scene_types.h"
#include "servers/register_server_types.h"
#include "drivers/gles3/rasterizer_gles3.h"
#include "project_settings.h"
#include "quick_hull.h"
#include "os/thread_dummy.h"

static ProjectSettings *globals = NULL;
extern Engine *engine = NULL;

#define OBJECT_COUNT 50
class TestMainLoop : public MainLoop {

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

protected:
public:
	virtual void input_event(const Ref<InputEvent> &p_event) {

		if (p_event->is_pressed())
			quit = true;
	}

	virtual void init() {

		print_line("INITIALIZING TEST RENDER");
		VisualServer *vs = VisualServer::get_singleton();
		test_cube = vs->get_test_cube();
		scenario = vs->scenario_create();

		Vector<Vector3> vts;

		/*
		PoolVector<Plane> sp = Geometry::build_sphere_planes(2,5,5);
		Geometry::MeshData md2 = Geometry::build_convex_mesh(sp);
		vts=md2.vertices;
*/
		/*

		static const int s = 20;
		for(int i=0;i<s;i++) {
			Basis rot(Vector3(0,1,0),i*Math_PI/s);

			for(int j=0;j<s;j++) {
				Vector3 v;
				v.x=Math::sin(j*Math_PI*2/s);
				v.y=Math::cos(j*Math_PI*2/s);

				vts.push_back( rot.xform(v*2 ) );
			}
		}*/
		/*for(int i=0;i<100;i++) {

			vts.push_back( Vector3(Math::randf()*2-1.0,Math::randf()*2-1.0,Math::randf()*2-1.0).normalized()*2);
		}*/
		/*
		vts.push_back(Vector3(0,0,1));
		vts.push_back(Vector3(0,0,-1));
		vts.push_back(Vector3(0,1,0));
		vts.push_back(Vector3(0,-1,0));
		vts.push_back(Vector3(1,0,0));
		vts.push_back(Vector3(-1,0,0));*/

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
		//vs->scenario_set_debug(scenario,VS::SCENARIO_DEBUG_WIREFRAME);

		/*
		RID sm = vs->shader_create();
		//vs->shader_set_fragment_code(sm,"OUT_ALPHA=mod(TIME,1);");
		//vs->shader_set_vertex_code(sm,"OUT_VERTEX=IN_VERTEX*mod(TIME,1);");
		vs->shader_set_fragment_code(sm,"OUT_DIFFUSE=vec3(1,0,1);OUT_GLOW=abs(sin(TIME));");
		RID tcmat = vs->mesh_surface_get_material(test_cube,0);
		vs->material_set_shader(tcmat,sm);
		*/

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
	virtual bool iteration(float p_time) {

		VisualServer *vs = VisualServer::get_singleton();
		//Transform t;
		//t.rotate(Vector3(0, 1, 0), ofs);
		//t.translate(Vector3(0,0,20 ));
		//vs->camera_set_transform(camera, t);

		ofs += p_time * 0.05;

		//return quit;

		for (List<InstanceInfo>::Element *E = instances.front(); E; E = E->next()) {

			Transform pre(Basis(E->get().rot_axis, ofs), Vector3());
			vs->instance_set_transform(E->get().instance, pre * E->get().base);
			/*
			if( !E->next() ) {

				vs->free( E->get().instance );
				instances.erase(E );
			}*/
		}

		return quit;
	}

	virtual bool idle(float p_time) {
		return quit;
	}

	virtual void finish() {
	}
};

int main(int argc, char *argv[]) {

  OS_X11 os;
// Main::setup()
	RID_OwnerBase::init_rid();

  /// THESE are needed for ObjectDB operations
	ThreadDummy::make_default();
	SemaphoreDummy::make_default();
	MutexDummy::make_default();
  RWLockDummy::make_default();
  IPDummy::make_default();

	//os.initialize_core();

  //engine = memnew(Engine); // needed in RasterizerGLES3::begin_frame() call to Engine::get_time_scale

	ClassDB::init();

	register_core_types();
	register_core_driver_types();

	//Thread::_main_thread_id = Thread::get_caller_id();

	globals = memnew(ProjectSettings);
	//input_map = memnew(InputMap);

	register_core_settings(); //here globals is present

  //os.initialize_logger();

	//translation_server = memnew(TranslationServer);
	//performance = memnew(Performance);
	//ClassDB::register_class<Performance>();
	//globals->add_singleton(ProjectSettings::Singleton("Performance", performance));

  OS::VideoMode current_videomode = os.get_default_video_mode();
  //OS::VideoMode current_videomode;

	GLOBAL_DEF("memory/limits/multithreaded_server/rid_pool_prealloc", 60);
	GLOBAL_DEF("network/limits/debugger_stdout/max_chars_per_second", 2048);
	GLOBAL_DEF("display/window/size/width", current_videomode.width);
	GLOBAL_DEF("display/window/size/height", current_videomode.height);
	GLOBAL_DEF("display/window/dpi/allow_hidpi", false);
	GLOBAL_DEF("display/window/size/fullscreen", current_videomode.fullscreen);
	GLOBAL_DEF("display/window/size/resizable", current_videomode.resizable);
	GLOBAL_DEF("display/window/size/borderless", current_videomode.borderless_window);
  //use_vsync = GLOBAL_DEF("display/window/vsync/use_vsync", use_vsync);
	GLOBAL_DEF("display/window/size/test_width", 0);
	GLOBAL_DEF("display/window/size/test_height", 0);
	GLOBAL_DEF("rendering/quality/intended_usage/framebuffer_allocation", 2);
	GLOBAL_DEF("rendering/quality/intended_usage/framebuffer_allocation.mobile", 3);

  String current_video_driver = "GLES3";
  int current_video_driver_idx = 0;
  GLOBAL_DEF("display/window/handheld/orientation", "landscape");

	ProjectSettings::get_singleton()->register_global_defaults();

//////////////////////////////////////
// Main setup2

	//os.initialize(current_videomode, current_video_driver_idx, 0);
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

	RasterizerGLES3::register_config();

	RasterizerGLES3::make_current();

	VisualServer* visual_server = memnew(VisualServerRaster);
  visual_server->init();

  //=====================================
  register_server_types();
	Color clear = GLOBAL_DEF("rendering/environment/default_clear_color", Color(0.3, 0.3, 0.3));
	VisualServer::get_singleton()->set_default_clear_color(clear);
	GLOBAL_DEF("application/config/icon", String());
	ProjectSettings::get_singleton()->set_custom_property_info("application/config/icon", PropertyInfo(Variant::STRING, "application/config/icon", PROPERTY_HINT_FILE, "*.png,*.webp"));

  register_scene_types();
	GLOBAL_DEF("display/mouse_cursor/custom_image", String());
	GLOBAL_DEF("display/mouse_cursor/custom_image_hotspot", Vector2());
	ProjectSettings::get_singleton()->set_custom_property_info("display/mouse_cursor/custom_image", PropertyInfo(Variant::STRING, "display/mouse_cursor/custom_image", PROPERTY_HINT_FILE, "*.png,*.webp"));

  ARVRServer* arvr_server = memnew(ARVRServer); // Needed for VisualServer::draw(), unfortunately
	//VisualServer* visual_server = memnew(VisualServerRaster);
  //Error err = Main::setup(argv[0], argc - 1, &argv[1]);

  //Main::start();

  //os.prepare_run();
  //Main::iteration();
  MainLoop* main_loop = memnew(TestMainLoop);
  main_loop->init(); // This create the cubes and add them to the visual server
  //os.set_main_loop(main_loop);
  std::cout << "before main loop..." << std::endl;

  do {
    glClear(GL_COLOR_BUFFER_BIT);
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

  //os.finish_run();
  //main_loop->finish(); // does nothing

  //Main::cleanup();

	unregister_scene_types();
	unregister_server_types();

  // os.finalize()
  memdelete(main_loop);
  visual_server->finish();
  memdelete(visual_server);

  // Main::cleanup()
  if (arvr_server)
    memdelete(arvr_server);

	if (globals)
		memdelete(globals);
  //if (engine)
    //memdelete(engine);

	unregister_core_driver_types();
	unregister_core_types();

	//os.clear_last_error();
	//os.finalize_core();

  glfwTerminate();

  return 0;//os.get_exit_code();
}
