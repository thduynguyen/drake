#include <limits.h>
#include <locale.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include "godot_renderer.h"
#include "scene/3d/light.h"
#include "scene/3d/mesh_instance.h"
#include "scene/3d/camera.h"

#define DEGREE M_PI / 180.0

String path = "/home/duynguyen/git/godot-demo-projects/3d/material_testers/";

class Scene {
  Viewport* viewport_;
  Environment* environment;
  RID camera_, scenario_;

public:
  Scene() {
  }

  void initialize() {
    VisualServer* vs = VisualServer::get_singleton();
    camera_ = vs->camera_create();
    scenario_ = vs->scenario_create();
    viewport_ = memnew(Viewport);
  }

  void cleanup() {
    memdelete(viewport_);
  }

  ~Scene() {
  }
};

void SetupScene2(const GodotRenderer& renderer) {
  SceneTree* tree = memnew(SceneTree);
  tree->init();

  Spatial* scene = memnew(Spatial);
  tree->add_current_scene(scene); // need to do this here so all subsequent children knows about the tree, espcially the camera

  Camera* camera = memnew(Camera);
  camera->set_perspective(65.0, 0.1, 100.0);
  scene->add_child(camera);
  Transform Tc;
  Tc.set_look_at(Vector3(20., 10., 0.), Vector3(), Vector3(0., 1., 0.));
  camera->set_transform(Tc);
  camera->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED); // SceneTree::_flush_transform_notifications() is private, so we have to do this :(

  SpatialMaterial *material = memnew(SpatialMaterial);
  material->set_albedo(Color(0.81, 0.58, 0.36, 1.0));
  material->set_specular(0.9);
  material->set_metallic(0.2);
  material->set_roughness(0.0);
  Ref<Texture> texture = ResourceLoader::load(path + "texture_wood.png");
  material->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
  // IMPORTANT: This calls to SpatialMaterial::_update_shader(). Without this materials wont' work
  // Godot sets up so that this is called in SceneTree::call_idle_call_backs()
  // Not sure if we need to do this as an idle callback...
  SpatialMaterial::flush_changes();

  String filename = path + "godot_ball.mesh";
  // This calls all the way down to RasterizerStorageGLES3::mesh_add_surface(), which initializes VAO
  // RasterizerStorageGLES3::mesh_add_surface <-- ArrayMesh::add_surface <-- ArrayMesh::_setv() <-- res->set() <-- ResourceInteractiveLoaderBinary::poll()
  Ref<Mesh> mesh = ResourceLoader::load(filename);

  MeshInstance *mesh_instance = memnew(MeshInstance);
  scene->add_child(mesh_instance);
  // call this or any function that calls _instance_queue_update
  // before calling set_surface_material to update VS instance's materials
  //vs->instance_set_scenario(mesh_instance->get_instance(), scenario);
  // Without scenario, set_mesh->set_base->VS::instance_set_base DOES NOT call _instance_queue_update to update
  // VS instance's materials, causing a mismatch between MeshInstance's and its corresponding VS instance's materials
  mesh_instance->set_mesh(mesh);
  // Without instance_queue_update, the following will not work as VS instance's materials has size 0!
  mesh_instance->set_surface_material(0, material);
  mesh_instance->set_surface_material(1, material);
  mesh_instance->set_surface_material(2, material);
  mesh_instance->set_surface_material(3, material);

  OmniLight *light = memnew(OmniLight());
  scene->add_child(light);
  light->set_color(Color(1.0, 1.0, 1.0));
  light->set_param(Light::PARAM_ENERGY, 5.0);
  light->set_param(Light::PARAM_SPECULAR, 0.5);
  light->set_param(Light::PARAM_RANGE, 50.0);
  light->set_transform(Transform(Basis(), Vector3(12.5, 5., 0.)));
  light->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED);
}

void SetupScene(const GodotRenderer& renderer) {
  Viewport* viewport_ = memnew(Viewport);
  VisualServer *vs = VisualServer::get_singleton();
  RID scenario = vs->scenario_create();

  RID camera = vs->camera_create();

  //Transform Tc_in = Transform(Basis(Vector3(0.0, -90.0 * DEGREE, 0.0)).transposed(), Vector3(20.0, 0.0, 0.0));
  Transform Tc;
  Vector3 position;
  Tc.set_look_at(Vector3(20., 0., 0.), position, Vector3(0, 1, 0));
  vs->camera_set_transform(camera, Tc);
  vs->camera_set_perspective(camera, 65, 0.1, 100);

  RID viewport = viewport_->get_viewport_rid();
  Size2i screen_size = OS::get_singleton()->get_window_size();
  vs->viewport_set_size(viewport, screen_size.x, screen_size.y);
  vs->viewport_attach_to_screen(viewport, Rect2(Vector2(), screen_size));
  vs->viewport_set_active(viewport, true);
  vs->viewport_attach_camera(viewport, camera);
  vs->viewport_set_scenario(viewport, scenario);

  SpatialMaterial *material = memnew(SpatialMaterial);
  material->set_albedo(Color(0.81, 0.58, 0.36, 1.0));
  material->set_specular(0.9);
  material->set_metallic(0.2);
  material->set_roughness(0.0);
  Ref<Texture> texture = ResourceLoader::load(path + "texture_wood.png");
  material->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
  // IMPORTANT: This calls to SpatialMaterial::_update_shader(). Without this materials wont' work
  // Godot sets up so that this is called in SceneTree::call_idle_call_backs()
  // Not sure if we need to do this as an idle callback...
  SpatialMaterial::flush_changes();

  String filename = path + "godot_ball.mesh";
  // This calls all the way down to RasterizerStorageGLES3::mesh_add_surface(), which initializes VAO
  // RasterizerStorageGLES3::mesh_add_surface <-- ArrayMesh::add_surface <-- ArrayMesh::_setv() <-- res->set() <-- ResourceInteractiveLoaderBinary::poll()
  Ref<Mesh> mesh = ResourceLoader::load(filename);
//for (int i = 0; i<4; ++i)
//vs->mesh_surface_set_material(mesh->get_rid(), i, material->get_rid());

#if 0
  SpatialMaterial* material2 = memnew(SpatialMaterial);
  material2->set_albedo(Color(1.0, 0., 0.));
  SpatialMaterial::flush_changes();
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
  //RID test_cube = vs->get_test_cube();
  RID test_cube = vs->mesh_create();
  vs->mesh_add_surface_from_mesh_data(test_cube, md);
  RID instance = vs->instance_create2(test_cube, scenario);

  //RID mesh_id = mesh->get_rid();
  vs->mesh_surface_set_material(test_cube, 0, material2->get_rid());
  std::wcout << "NUM SURFACES: " << vs->mesh_get_surface_count(test_cube) << std::endl;
#endif

  // This calls vs->create_instance() in VisualInstance constructor
  // Using instance_create2() doesn't work, probably because the object_id is not attached to the instance
  // (VS::instance_attach_object_instance_id call in VisualInstance's ctor)
  // The instance is created and linked with the scenario, however, its data is missing.
  // Doing this is not ideal, however, as nobody manages this pointer, it's not freed at the end
  // hence the program doesn't exit naturally.
  MeshInstance *mesh_instance = memnew(MeshInstance);
  // call this or any function that calls _instance_queue_update
  // before calling set_surface_material to update VS instance's materials
  vs->instance_set_scenario(mesh_instance->get_instance(), scenario);
  // Without scenario, set_mesh->set_base->VS::instance_set_base DOES NOT call _instance_queue_update to update
  // VS instance's materials, causing a mismatch between MeshInstance's and its corresponding VS instance's materials
  mesh_instance->set_mesh(mesh);
  // Without instance_queue_update, the following will not work as VS instance's materials has size 0!
  mesh_instance->set_surface_material(0, material);
  mesh_instance->set_surface_material(1, material);
  mesh_instance->set_surface_material(2, material);
  mesh_instance->set_surface_material(3, material);
  Basis R(Vector3(0.0, 1.0, 0.0), 90.0 * DEGREE);
  R = Basis(Vector3(0.0, 0.0, 1.0), -10.0 * DEGREE) * R;
  vs->instance_set_transform(mesh_instance->get_instance(), Transform(R, Vector3(5, -5, -3.)));

  //vs->instance_set_transform(instance, Transform(Basis(), Vector3()));

  OmniLight *light = memnew(OmniLight());
  light->set_color(Color(1.0, 1.0, 1.0));
  light->set_param(Light::PARAM_ENERGY, 5.0);
  light->set_param(Light::PARAM_SPECULAR, 0.5);
  light->set_param(Light::PARAM_RANGE, 50.0);
  vs->instance_set_transform(light->get_instance(), Transform(Basis(), Vector3(12.5, 5., 0.)));
  vs->instance_set_scenario(light->get_instance(), scenario);

  String skyfilename = path + "park.hdr";
  Ref<Texture> sky_texture = ResourceLoader::load(skyfilename);
  PanoramaSky *sky = memnew(PanoramaSky);
  sky->set_panorama(sky_texture);
  Environment *env = memnew(Environment());
  env->set_background(Environment::BG_SKY);
  env->set_sky(sky);

  vs->camera_set_environment(camera, env->get_rid());
}

int main(int argc, char *argv[]) {
  GodotRenderer renderer(640, 480);
  renderer.Initialize();
  SetupScene2(renderer); // This create the cubes and add them to the visual server

  renderer.MainLoop();

  renderer.Cleanup();

  return 0; //os_.get_exit_code();
}
