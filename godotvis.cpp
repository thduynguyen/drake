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

void SetupScene(const GodotRenderer& renderer) {
  SceneTree* tree = memnew(SceneTree);
  tree->init();

  String skyfilename = path + "night.hdr";
  Ref<Texture> sky_texture = ResourceLoader::load(skyfilename);
  PanoramaSky *sky = memnew(PanoramaSky);
  sky->set_panorama(sky_texture);
  sky->set_radiance_size(Sky::RADIANCE_SIZE_64);
  Environment* env = memnew(Environment);
  env->set_background(Environment::BG_SKY);
  env->set_sky(sky);
  env->set_bg_energy(1.0);
  tree->get_root()->get_world()->set_environment(env);

  Spatial* scene = memnew(Spatial);
  tree->add_current_scene(scene); // need to do this here so all subsequent children knows about the tree, espcially the camera

  Camera* camera = memnew(Camera);
  scene->add_child(camera);
  camera->set_perspective(65.0, 0.1, 100.0);
  Transform Tc(Basis(), Vector3(0, 2.5, 15.0));
  camera->set_transform(Tc);
  camera->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED); // SceneTree::_flush_transform_notifications() is private, so we have to do this :(

  SpatialMaterial *material = memnew(SpatialMaterial);
  material->set_albedo(Color(1.0, 1.0, 1.0, 1.0));
  Ref<Texture> texture = ResourceLoader::load(path + "aluminium_albedo.png");
  material->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
  material->set_metallic(0.59);
  material->set_specular(0.5);
  material->set_roughness(0.4);
  Ref<Texture> normal_tex = ResourceLoader::load(path + "aluminium_normal.png");
  material->set_feature(SpatialMaterial::FEATURE_NORMAL_MAPPING, true);
  material->set_texture(SpatialMaterial::TEXTURE_NORMAL, normal_tex);
  material->set_normal_scale(0.21);
  material->set_feature(SpatialMaterial::FEATURE_ANISOTROPY, true);
  material->set_anisotropy(0.99);
  // IMPORTANT: This calls to SpatialMaterial::_update_shader(). Without this materials wont' work
  // Godot sets up so that this is called in SceneTree::call_idle_call_backs()
  // Not sure if we need to do this as an idle callback...
  SpatialMaterial::flush_changes();

  String filename = path + "godot_ball.mesh";
  // This calls all the way down to RasterizerStorageGLES3::mesh_add_surface(), which initializes VAO
  // RasterizerStorageGLES3::mesh_add_surface <-- ArrayMesh::add_surface <-- ArrayMesh::_setv() <-- res->set() <-- ResourceInteractiveLoaderBinary::poll()
  Ref<Mesh> mesh = ResourceLoader::load(filename);

  MeshInstance *mesh_instance = memnew(MeshInstance);
  scene->add_child(mesh_instance); // Must add before doing any settings
  mesh_instance->set_mesh(mesh);
  mesh_instance->set_surface_material(0, material);
  mesh_instance->set_surface_material(1, material);
  mesh_instance->set_surface_material(2, material);
  mesh_instance->set_surface_material(3, material);
  Basis R(Vector3(0.0, 1.0, 0.0), 90.0 * DEGREE);
  R = Basis(Vector3(0.0, 0.0, 1.0), -10.0 * DEGREE) * R;
  mesh_instance->set_transform(Transform(Basis(), Vector3(0, 0.0, 0.)));
  mesh_instance->set_scale(Vector3(0.8, 0.8, 0.8));
  mesh_instance->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED);

  OmniLight *light = memnew(OmniLight());
  scene->add_child(light);
  light->set_color(Color(1.0, 1.0, 1.0));
  light->set_param(Light::PARAM_ENERGY, 2.0);
  light->set_param(Light::PARAM_RANGE, 50.0);
  light->set_transform(Transform(Basis(), Vector3(1.0, 5., 10.)));
  light->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED);

}


int main(int argc, char *argv[]) {
  GodotRenderer renderer(1280, 960);
  renderer.Initialize();
  SetupScene(renderer); // This create the cubes and add them to the visual server

  renderer.MainLoop();

  renderer.Cleanup();

  return 0; //os_.get_exit_code();
}
