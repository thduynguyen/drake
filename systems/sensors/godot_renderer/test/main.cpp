#include <limits.h>
#include <locale.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#include "godotvis/godot_renderer.h"
#include "godotvis/godot_scene.h"
#include <array>

using namespace godotvis;

std::string path = "/home/duynguyen/git/godot-demo-projects/3d/material_testers/";

int main(int argc, char *argv[]) {
  GodotRenderer renderer(1280, 960);
  renderer.Initialize();

  GodotScene scene;
  scene.Initialize();
  scene.SetupEnvironment(path + "night.hdr");
  scene.AddCamera(65.0, 0.1, 100.0);

  using Vec3 = std::array<double, 3>;
  using Mat33 = double[3][3];
  scene.SetCameraPose(Mat33{{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}}, Vec3{0., 2.5, 15.});

  int id = scene.AddMeshInstance(path + "godot_ball.mesh");
  double** R = new double*[3];
  for (int r = 0; r<3; ++r) {
    R[r] = new double[3];
    for (int c = 0; c<3; ++c)
      R[r][c] = 0.;
  }
  R[0][0] = R[1][1] = R[2][2] = 1.;
  scene.SetInstancePose(id, R, Vec3{0., 0., 0.});
  scene.SetInstanceScale(id, 0.8, 0.8, 0.8);

  int cube_id = scene.AddCubeInstance(1., 1., 1.);
  int sphere_id = scene.AddSphereInstance(1.);
  int cylinder_id = scene.AddCylinderInstance(0.5, 1.0);
  scene.SetInstancePose(sphere_id, R, Vec3{1.0, 0., 0.});
  scene.SetInstancePose(cylinder_id, R, Vec3{-1.0, 0., 0.});

  renderer.Draw();
  // TODO: Why can't I use Ref<Image> variable?
  scene.Capture()->save_png("/home/duynguyen/Downloads/rgb1.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/depth1.png");

  scene.ApplyMaterialShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/rgb2.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/depth2.png");

  scene.ApplyMaterialShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/rgb3.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/depth3.png");

  scene.Finish();
  renderer.Cleanup();

  return 0; //os_.get_exit_code();
}
