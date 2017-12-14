#include "scene/3d/light.h"
#include "scene/3d/mesh_instance.h"
#include "scene/3d/camera.h"
#include "scene/resources/primitive_meshes.h"

#include <string>
#include <vector>

namespace godotvis {

class GodotScene {
  SceneTree* tree_ = NULL;
  Spatial* scene_root_ = NULL; //!< This will be freed by scene tree. Keep the pointer to add intances.
  Camera* camera_ = NULL; //!< This will be freed by scene tree. Keep the pointer to update.
  Environment* env = NULL;
  MeshInstance* mesh_instance_ = NULL;
  Ref<ShaderMaterial> shader_material_;
  SpatialMaterial* material = NULL;
  CubeMesh* cube_ = nullptr;
  SphereMesh* sphere_ = nullptr;
  CylinderMesh* cylinder_ = nullptr;
  std::vector<int> mesh_instance_ids_;

public:
  GodotScene() {}

  void Initialize();
  void SetupEnvironment(const std::string& env_filename);

  /// Add a camera to the scene. Only support one camera for now.
  /// TODO: handle distortion
  void AddCamera(double fov_y, double z_near, double z_far);

  template<class MAT3, class VEC3>
  void SetCameraPose(const MAT3& r, const VEC3& t);

  Ref<Image> Capture();
  void ApplyDepthShader();
  void ApplyMaterialShader();
  void Finish();
  int AddMeshInstance(const std::string& filename);
  int AddCubeInstance(double x_length, double y_length, double z_length);
  int AddSphereInstance(double radius);
  int AddCylinderInstance(double radius, double height);

  template<class MAT3, class VEC3>
  void SetInstancePose(int id, const MAT3& r, const VEC3& t);

  template <class MAT3, class VEC3>
  void SetInstancePose(Spatial* instance, const MAT3& r, const VEC3& t);

  void SetInstanceScale(int id, double sx, double sy, double sz);

private:
  void InitDepthShader();
  Spatial* get_spatial_instance(int id);
  Ref<Mesh> LoadMesh(const std::string& filename);
  int AddInstance(const Ref<Mesh>& mesh);
};

} // namespace godotvis

#include "godot_scene.inl"
