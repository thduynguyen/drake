#pragma once

namespace godotvis {

template<class MAT3, class VEC3>
void GodotScene::SetInstancePose(int id, const MAT3& r, const VEC3& t) {
  Spatial* instance = get_spatial_instance(id);
  SetInstancePose(instance, r, t);
}

template <class MAT3, class VEC3>
void GodotScene::SetInstancePose(Spatial* instance, const MAT3& r, const VEC3& t) {
  instance->set_transform(
      Transform(Basis(r[0][0], r[0][1], r[0][2],
                      r[1][0], r[1][1], r[1][2],
                      r[2][0], r[2][1], r[2][2]),
                Vector3(t[0], t[1], t[2])));
  instance->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED);
}

template<class MAT3, class VEC3>
void GodotScene::SetCameraPose(const MAT3& r, const VEC3& t) {
  SetInstancePose(camera_, r, t);
}

} // namespace godotvis
