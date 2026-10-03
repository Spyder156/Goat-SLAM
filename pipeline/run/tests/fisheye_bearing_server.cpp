// Offline audit IPC: every bearing and reprojection calls the production class.
#include "CameraModels/Fisheye624.h"
#include <cstdio>
#include <cstdint>
#include <vector>
int main() {
  uint32_t count;
  while (fread(&count, sizeof(count), 1, stdin) == 1) {
    if (!count) return 0;
    if (count > 1000000) return 2;
    double input_parameters[16];
    if (fread(input_parameters, sizeof(double), 16, stdin) != 16) return 3;
    std::vector<float> parameters(input_parameters, input_parameters + 16);
    ORB_SLAM3::Fisheye624 camera(parameters);
    std::vector<double> pixels(2 * count), output(5 * count);
    if (fread(pixels.data(), sizeof(double), pixels.size(), stdin) != pixels.size()) return 4;
    for (uint32_t i = 0; i < count; ++i) {
      cv::Point3f ray = camera.unproject(cv::Point2f(pixels[2*i], pixels[2*i+1]));
      Eigen::Vector3d direction(ray.x, ray.y, ray.z);
      Eigen::Vector2d pixel = camera.project(direction);
      output[5*i] = ray.x; output[5*i+1] = ray.y; output[5*i+2] = ray.z;
      output[5*i+3] = pixel.x(); output[5*i+4] = pixel.y();
    }
    if (fwrite(output.data(), sizeof(double), output.size(), stdout) != output.size()) return 5;
    fflush(stdout);
  }
  return 0;
}
