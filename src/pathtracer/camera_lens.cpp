#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {

  // Part 2, Task 4:
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.
  // Normalized Image space -> Camera Space
  double hScale = tan(0.5 * hFov / 180 * PI);
  double vScale = tan(0.5 * vFov / 180 * PI);
  double hSensor = -1 * hScale + (2 * hScale * x);
  double ySensor = -1 * vScale + (2 * vScale * y);
  Vector3D sensorPt = Vector3D(-hSensor, -ySensor, 1);

  // Compute the ray direction through the center of lens.
  Vector3D focusDir = (-1 * sensorPt).unit();

  // Compute pFocus and pLens
  double t = (-1 * this->focalDistance) / focusDir.z;
  Vector3D pFocus = focusDir * t;
  Vector3D pLens = Vector3D(lensRadius * cos(rndTheta), lensRadius * sin(rndTheta), 0);
  pLens *= sqrt(rndR);

  // Create final ray.
  Vector3D rayDir = (c2w * (pFocus - pLens)).unit();
  Ray finalRay = Ray(pLens + pos, rayDir);
  finalRay.depth = 1;
  finalRay.min_t = nClip;
  finalRay.max_t = fClip;
  return finalRay;
  
  // return Ray(pos, Vector3D(0, 0, -1));
}


} // namespace CGL
