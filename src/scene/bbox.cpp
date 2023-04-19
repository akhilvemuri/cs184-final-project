#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {


Vector3D ray_plane_intersect(const Ray& r, Vector3D val) {
	return (val - r.o) / r.d;
}

bool intersect_helper(double ray0, double ray1, double pt0, double pt1) {
	if (pt0 <= pt1 && pt0 < ray1 && pt1 > ray0) {
		return true;
	}
	return false;
}

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
	
  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
	Vector3D entry = ray_plane_intersect(r, min);
	Vector3D exit = ray_plane_intersect(r, max);
	for (int i = 0; i < 3; i++) {
		if (entry[i] > exit[i]){
			std::swap(entry[i], exit[i]);
		}
	}
	double t_min = std::max(entry[0], std::max(entry[1], entry[2]));
	double t_max = std::min(exit[0], std::min(exit[1], exit[2]));
	//return true; 
	if (!intersect_helper(t0, t1, t_min, t_max)) {
		return false;
	}
	else {
		t0 = t_min; 
		t1 = t_max;
		return true;

	}

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
