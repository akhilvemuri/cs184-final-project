#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {




bool Sphere::test(const Ray &r, double &t1, double &t2) const {

    // TODO (Part 1.4):
    // Implement ray - sphere intersection test.
    // Return true if there are intersections and writing the
    // smaller of the two intersection times in t1 and the larger in t2.
    float a = CGL::dot(r.d, r.d);
    float b = 2 * CGL::dot((r.o - this->o), r.d);
    float c = CGL::dot((r.o - this->o), (r.o - this->o)) - pow(this->r, 2);
    float delta = b * b - 4 * a * c;
    if (delta < 0) {
        return false;
    }
    double tmp1 = (- b + pow(delta, 0.5)) / (2 * a);
    double tmp2 = (- b - pow(delta, 0.5)) / (2 * a);
    t1 = min(tmp1, tmp2);
    t2 = max(tmp1, tmp2);
    return true;

}


bool Sphere::has_intersection(const Ray &r) const {

    // TODO (Part 1.4):
    // Implement ray - sphere intersection.
    // Note that you might want to use the the Sphere::test helper here.
    double t1;
    double t2;
    
    bool intersects = test(r, t1, t2);

    if (!intersects) {
        return false;
    }
    else if (t1 >= 0 && t1 >= r.min_t && t1 <= r.max_t) {
        r.max_t = t1;
        return true;
    }
    else if (t2 >= 0 && t2 >= r.min_t && t2 <= r.max_t) {
        r.max_t = t2;
        return true;
    }
    return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

    // TODO (Part 1.4):
    // Implement ray - sphere intersection.
    // Note again that you might want to use the the Sphere::test helper here.
    // When an intersection takes place, the Intersection data should be updated
    // correspondingly.

    double t1 = 0;
    double t2 = 0;
    bool intersects = test(r, t1, t2);
    if (!intersects) {
        return false;
    }
    else if (t1 >= 0 && t1 >= r.min_t && t1 <= r.max_t) {
        r.max_t = t1;
        i->t = t1;
        Vector3D intersectionpoint = r.o + t1 * r.d;
        Vector3D normal = intersectionpoint - this->o;
        i->n = normal/normal.norm();
        i->primitive = this;
        i->bsdf = get_bsdf();
        return true;
    } 
    else if (t2 >= 0 && t2 >= r.min_t && t2 <= r.max_t) {
        r.max_t = t2;
        i->t = t2;
        Vector3D intersectionpoint = r.o + t2 * r.d;
        Vector3D normal = intersectionpoint - this->o;
        i->n = normal / normal.norm();
        i->primitive = this;
        i->bsdf = get_bsdf();
        return true;
    }
    return false;

}

void Sphere::draw(const Color &c, float alpha) const {
    Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
    // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
