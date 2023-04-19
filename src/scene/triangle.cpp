#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
    p1 = mesh->positions[v1];
    p2 = mesh->positions[v2];
    p3 = mesh->positions[v3];
    n1 = mesh->normals[v1];
    n2 = mesh->normals[v2];
    n3 = mesh->normals[v3];
    bbox = BBox(p1);
    bbox.expand(p2);
    bbox.expand(p3);

    bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }


// Compute (t, b1, b2) using Moller Trumbore Algorithm

Vector3D intersect_helper(Vector3D p1, Vector3D p2, Vector3D p3, const Ray& r) {
    Vector3D E1 = p2 - p1;
    Vector3D E2 = p3 - p1;
    Vector3D S = r.o - p1;
    Vector3D S1 = CGL::cross(r.d, E2);
    Vector3D S2 = CGL::cross(S, E1);
    Vector3D intersection_point = Vector3D(CGL::dot(S2, E2), CGL::dot(S1, S), CGL::dot(S2, r.d)) / CGL::dot(S1, E1);
    return intersection_point;
}

bool Triangle::has_intersection(const Ray &r) const {
    // Part 1, Task 3: implement ray-triangle intersection
    // The difference between this function and the next function is that the next
    // function records the "intersection" while this function only tests whether
    // there is a intersection.

    Vector3D intersection_point = intersect_helper(p1, p2, p3, r);
    float t = intersection_point.x;
    float b1 = intersection_point.y;
    float b2 = intersection_point.z;
    if (t >= 0 && b1 >= 0 && b2 >= 0 && 1 - b1 - b2 >= 0 && t >= r.min_t && t <= r.max_t) {
        r.max_t = t;
        return true;
    }
    return false;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
    // Part 1, Task 3:
    // implement ray-triangle intersection. When an intersection takes
    // place, the Intersection data should be updated accordingly
    Vector3D intersection_point = intersect_helper(p1, p2, p3, r);
    float t = intersection_point.x;
    float b1 = intersection_point.y;
    float b2 = intersection_point.z;
    if (t >= 0 && b1 >= 0 && b2 >= 0 && 1-b1-b2 >= 0 && t >= r.min_t && t <= r.max_t) {
        r.max_t = t;
        isect->t = t;
        isect->n = (1 - b1 - b2) * n1 + b1 * n2 + b2 * n3;
        isect->primitive = this;
        isect->bsdf = get_bsdf();
        return true;
    }
    return false;



}

void Triangle::draw(const Color &c, float alpha) const {
    glColor4f(c.r, c.g, c.b, alpha);
    glBegin(GL_TRIANGLES);
    glVertex3d(p1.x, p1.y, p1.z);
    glVertex3d(p2.x, p2.y, p2.z);
    glVertex3d(p3.x, p3.y, p3.z);
    glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
    glColor4f(c.r, c.g, c.b, alpha);
    glBegin(GL_LINE_LOOP);
    glVertex3d(p1.x, p1.y, p1.z);
    glVertex3d(p2.x, p2.y, p2.z);
    glVertex3d(p3.x, p3.y, p3.z);
    glEnd();
}

} // namespace SceneObjects
} // namespace CGL
