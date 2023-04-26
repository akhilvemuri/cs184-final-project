#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>



#include <time.h>



using namespace std;

namespace CGL {
    namespace SceneObjects {

        BVHAccel::BVHAccel(const std::vector<Primitive*>& _primitives,
            size_t max_leaf_size) {

            primitives = std::vector<Primitive*>(_primitives);
            root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
        }

        BVHAccel::~BVHAccel() {
            if (root)
                delete root;
            primitives.clear();
        }

        BBox BVHAccel::get_bbox() const { return root->bb; }

        void BVHAccel::draw(BVHNode* node, const Color& c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->draw(c, alpha);
                }
            }
            else {
                draw(node->l, c, alpha);
                draw(node->r, c, alpha);
            }
        }

        void BVHAccel::drawOutline(BVHNode* node, const Color& c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->drawOutline(c, alpha);
                }
            }
            else {
                drawOutline(node->l, c, alpha);
                drawOutline(node->r, c, alpha);
            }
        }


        BVHNode* BVHAccel::construct_bvh(std::vector<Primitive*>::iterator start,
            std::vector<Primitive*>::iterator end,
            size_t max_leaf_size) {



            // TODO (Part 2.1):
            // Construct a BVH from the given vector of primitives and maximum leaf
            // size configuration. The starter code build a BVH aggregate with a
            // single leaf node (which is also the root) that encloses all the
            // primitives.


            BBox bbox;
            int cnt = 0;

            for (auto p = start; p != end; p++) {
                BBox bb = (*p)->get_bbox();
                bbox.expand(bb);
                cnt = cnt + 1;
            }

            BVHNode* node = new BVHNode(bbox);

            if (cnt <= max_leaf_size) {
                node->start = start;
                node->end = end;
                return node;
            }
            // Split along which axis

            Vector3D avg = Vector3D();
            for (auto p = start; p != end; p++) {
                Vector3D centroid = (*p)->get_bbox().centroid();
                avg = avg + centroid;
            }
            avg = avg / cnt;

            int x_cnt = 0;
            int y_cnt = 0;
            int z_cnt = 0;
            for (auto p = start; p != end; p++) {
                Vector3D centroid = (*p)->get_bbox().centroid();
                if (centroid.x <= avg.x) { x_cnt += 1; };
                if (centroid.y <= avg.y) { y_cnt += 1; };
                if (centroid.z <= avg.z) { z_cnt += 1; };
            }


            float mp_list[] = { (float)(x_cnt * (cnt - x_cnt)), (float)(y_cnt * (cnt - y_cnt)), (float)(z_cnt * (cnt - z_cnt)) };
            int max_axis = max(mp_list[0], max(mp_list[1], mp_list[2]));

            int ind;
            float threshold;
            if (max_axis == mp_list[0]) {
                ind = 0;
                threshold = avg.x;
                std::sort(start, end, [](const Primitive* a, const Primitive* b)
                    {
                        return a->get_bbox().centroid().x < b->get_bbox().centroid().x;
                    });
            }
            else if (max_axis == mp_list[1]) {
                ind = 1;
                threshold = avg.y;
                std::sort(start, end, [](const Primitive* a, const Primitive* b)
                    {
                        return a->get_bbox().centroid().y < b->get_bbox().centroid().y;
                    });
            }
            else {
                ind = 2;
                threshold = avg.z;
                std::sort(start, end, [](const Primitive* a, const Primitive* b)
                    {
                        return a->get_bbox().centroid().z < b->get_bbox().centroid().z;
                    });
            }



            // std::vector<Primitive*> left = std::vector<Primitive*>();
            // std::vector<Primitive*> right = std::vector<Primitive*>();
            std::vector<Primitive*>::iterator bound;
            for (auto p = start; p != end; p++) {
                float centroid = (*p)->get_bbox().centroid()[ind];
                if (centroid > threshold) {
                    bound = p;
                    break;
                }
            }




            node->l = construct_bvh(start, bound, max_leaf_size);
            node->r = construct_bvh(bound, end, max_leaf_size);


            return node;


        }

        bool BVHAccel::has_intersection(const Ray& ray, BVHNode* node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            // Take note that this function has a short-circuit that the
            // Intersection version cannot, since it returns as soon as it finds
            // a hit, it doesn't actually have to find the closest hit.
            double t0 = ray.min_t;
            double t1 = ray.max_t;

            if (!node->bb.intersect(ray, t0, t1)) {
                return false;
            }
            else if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    total_isects++;
                    if ((*p)->has_intersection(ray))
                        return true;
                }
                return false;
            }
            else {
                return has_intersection(ray, node->l) || has_intersection(ray, node->r);
            }
        }

        bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode* node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.

            double t0 = ray.min_t;
            double t1 = ray.max_t;
            if (!node->bb.intersect(ray, t0, t1)) {
                return false;
            }
            else if (node->isLeaf()) {
                bool hit = false;
                for (auto p = node->start; p != node->end; p++) {
                    total_isects++;
                    hit = (*p)->intersect(ray, i) || hit;
                }
                return hit;
            }
            else {
                bool a = intersect(ray, i, node->l);
                bool b = intersect(ray, i, node->r);
                return (a || b);
            }
        }

    } // namespace SceneObjects
} // namespace CGL
