#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 

  for (int n = 0; n < num_samples; n++) {
    Vector3D w_in = hemisphereSampler->get_sample();  // object space
    Vector3D d = o2w * w_in;
    Vector3D o = hit_p;

    Ray r2 = Ray(o, d);
    r2.min_t = EPS_F;
    
    Intersection isect2;
    if (bvh->intersect(r2, &isect2)) {
      // TODO: ADD COMPUTATION HERE?
      Vector3D f = isect.bsdf->f(w_out, w_in);
      Vector3D L_in = isect2.bsdf->get_emission();
      L_out += L_in * f * abs_cos_theta(w_in) / (1 / (2 * PI));
    }
  }

  L_out /= num_samples;
  return L_out;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  for (auto light : scene->lights) {
    int num_samples;
    if (light->is_delta_light()) {
      num_samples = 1;
    } else {
      num_samples = ns_area_light;
    }

    Vector3D wi;  // world space
    double distToLight;
    double pdf;
    
    for (int n = 0; n < num_samples; n++) {
      Vector3D L_in = light->sample_L(hit_p, &wi, &distToLight, &pdf);  // sample_L returns wi in world space
      Vector3D w_in = w2o * wi;  // cos_theta() calculates vectors using object space
      
      if (abs_cos_theta(w_in) > 0) {
        Vector3D d = wi;
        Vector3D o = hit_p;
        
        // Attempt an Intersect
        Ray r2 = Ray(o, d);
        r2.min_t = EPS_F;
        r2.max_t = distToLight - EPS_F;
        bool blocked = false;
        for (double i = r2.min_t; i < r2.max_t; i+=0.1) {
          if (coin_flip(0.015)) {
            blocked = true;
          }
        }
//        default_random_engine generator;
//        exponential_distribution<double> distribution (0.15);
//        double number = distribution(generator);
//        if (number < r2.max_t) blocked = true;
        
        Intersection isect2;
        if (!blocked) blocked = bvh->intersect(r2, &isect2);
        if (!blocked) {
          Vector3D f = isect.bsdf->f(w_out, w_in);
          L_out += L_in * f * abs_cos_theta(w_in) / pdf;
        }
      }
    }
    // TODO: ADD SOME STUFF HERE
    L_out /= num_samples;
  }

  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  double reduction = 1; //pow(0.90,isect.t * 0.5);
  return isect.bsdf->get_emission() * reduction;
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  double reduction = 1; //pow(0.90,isect.t * 0.5);
  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect) * reduction;
  } else {
    return estimate_direct_lighting_importance(r, isect) * reduction;
  }
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  if (!isect.bsdf->is_delta())
    L_out += one_bounce_radiance(r, isect);

  Vector3D w_in;
  double pdf;
  double cpdf = 0.65;

  if (r.depth == max_ray_depth || (coin_flip(cpdf) && r.depth >= 1)) {
    Vector3D f = isect.bsdf->sample_f(w_out, &w_in, &pdf);
    Vector3D d = o2w * w_in;
    Vector3D o = hit_p;

    Ray r2 = Ray(o, d);
    r2.depth = r.depth - 1;
    r2.min_t = EPS_F;
    Intersection isect2;

    if (abs_cos_theta(w_in) > 0 && bvh->intersect(r2, &isect2)) {
      // Simulate Hitting Particle
      for (double i = 0; i < isect2.t; i+= 0.1) {
        if (coin_flip(0.015)) {
          isect2.t = i;
          isect2.bsdf = new DiffuseBSDF(Vector3D(1,1,1));
          isect2.n = -r2.d;
          break;
        }
      }
      Vector3D L_in = at_least_one_bounce_radiance(r2, isect2);
      if (isect.bsdf->is_delta())
        L_in += zero_bounce_radiance(r2, isect2);
      double reduction = 1; //pow(0.70,isect.t * 1);
      L_out += L_in * f * abs_cos_theta(w_in) / pdf / cpdf * reduction;
    }
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;
    
  // Simulate Hitting Particle
  for (double i = 0; i < isect.t; i+= 0.1) {
    if (coin_flip(0.015)) {
      isect.t = i;
      isect.bsdf = new DiffuseBSDF(Vector3D(1,1,1));
      break;
    }
  }
//  default_random_engine generator;
//  exponential_distribution<double> distribution (0.015);
//  double number = distribution(generator);
//  if (number < isect.t) {
//    isect.t = number;
//    isect.bsdf = new DiffuseBSDF(Vector3D(1,1,1));
//  }
  
  // TODO (Part 3): Return the direct illumination.
  L_out += zero_bounce_radiance(r, isect);
  if (r.depth == 1) {
    L_out += one_bounce_radiance(r, isect);
  }
  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  else if (r.depth > 1) {
    L_out += at_least_one_bounce_radiance(r, isect);
  }
  double reduction = 1; //pow(0.70,isect.t * 0.5);
  L_out *= reduction;
  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  // Part 5
  Vector3D radiance = Vector3D();
  float s1 = 0;
  float s2 = 0;
  int num_samples = ns_aa;
  for (int i = 0; i < ns_aa; i++) {
      // Generating the rays
      Vector2D sample = gridSampler->get_sample();
      Ray r = camera->generate_ray((x + sample.x) / sampleBuffer.w, (y + sample.y) / sampleBuffer.h);
      r.depth = max_ray_depth;
      Vector3D sample_radiance = est_radiance_global_illumination(r);
      radiance = radiance + sample_radiance;
      float illuminance = sample_radiance.illum();
      s1 = s1 + illuminance;
      s2 = s2 + pow(illuminance, 2);
      if (i % samplesPerBatch == 0) {
          int n = i + 1;
          float mu = s1 / n;
          float var = (s2 - pow(s1, 2) / n) / (n - 1);
          float I = 1.96 * pow(var, 0.5) / pow(n, 0.5);
          if (I <= maxTolerance * mu) {
              num_samples = n;
              break;
          }
      }
  }
  radiance = radiance / num_samples;
  sampleBuffer.update_pixel(radiance, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;

}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
