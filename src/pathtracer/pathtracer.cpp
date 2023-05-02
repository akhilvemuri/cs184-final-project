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


#define FOG_ON

#ifdef FOG_ON
double rand_fog_t(const Ray &r) {
  double fog_amt = 0.5;
  return - log(random_uniform()) / fog_amt / r.d.norm();
}

double prob_fog_t_greater_than(const Ray &r, double t) {
  double fog_amt = 0.5;
  return exp(- t * fog_amt * r.d.norm());
}

Vector3D fog_f(const Vector3D wo, const Vector3D wi) {
  double cos_theta = dot(wo.unit(), wi.unit());
  double g = 0.6;
  double idk_stuff = (1 - g * g) / pow(1 + g * g - 2 * g * cos_theta, 1.5);

  return Vector3D(1,1,1) * idk_stuff / (4.0 * PI);
}


Vector3D sample_fog(const Vector3D wo, Vector3D* wi, double* pdf) {
  double g = 0.6;
  double inner = (1 - g * g) / (1 - g + 2 * g * random_uniform());
  double z = (1 / (2 * g)) * (1 + g * g - inner * inner);
  double sinTheta = sqrt(std::max(0.0, 1.0f - z * z));

  double phi = 2.0f * PI * random_uniform();

  // relative_dir is relative to wo, where (0, 0, 1) is wo
  // need to transform to "world" coordinates 
  double idk_stuff = (1 - g * g) / pow(1 + g * g - 2 * g * z, 1.5);
  Vector3D relative_dir = Vector3D(cos(phi) * sinTheta, sin(phi) * sinTheta, z);

  Matrix3x3 o2w;
  make_coord_space(o2w, wo);
  *wi = o2w * relative_dir;
  *pdf = idk_stuff / (4.0 * PI); // should cancel out fog_f

  // cout << "cheese " << relative_dir <<  " " << *pdf << " " << fog_f(wo, *wi) << "\n";

  return fog_f(wo, *wi) / abs_cos_theta(*wi);
  // divide by abs_cos_theta to cancel it out
  // really should be in f function but we don't do costheta thing in estimate_fog
}
#endif

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

        Ray r2 = Ray(o, d);
        r2.min_t = EPS_F;
        r2.max_t = distToLight - EPS_F;

        Intersection isect2;
        if (!bvh->intersect(r2, &isect2)) {
          #ifdef FOG_ON
          double prob_no_fog = prob_fog_t_greater_than(r2, distToLight);
          #else
          double prob_no_fog = 1;
          #endif
          Vector3D f = isect.bsdf->f(w_out, w_in);
          L_out += L_in * f * abs_cos_theta(w_in) * prob_no_fog / pdf;
        }
      }
    }

    L_out /= num_samples;
  }

  return L_out;
}

#ifdef FOG_ON
Vector3D PathTracer::estimate_fog(const Ray &r, const Vector3D hit_p) {

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

      Vector3D d = wi;
      Vector3D o = hit_p;

      Ray r2 = Ray(o, d);
      r2.min_t = EPS_F;
      r2.max_t = distToLight - EPS_F;

      Intersection isect2;
      if (!bvh->intersect(r2, &isect2)) {
        double prob_no_fog = prob_fog_t_greater_than(r2, distToLight);
        L_out += L_in * fog_f(-r.d, wi) * prob_no_fog / pdf;
      }
      
    }

    L_out /= num_samples;
  }

  return L_out;
  
}
#endif

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  #ifdef FOG_ON
  double rand_t = rand_fog_t(r);
  if (rand_t < isect.t) {
    return Vector3D();
  }
  #endif
  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  #ifdef FOG_ON
  double rand_t = rand_fog_t(r);
  if (rand_t < isect.t) {
    const Vector3D hit_p = r.o + r.d * rand_t;
    return estimate_fog(r, hit_p);
  }
  #endif

  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  } else {
    return estimate_direct_lighting_importance(r, isect);
  }
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  if (r.depth <= 0) {
    return Vector3D();
  }
  
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.


  Vector3D L_out = Vector3D();
  
  if (!isect.bsdf->is_delta())
    L_out += one_bounce_radiance(r, isect);

  double rr_prob = 0.9;
  if (!coin_flip(rr_prob)) { // russian roulette
    return L_out;
  }

  Vector3D w_in;
  double pdf;
  Vector3D reflectance;

  #ifdef FOG_ON 
  double rand_t = rand_fog_t(r);
  if (rand_t < isect.t) {
    const Vector3D hit_fog = r.o + r.d * rand_t;
    reflectance = sample_fog(w_out, &w_in, &pdf);
  } else {
    reflectance = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  }
  #else
    reflectance = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  #endif


  Ray new_ray = Ray(hit_p, o2w * w_in, (int)(r.depth - 1));
  new_ray.min_t = EPS_F;
  Intersection new_isect;
  double cos_term = abs_cos_theta(w_in);
  if (bvh->intersect(new_ray, &new_isect)) {
    Vector3D L_in = at_least_one_bounce_radiance(new_ray, new_isect);
    if (isect.bsdf->is_delta())
      L_in += zero_bounce_radiance(new_ray, new_isect);
    L_out += L_in * reflectance * cos_term / pdf / rr_prob;
  } else {
    // rand_t = rand_fog_t(new_ray);
    // const Vector3D hit_fog = new_ray.o + new_ray.d * rand_t;
    // Vector3D L_in = estimate_fog(new_ray, hit_fog);
    // L_out += L_in * reflectance * cos_term / pdf / rr_prob;
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

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.

  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

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
  // int num_samples = ns_aa;          // total samples to evaluate
  // Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  //// Part 1.2: 
  //Vector3D radiance = Vector3D();
  //for (int i = 0; i < ns_aa; i++) {
  //   // Generating the rays
  //    Vector2D sample = gridSampler->get_sample();
  //    Ray r = camera->generate_ray((x + sample.x)/sampleBuffer.w, (y + sample.y)/sampleBuffer.h);
  //    r.depth = max_ray_depth;
  //    radiance = radiance + est_radiance_global_illumination(r);
  //}
  //radiance = radiance / num_samples;
  //sampleBuffer.update_pixel(radiance, x, y);
  //sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;

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
