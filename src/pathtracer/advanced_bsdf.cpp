#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>
#define _USE_MATH_DEFINES
#include <math.h>


#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

    Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
        return Vector3D();
    }

    Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

        // TODO Project 3-2: Part 1
        // Implement MirrorBSDF
        reflect(wo, wi);
        *pdf = 1.0;
        return reflectance / abs_cos_theta(*wi);
    }

    void MirrorBSDF::render_debugger_node()
    {
        if (ImGui::TreeNode(this, "Mirror BSDF"))
        {
            DragDouble3("Reflectance", &reflectance[0], 0.005);
            ImGui::TreePop();
        }
    }

// Microfacet BSDF //

    double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
        return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
    }

    double MicrofacetBSDF::D(const Vector3D h) {
        // TODO Project 3-2: Part 2
        // Compute Beckmann normal distribution function (NDF) here.
        // You will need the roughness alpha.
        Vector3D n = Vector3D(0, 0, 1);
        double theta_h = acos(dot(h, n));
        double d_value = exp(-pow(tan(theta_h), 2) / pow(alpha, 2));
        d_value = d_value / (M_PI * pow(alpha, 2) * pow(cos(theta_h), 4));
        return d_value;
    }

    Vector3D MicrofacetBSDF::F(const Vector3D wi) {
        // TODO Project 3-2: Part 2
        // Compute Fresnel term for reflection on dielectric-conductor interface.
        // You will need both eta and etaK, both of which are Vector3D.
        Vector3D n = Vector3D(0, 0, 1);

        double cos_wi = dot(wi/wi.norm(), n);
        // Vector3D cos_wi = Vector3D(cos(wi.x), cos(wi.y), cos(wi.z));
        Vector3D R_s = (eta * eta + k * k - 2 * eta * cos_wi + cos_wi * cos_wi) / (eta * eta + k * k + 2 * eta * cos_wi + cos_wi * cos_wi);
        Vector3D R_p = ((eta * eta + k * k) * cos_wi * cos_wi - 2 * eta * cos_wi + 1) / ((eta * eta + k * k) * cos_wi * cos_wi + 2 * eta * cos_wi + 1);
        return (R_s + R_p) / 2;
    }

    Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
        // TODO Project 3-2: Part 2
        // Implement microfacet model here.
        Vector3D h = (wo + wi) / (wo + wi).norm();
        Vector3D n = Vector3D(0, 0, 1);
        if (dot(n, wo) <= 0 || dot(n, wi) <= 0) {
            return Vector3D();
        }
        Vector3D result = (F(wi) * G(wo, wi) * D(h)) / (4 * dot(n, wo) * dot(n, wi));
        return result;
    }

    Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
        // TODO Project 3-2: Part 2
        // *Importance* sample Beckmann normal distribution function (NDF) here.
        // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
        //       and return the sampled BRDF value.


        Vector2D r_sample = sampler.get_sample();
        double r1 = r_sample.x;
        double r2 = r_sample.y;
        double alpha_2 = pow(alpha, 2);

        double theta_h = atan(pow(-alpha_2 * log(1 - r1), 0.5));
        double phi_h = 2 * M_PI * r2;
        double p_theta_h = (2 * sin(theta_h) / (alpha_2 * pow(cos(theta_h), 3))) * exp(-pow(tan(theta_h), 2) / alpha_2);
        double p_phi_h = 1 / (2 * M_PI);
        Vector3D h = Vector3D(sin(theta_h) * cos(phi_h), sin(theta_h) * sin(phi_h), cos(theta_h));
        Vector3D orthog = dot(wo, h) * h;
        *wi = 2 * orthog - wo;
        // *wi = (h - wo) / (h - wo).norm();
        if (dot(Vector3D(0, 0, 1), *wi) <= 0) {
            *pdf = 0;
            return 0;
        }
        double p_h = p_theta_h * p_phi_h / sin(theta_h);
        double p_wi = p_h / (4 * dot(*wi, h));
        *pdf = p_wi;
        // *wi = cosineHemisphereSampler.get_sample(pdf);
        Vector3D result = MicrofacetBSDF::f(wo, *wi);
        return result;
    }

    void MicrofacetBSDF::render_debugger_node()
    {
        if (ImGui::TreeNode(this, "Micofacet BSDF"))
        {
            DragDouble3("eta", &eta[0], 0.005);
            DragDouble3("K", &k[0], 0.005);
            DragDouble("alpha", &alpha, 0.005);
            ImGui::TreePop();
        }
    }

// Refraction BSDF //

    Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
        return Vector3D();
    }

    Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
        // TODO Project 3-2: Part 1
        // Implement RefractionBSDF
        bool refracted = refract(wo, wi, ior);
        if (!refracted) {
            return Vector3D();
        }
        *pdf = 1.0;
        double eta = wo.z > 0 ? 1.0 / ior : ior / 1.0;
        return transmittance / abs_cos_theta(*wi) / pow(eta, 2);
    }

    void RefractionBSDF::render_debugger_node()
    {
        if (ImGui::TreeNode(this, "Refraction BSDF"))
        {
            DragDouble3("Transmittance", &transmittance[0], 0.005);
            DragDouble("ior", &ior, 0.005);
            ImGui::TreePop();
        }
    }

// Glass BSDF //

    Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
        return Vector3D();
    }

    Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

        // TODO Project 3-2: Part 1
        // Compute Fresnel coefficient and either reflect or refract based on it.

        // compute Fresnel coefficient and use it as the probability of reflection
        // - Fundamentals of Computer Graphics page 305
        // 
        bool refracted = refract(wo, wi, ior);
        if (!refracted) {
            reflect(wo, wi);
            *pdf = 1.0;
            return reflectance / abs_cos_theta(*wi);
        }
        double R_0 = pow((1 - ior) / (1 + ior), 2);
        double R = R_0 + (1 - R_0) * (pow((1 - abs(wo.z)), 5));
        double c = ((double) rand()) / RAND_MAX;
        if (c < R) {
            reflect(wo, wi);
            *pdf = R;
            return R * reflectance / abs_cos_theta(*wi);
        }
        else {
            refract(wo, wi, ior);
            *pdf = 1 - R;
            double eta = wo.z > 0 ? 1.0 / ior : ior / 1.0;
            return (1 - R) * transmittance / abs_cos_theta(*wi) / pow(eta, 2);
        }
        return Vector3D();

    }

    void GlassBSDF::render_debugger_node()
    {
        if (ImGui::TreeNode(this, "Refraction BSDF"))
        {
            DragDouble3("Reflectance", &reflectance[0], 0.005);
            DragDouble3("Transmittance", &transmittance[0], 0.005);
            DragDouble("ior", &ior, 0.005);
            ImGui::TreePop();
        }
    }

    void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

        // TODO Project 3-2: Part 1
        // Implement reflection of wo about normal (0,0,1) and store result in wi.
        wi->x = -wo.x;
        wi->y = -wo.y;
        wi->z = wo.z;
        //*wi = Vector3D(-wo.x, -wo.y, wo.z);

    }

    bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

        // TODO Project 3-2: Part 1
        // Use Snell's Law to refract wo surface and store result ray in wi.
        // Return false if refraction does not occur due to total internal reflection
        // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
        // ray entering the surface through vacuum.


        // z > 0 -> entering, z < 0 -> exiting

        // Snell
        double eta;
        int sign = 1;
        if (wo.z > 0) {
            eta = 1.0 / ior;
        }
        else {
            eta = ior / 1.0;
            sign = -1;
        }

        double tmp = 1 - pow(eta, 2) * (1 - pow(wo.z, 2));
        if (tmp < 0) {
            return false;
        }
        double cos_theta_prime = sqrt(tmp);
        
        wi->x = -eta * wo.x;
        wi->y = -eta * wo.y;
        wi->z = -sign * cos_theta_prime;
        return true;

    }

} // namespace CGL