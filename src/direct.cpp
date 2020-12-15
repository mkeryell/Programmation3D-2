#include <algorithm>
#include "integrator.h"
#include "scene.h"

class DirectIntegrator : public Integrator {
public:
    DirectIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {

        /** TODO : Find the surface that is visible in the requested direction
                   Return its ambiant color */
        Hit hit;
        scene->intersect(ray, hit);
        // If an intersection was found, return the color of the object
        if ( hit.foundIntersection() ) {

            const Material &material = *(hit.shape()->material());
            
            Vector3f viewDir = -ray.direction;
            Point3f intersect_point = ray.at(hit.t());
            Vector3f normal = hit.normal();
            Color3f res_color{0.f};
            Point3f ray_o = intersect_point + normal*Epsilon;
            
            const LightList &lightlist = scene->lightList();
            for (auto light : lightlist) {
                
                // We need to check if light directly hits the point:
                float dist;
                // Vector from intersect point + Epsilon to the light
                Ray shadow_ray(ray_o, light->direction(ray_o, &dist));
                Hit shadow_hit;
                scene->intersect(shadow_ray, shadow_hit);

                if (shadow_hit.t() < dist - Epsilon) {
                }
                else {
                    Vector3f lightDir = light->direction(intersect_point);

                    // Calling brdf method without considering last uv parameter used later for textures
                    Color3f rho = material.brdf(viewDir, lightDir, normal, {});

                    float intersect_n = std::max(0.f,lightDir.dot(normal)); // Verifier si Li ou -Li

                    Color3f intensity = light->intensity(intersect_point);
                
                    res_color += (rho*intersect_n)*intensity;
                }
            }
            return res_color;
        }
        // Otherwise, return background
        return scene->backgroundColor();
    }

    std::string toString() const {
        return "DirectIntegrator[]";
    }
};

REGISTER_CLASS(DirectIntegrator, "direct")
