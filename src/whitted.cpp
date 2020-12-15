#include <algorithm>
#include "integrator.h"
#include "scene.h"

class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList &props) {
        m_recursion = props.getInteger("maxRecursion", 4);
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {
        // If the ray was reflected too much, we stop the recursion and return black color
        if (ray.recursionLevel <= 0) {
          return {};
        }

        Hit hit;
        scene->intersect(ray, hit);
        // If an intersection was found, return the color of the object
        if (hit.foundIntersection()) {

            const Material &material = *(hit.shape()->material());

            Vector3f view_dir = -ray.direction;
            Point3f intersect_point = ray.at(hit.t());
            Vector3f normal = hit.normal();
            // We start the loop with black color
            Color3f res_color{};
            // We set the intersection point slightly further from the object so the reflected ray will not intersect with the object itself at its (the reflected ray's) origin
            Point3f ray_o = intersect_point + normal*Epsilon;
            
            for (auto light : scene->lightList()) {
                
                // We need to check if light directly hits the point:
                float light_dist;
                // Vector from intersect point + Epsilon to the light
                Ray shadow_ray { ray_o, light->direction(ray_o, &light_dist) };
                Hit shadow_hit;
                scene->intersect(shadow_ray, shadow_hit);


                // If there is no object between the light and the intersection point, then the light hits our point. We here use an inclusive inequality to consider the case where the intersection happens far enough so that shadow_hit.t() and light_dist are both infinite
                if (shadow_hit.t() >= light_dist) {
                    Vector3f light_dir = light->direction(intersect_point);

                    // Calling brdf method without considering last uv parameter used later for textures
                    Color3f rho = material.brdf(view_dir, light_dir, normal, hit.TextCoord());

                    // The light only hits surfaces on the light's side
                    float intersect_n = std::max(0.f,light_dir.dot(normal));
                
                    // We add the light's contribution to the resulting color
                    res_color += rho*intersect_n*light->intensity(intersect_point);
                }
            }
            // Compute the reflected ray, decrementing the recursive counter

            Vector3f reflecting_dir = 2*normal*view_dir.dot(normal) - view_dir;
            Ray reflected_ray { ray_o, reflecting_dir };
            reflected_ray.recursionLevel = ray.recursionLevel-1;

            return res_color + Li(scene, reflected_ray) * material.reflectivity();
        }
        // To get the same rendering as the example, we mustn't consider the background if we are a reflected ray
        
        if (ray.recursionLevel < maxRecursion()) {
            return Color3f(0.f);
        }
        
        return scene->backgroundColor();

    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }

};

REGISTER_CLASS(WhittedIntegrator, "whitted")
