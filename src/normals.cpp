#include "integrator.h"
#include "scene.h"

class NormalsIntegrator : public Integrator {
public:
    NormalsIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {

        /** TODO : Find the surface that is visible in the requested direction
                   Return its ambiant color */
        Hit hit;
        scene->intersect(ray, hit);
        // If an intersection was found, return the color of the object
        if ( hit.foundIntersection() ) {
            float r = std::abs(hit.normal().x());
            float g = std::abs(hit.normal().y());
            float b = std::abs(hit.normal().z());

            Color3f color{r,g,b};
            return color;
        }
        // Otherwise, return background color
        return scene->backgroundColor();
    }

    std::string toString() const {
        return "NormalsIntegrator[]";
    }
};

REGISTER_CLASS(NormalsIntegrator, "normals")
