#include <cmath>

#include "integrator.h"
#include "scene.h"

class TexCoordsIntegrator : public Integrator {
public:
    TexCoordsIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {

        /** TODO : Find the surface that is visible in the requested direction
                   Return its ambiant color */
        Hit hit;
        scene->intersect(ray, hit);
        // If an intersection was found, return the color of the object
        if ( hit.foundIntersection() ) {
            return {hit.TextCoord().x(), 0.5*(1 - std::cos(2*EIGEN_PI*hit.TextCoord().y())), 0.5*(1 - std::sin(2*EIGEN_PI*hit.TextCoord().y()))};
        }
        // Otherwise, return background
        return scene->backgroundColor();
    }

    std::string toString() const {
        return "TexCoordsIntegrator[]";
    }
};

REGISTER_CLASS(TexCoordsIntegrator, "texcoords")
