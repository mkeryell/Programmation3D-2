#include <cassert>
#include <cmath>
#include "plane.h"

Plane::Plane()
{
}

Plane::Plane(const PropertyList &propList)
{
    m_position = propList.getPoint("position",Point3f(0,0,0));
    m_normal = propList.getVector("normal",Point3f(0,0,1));
}

Plane::~Plane()
{
}

bool Plane::intersect(const Ray& ray, Hit& hit) const
{
    // Point on plane and normal
    auto plane_p = m_position;
    auto plane_n = m_normal;

    // Direction and origin of ray
    auto ray_d = ray.direction;
    auto ray_o = ray.origin;

    assert(std::abs(plane_n.dot(plane_n) - 1) < Epsilon);
    assert(std::abs(ray_d.dot(ray_d) - 1) < Epsilon);
    // If denom is equals to 0, then the solution is infinite, and the ray is parallel to the plane
    
    auto denom = ray_d.dot(plane_n);

    if (std::abs(denom) > Epsilon) {
        auto D = plane_n.dot(plane_p);
        auto solution = (D - plane_n.dot(ray_o))/(denom);
        
        // If solution is positive, the intersection occurs in front of the camera
        if (solution >= 0) {
            hit.setT(solution);
            hit.setNormal(plane_n);

            // We define a vector space in our plane using orthogonal vectors
            auto u = m_normal.cross(Vector3f {0,0,1} );
            if (u == Vector3f{0,0,0}) {
                // The normal was colinear to {1,0,0}. Pick another "random" vector 
                u = m_normal.cross(Vector3f {0,1,0} );
            }
            u = u.normalized();
            auto v = m_normal.cross(u).normalized();

            auto colinear_vector = ray.at(solution) - plane_p;
            auto p_u = u.dot(colinear_vector);
            auto p_v = v.dot(colinear_vector);

            hit.setTextCoord(p_u, p_v);
            //hit.setNormal(plane_n.dot(-ray_d)*plane_n.normalized());
            return true;
        }
    }

    return false;
}

REGISTER_CLASS(Plane, "plane")
