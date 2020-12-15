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
    Point3f plane_p = this->m_position;
    Vector3f plane_n = this->m_normal;

    Vector3f ray_d = ray.direction;
    Vector3f ray_o = ray.origin;

    // If denom is equals to 0, then the solution is infinite, and the ray is parallel to the plane
    float denom = ray_d.dot(plane_n);

    if (denom >= 0 && denom < Epsilon)
        return false;
    
    auto D = plane_n.dot(plane_p);

    float solution = (D - plane_n.dot(ray_o))/(denom);

    if (solution > 0) {
        hit.setT(solution);
        hit.setNormal(plane_n);
        return true;
    }
    else return false;
}

REGISTER_CLASS(Plane, "plane")
