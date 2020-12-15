#include "sphere.h"
#include <cmath>
#include <iostream>

Sphere::Sphere(float radius)
    : m_radius(radius)
{
}

Sphere::Sphere(const PropertyList &propList)
{
    m_radius = propList.getFloat("radius",1.f);
    m_center = propList.getPoint("center",Point3f(0,0,0));
}

Sphere::~Sphere()
{
}

bool Sphere::intersect(const Ray& ray, Hit& hit) const
{   
    /// TODO: compute ray-sphere intersection
    // Members of the quadratic formula, with 'a' equal to 1 
    auto b = 2.*ray.direction.dot(ray.origin - m_center);
    auto c = (ray.origin - m_center).squaredNorm() - m_radius * m_radius;

    // Computing the discriminant of the quadratic formula
    auto discr = b * b - 4. * c;
    // We have at least one intersection if the discriminant is positive
    
    if (discr >= 0) {
        auto discr_sqrt = std::sqrt(discr);
        // Because discriminant is >=0, this is the smallest solution
        auto solution = (-b - discr_sqrt)*0.5;
        
        // If it is negative, then we compute the greatest solution
        if (solution < 0)
            solution = (-b + discr_sqrt)*0.5;
        // We verify the greatest solution is still in front of the camera
        if (solution < 0)
            return false;

        hit.setT(solution);
        auto intersection_p = ray.at(solution);
        hit.setNormal((intersection_p - m_center).normalized());

        // The u coordinate ranges from 0 (South hemisphere) to 1 (North hemisphere)
        float u = 1 - std::acos( hit.normal().z()) / EIGEN_PI;
        // The v coordinate ranges from 0 to 1 following a counter clockwise rotation seen from upward
        float v = 0.5*(1 + std::atan2(hit.normal().x(), hit.normal().y()) / EIGEN_PI);

        hit.setTextCoord(u,v);

        return true;
    }
    
    return false;
}

REGISTER_CLASS(Sphere, "sphere")
