bool Plane::intersect(const Ray& ray, Hit& hit) const
{
    // Point on plane and normal
    auto plane_p = this->m_position;
    auto plane_n = this->m_normal;

    // Direction and origin of ray
    auto ray_d = ray.direction;
    auto ray_o = ray.origin;

    // If denom is equals to 0, then the solution is infinite, and the ray is parallel to the plane
    auto denom = ray_d.dot(plane_n);

    if (denom >= Epsilon) {
        //auto D = plane_n.dot(plane_p);
        //auto solution = (D - plane_n.dot(ray_o))/(denom);
        auto solution = (plane_p - ray_o).dot(plane_n)/denom;
        // If solution is positive, the intersection occurs in front of the camera
        if (solution > 0) {
            hit.setT(solution);
            hit.setNormal(plane_n);
            return true;
        }
    }

    else return false;
}