#include "light.h"

class PointLight : public Light
{
public:
    PointLight(const PropertyList &propList)
        : Light(propList.getColor("intensity", Color3f(1.f)))
    {
        m_position = propList.getPoint("position", Point3f::UnitX());
    }

    Vector3f direction(const Point3f& x, float* dist = 0) const
    {
        Vector3f direction = m_position-x;
        if (dist != nullptr) *dist = direction.norm(); 
        return direction.normalized();
    }

    Color3f intensity(const Point3f& x) const
    {   
        float dist;
        direction(x, &dist);
        Color3f color{ m_intensity / (dist*dist + Epsilon) };
        return color;
    }

    std::string toString() const {
        return tfm::format(
            "PointLight[\n"
            "  intensity = %s\n"
            "  position = %s\n"
            "]", m_intensity.toString(),
                 ::toString(m_position));
    }

protected:
    Point3f m_position;
};

REGISTER_CLASS(PointLight, "pointLight")
