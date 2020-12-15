#include "material.h"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>

void Material::loadTextureFromFile(const std::string& fileName)
{
    if (fileName.size()==0)
        std::cerr << "Material error : no texture file name provided" << std::endl;
    else
        m_texture = new Bitmap(fileName);
}

Diffuse::Diffuse(const PropertyList &propList)
{
    m_diffuseColor = propList.getColor("diffuse",Color3f(0.2));

    std::string texturePath = propList.getString("texture","");
    if(texturePath.size()>0){
        filesystem::path filepath = getFileResolver()->resolve(texturePath);
        loadTextureFromFile(filepath.str());
        setTextureScale(propList.getFloat("scale",1));
        setTextureMode(TextureMode(propList.getInteger("mode",0)));
    }
}

Color3f Diffuse::diffuseColor(const Vector2f& uv) const
{ 
    if(!texture())
        return m_diffuseColor;

    // We access the color in the texture bitmap using the uv coordinates with cyclic repetition (hence the fmod)
    auto u = std::fmod(uv.x()/textureScaleU(), 1.f);
    auto v = std::fmod(uv.y()/textureScaleV(), 1.f);

    // If the uv coordinates are negative, we bring them back between 0 and 1
    u = u < 0 ? u + 1 : u;
    v = v < 0 ? v + 1 : v;

    auto u_index = std::clamp(static_cast<Eigen::Index>(u*texture()->rows()), 0l, texture()->rows() - 1);
    auto v_index = std::clamp(static_cast<Eigen::Index>(v*texture()->cols()), 0l, texture()->cols() - 1);

    // We either return the texture color by itself or mixed with the diffuse color, according to textureMode()
    return textureMode() == MODULATE ? m_diffuseColor * (*texture())(u_index,v_index)
                                     : (*texture())(u_index,v_index);
}

REGISTER_CLASS(Diffuse, "diffuse")
