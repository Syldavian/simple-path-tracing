#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>
#include <algorithm>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>

// Implement Shade function that computes Phong introduced in class.
class Material {
public:

    explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, int refl=0, float s = 0) :
            diffuseColor(d_color), specularColor(s_color), refltype(refl), shininess(s) {

    }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    virtual Vector3f getSpecularColor() const {
        return specularColor;
    }
    
    virtual int getReflectionType() const {
        return refltype;
    }

    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f N=hit.getNormal().normalized();
        Vector3f L=dirToLight.normalized();
        Vector3f V=-ray.getDirection().normalized();
        Vector3f R=(2*(Vector3f::dot(L,N))*N-L).normalized();
        Vector3f diffuse=std::max((float)0,Vector3f::dot(L,N))*diffuseColor;
        Vector3f specular=pow(std::max((float)0,Vector3f::dot(V,R)),shininess)*specularColor;
        Vector3f shaded=lightColor*(diffuse+specular);
        return shaded;
    }

protected:
    Vector3f diffuseColor;
    Vector3f specularColor;
    float shininess;
    int refltype; //reflection type (DIFFuse, SPECular, REFRactive)
};


#endif // MATERIAL_H
