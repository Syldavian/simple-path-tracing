#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// Implement functions and add more fields as necessary

class Sphere : public Object3D {
public:
    Sphere() {
        // unit ball at the center
        center=Vector3f::ZERO;
        radius=1;
    }

    Sphere(const Vector3f &center, float radius, Material *material, Image *img=nullptr, bool tex=false) : 
    Object3D(material),center(center),radius(radius),material(material),image(img),useTexture(tex){

    }

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        if (material->getDiffuseColor()[0]>=15)
            return false;
        Vector3f o=r.getOrigin();
        Vector3f l=center-o;
        Vector3f d=r.getDirection();
        d.normalize();
        float s=Vector3f::dot(l,d);
        if(s<1e-3&&l.squaredLength()>(radius*radius))
            return false;
        float mSquared=l.squaredLength()-s*s;
        if(mSquared>(radius*radius))
            return false;
        float q=sqrt(fabs(radius*radius-mSquared));
        float t=0;
        if(l.squaredLength()>(radius*radius))
            t=s-q;
        else
            t=s+q;
        if(t<tmin||t>h.getT()) 
            return false;
        Vector3f normal=(o+t*d-center).normalized();
        Vector3f intersection=o+t*d;
        if(useTexture){
            float up=0.5+(atan2(intersection.z(),intersection.x()))/(2*M_PI);
            float vp=0.5-asin(intersection.y())/M_PI;
            int u=(int)floor(image->Width()*up);
            int v=(int)floor(image->Height()*vp);
            if(u<0) 
                u=0;
            if(v<0) 
                v=0;
            h.set(t,material,normal,image->GetPixel(u,v));
        }
        else
            h.set(t,material,normal);
        return true;
    }

protected:
    Vector3f center;
    float radius;
    Material* material;
    Image *image;
    bool useTexture;
};


#endif
