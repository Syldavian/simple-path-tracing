#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <vecmath.h>
#include "object3d.hpp"

// transforms a 3D point using a matrix, returning a 3D point
static Vector3f transformPoint(const Matrix4f &mat, const Vector3f &point) {
    return (mat * Vector4f(point, 1)).xyz();
}

// transform a 3D directino using a matrix, returning a direction
static Vector3f transformDirection(const Matrix4f &mat, const Vector3f &dir) {
    return (mat * Vector4f(dir, 0)).xyz();
}

// implement this class so that the intersect function first transforms the ray
class Transform : public Object3D {
public:
    Transform() {}

    Transform(const Matrix4f &m, Object3D *obj, bool tex=false) : 
    o(obj),useTexture(tex) {
        transform = m.inverse();
    }

    ~Transform() {
    }

    virtual bool intersect(const Ray &r, Hit &h, float tmin) {
        Vector3f trSource = transformPoint(transform, r.getOrigin());
        Vector3f trDirection = transformDirection(transform, r.getDirection());
        float l=trDirection.length();
        trDirection.normalize();
        Ray tr(trSource, trDirection);
        Material dummy(Vector3f::ZERO);
        Hit h_tr(h.getT()*l,&dummy,Vector3f(0));
        bool inter = o->intersect(tr, h_tr, tmin*l);
        if (inter) {
            if(useTexture)
                h.set(h_tr.getT()/l, h_tr.getMaterial(), transformDirection(transform.transposed(),h_tr.getNormal()).normalized(),h_tr.color);
            else
                h.set(h_tr.getT()/l, h_tr.getMaterial(), transformDirection(transform.transposed(),h_tr.getNormal()).normalized());
        }
        return inter;
    }

protected:
    Object3D *o; //un-transformed object
    Matrix4f transform;
    bool useTexture;
};

#endif //TRANSFORM_H
