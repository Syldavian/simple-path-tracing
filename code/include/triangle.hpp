#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include "plane.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0]=a;
		vertices[1]=b;
		vertices[2]=c;
		normal=Vector3f::cross(b-a,c-a).normalized();
		d=Vector3f::dot(normal,a);
		material=m;
	}

	bool intersect(const Ray& ray, Hit& hit, float tmin) override {
		Hit h=hit;
		if(!Plane(normal,Vector3f::dot(normal,vertices[0]),material).intersect(ray,h,tmin))
			return false;
		Vector3f p=ray.pointAtParameter(h.getT());
		Vector3f n1=Vector3f::cross(vertices[0]-p,vertices[1]-p);
		Vector3f n2=Vector3f::cross(vertices[1]-p,vertices[2]-p);
		Vector3f n3=Vector3f::cross(vertices[2]-p,vertices[0]-p);
		if(Vector3f::dot(n1,normal)<0||Vector3f::dot(n2,normal)<0||Vector3f::dot(n3,normal)<0)
			return false;
		hit=h;
		return true;
	}
	
	Vector3f normal;
	Vector3f vertices[3];
	Material* material;
protected:
	float d;
};

#endif //TRIANGLE_H
