#include "mesh.hpp"
#include "plane.hpp"
#include "kdtree.hpp"
#include <algorithm>
#include <cstdlib>

using namespace std;

Node& KDTree::operator [] (int i){
    return t[i];
}

bool KDTree::checkIntersect(int x, const Ray &r, float tmin,Material *material){
    Vector3f a=t[x].minimum,b=t[x].maximum;
    Hit h1,h2;
    float m1=-1e30,m2=1e30;
    if(IntersectBox(Vector3f(1,0,0),a.x(),material).intersect_with_box(r,h1,tmin)
        &&IntersectBox(Vector3f(1,0,0),b.x(),material).intersect_with_box(r,h2,tmin)){
        m1=max(m1,min(h1.getT(),h2.getT()));
        m2=min(m2,max(h1.getT(),h2.getT()));
    }
    if (IntersectBox(Vector3f(0,1,0),a.y(),material).intersect_with_box(r,h1,tmin)
        &&IntersectBox(Vector3f(0,1,0),b.y(),material).intersect_with_box(r,h2,tmin)){
        m1=max(m1,min(h1.getT(),h2.getT()));
        m2=min(m2,max(h1.getT(),h2.getT()));
    }
    if (IntersectBox(Vector3f(0,0,1),a.z(),material).intersect_with_box(r,h1,tmin)
        &&IntersectBox(Vector3f(0,0,1),b.z(),material).intersect_with_box(r,h2,tmin)){
        m1=max(m1,min(h1.getT(),h2.getT()));
        m2=min(m2,max(h1.getT(),h2.getT()));
    }
    if(m1-m2<1e-4)
        return true;
    else
        return false;
}

