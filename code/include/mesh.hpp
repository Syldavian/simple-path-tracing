#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "kdtree.hpp"
#include "Vector2f.h"
#include "Vector3f.h"

struct TriangleIndex {
    TriangleIndex() {
        x[0] = 0; x[1] = 0; x[2] = 0;
    }
    int &operator[](const int i) { return x[i]; }
    // By Computer Graphics convention, counterclockwise winding is front face
    int x[3]{};
};
    
class Mesh : public Object3D {

public:
    Mesh(const char *filename, Material *m, float scale, Vector3f offset);

    std::vector<Vector3f> v;
    std::vector<TriangleIndex> t;
    std::vector<Vector3f> n;
    KDTree tree;
    
    bool intersect(const Ray &r, Hit &h, float tmin) override;

    struct Cmp{
        bool operator()(TriangleIndex &a, TriangleIndex &b) const;
        Mesh *parent;
        Cmp() = delete;
        Cmp(Mesh *p):parent(p){}
    };

private:
    // Normal can be used for light estimation
    void computeNormal();
    int buildTree(int l, int r, int d);
    bool query(int x, const Ray &r, Hit &h, float tmin);
};

#endif
