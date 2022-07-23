#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include "Vector2f.h"
#include "Vector3f.h"

struct Node{
    int leftchild,rightchild;
    Vector3f minimum,maximum;
};


class KDTree{
public:
    std::vector<Node> t;
    int root;
    bool checkIntersect(int x, const Ray &r, float tmin, Material *material);
    Node& operator [] (int i);
};


#endif