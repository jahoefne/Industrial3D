//
// Created by moe on 06/11/15.
//

#ifndef INDUSTRIAL3D_K3DTREE_H
#define INDUSTRIAL3D_K3DTREE_H

#include "Point3D.h"
#include "PointCloud.h"

class KDNode {
public:
    double median;
    KDNode *left;
    KDNode *right;
    Point3D *ptrFirstPoint;
    Point3D *ptrLastPoint;
};

class K3DTree {
private:
    KDNode *root;

    /** Recursive construction method */
    KDNode *recConstruct(Point3D *begin, Point3D *end, int depth) {
        unsigned long numPoints = (end - begin);
        unsigned long medianPosition = numPoints / 2;
        double medianValue = 0;
        int currentDimension = depth % 3;

        if (currentDimension == 0) {
            std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.x > b.x; });
            medianValue = (begin + medianPosition)->x;
        } else if (currentDimension == 1) {
            std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.y > b.y; });
            medianValue = (begin + medianPosition)->y;
        } else if (currentDimension == 2) {
            std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.z > b.z; });
            medianValue = (begin + medianPosition)->z;
        }

        KDNode *child = new KDNode;
        child->median = medianValue;
        child->ptrFirstPoint = begin;
        child->ptrLastPoint = end;

        if (numPoints > 1) {
            child->left = this->recConstruct(begin, begin + medianPosition, depth + 1);
            child->right = this->recConstruct(begin + medianPosition, end, depth + 1);
        }

        return child;
    }

public:
    /** Constructor */
    K3DTree(PointCloud *cloud) {
        printf("Building KD Tree \n");
        this->root = this->recConstruct(&cloud->points[0], &cloud->points[cloud->points.size()], 0);
    }
};

#endif //INDUSTRIAL3D_K3DTREE_H
