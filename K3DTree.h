//
// Created by moe on 06/11/15.
//

#ifndef INDUSTRIAL3D_K3DTREE_H
#define INDUSTRIAL3D_K3DTREE_H

#include <algorithm>
#include "PointCloud.h"

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

class KDNode {
public:
    double median;
    KDNode *left = nullptr;
    KDNode *right = nullptr;
    Point3D *ptrFirstPoint;
    Point3D *ptrLastPoint;
};

class K3DTree {
public:
    // std::vector<Point3D> points_range;
    KDNode *root;

    /** Constructor */
    K3DTree(std::vector<Point3D> *points) {
        printf("Building KD Tree with %ld Points\n", points->size());
        this->root = this->recConstruct(&points->at(0), &points->at(points->size() - 1), 0);
    }

    /** Recursive construction method */
    KDNode *recConstruct(Point3D *begin, Point3D *end, int depth) {
        unsigned long numPoints = (end - begin);
        unsigned long medianPosition = numPoints / 2;
        double medianValue = 0;

        switch (depth % 3) {
            case X_AXIS:
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.x < b.x; });
                medianValue = (begin + medianPosition)->x;
                break;
            case Y_AXIS:
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.y <  b.y; });
                medianValue = (begin + medianPosition)->y;
                break;
            case Z_AXIS:
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.z < b.z; });
                medianValue = (begin + medianPosition)->z;
                break;
            default:
                printf("Can not happen!");
                break;
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


    Point3D *closestNeighbour(Point3D *tgt) {
        Point3D* close = recClosestNeighbour(tgt, root, X_AXIS);
        printf("\nClose point is %lf %lf %lf", close->x, close->y, close->z);
        close->r=200; close->g=200;

        double distance = close->distance3d(tgt);
        printf("\n\tthe distance from our query point is %lf ",distance);

        std::vector<Point3D*>* evenCloser = findRadiusNeighbors(tgt, distance);
        printf("\nRange query on the distance of our close point. There are %lu points even closer" , evenCloser->size());

        std::sort(evenCloser->begin(), evenCloser->end(),
                  [&](Point3D *a, Point3D *b) -> bool { return tgt->distance3d(a) < tgt->distance3d(b); });

        Point3D* closest = evenCloser->front();
        printf("\n\tthe Point with the closest distance from our query point is %lf away",tgt->distance3d(closest));
        return closest;
    }

    std::vector<Point3D*>* findRadiusNeighbors(Point3D* queryPoint, double radius){
        std::vector<Point3D*> *neighbors = new std::vector<Point3D*>();
        findRadiusNeighborsRec(root, queryPoint, radius, neighbors, 0);
        return neighbors;
    }

    std::vector<Point3D*>* findRange(Point3D* min, Point3D* max){
        std::vector<Point3D*> *neighbors = new std::vector<Point3D*>();
        findRangeRec(root, min, max, neighbors, 0);
        return neighbors;
    }

private:

    void findRadiusNeighborsRec(KDNode* node, Point3D* queryPoint, double radius, std::vector<Point3D*>* neighbors, unsigned int depth)
    {
        if (node->left == node->right) // if this is true the node is a leaf
        {
            Point3D* pt = node->ptrFirstPoint;

            // We don't use the expensive sqrt-function but instead compare the squared values
            const double sqDistance = pt->sqDistance3d(queryPoint); // squared distance
            if (sqDistance > (radius*radius)) return; // point outside the sphere
            neighbors->push_back(node->ptrFirstPoint);
        }
        else
        {
            const unsigned int DIM = depth % 3;
            double v_min, v_max;
            if (DIM == 0){
                v_min = queryPoint->x - radius;
                v_max = queryPoint->x + radius;
            }
            else if (DIM == 1){
                v_min = queryPoint->y - radius;
                v_max = queryPoint->y + radius;
            }
            else{
                v_min = queryPoint->z - radius;
                v_max = queryPoint->z + radius;
            }

            if (v_min <= node->median) findRadiusNeighborsRec(node->left , queryPoint, radius, neighbors, depth + 1);
            if (v_max  > node->median) findRadiusNeighborsRec(node->right, queryPoint, radius, neighbors, depth + 1);
        }
    }

    void findRangeRec(const KDNode* node, Point3D* min, Point3D* max, std::vector<Point3D*>* neighbors, unsigned int depth)
    {
        if (node->left == node->right) //if this is true the node is a leaf
        {
            const Point3D& pt=*node->ptrFirstPoint;
            //a final check still must be done
            if (pt.x < min->x || pt.x > max->x) return;
            if (pt.y < min->y || pt.y > max->y) return;
            if (pt.z < min->z || pt.z > max->z) return;

            neighbors->push_back(node->ptrFirstPoint);
        }
        else
        {
            const unsigned int DIM=depth % 3;
            double v_min, v_max;
            if (DIM == X_AXIS){
                v_min=min->x;
                v_max=max->x;
            }
            else if(DIM == Y_AXIS){
                v_min=min->y;
                v_max=max->y;
            }
            else{
                v_min=min->z;
                v_max=max->z;
            }

            if (v_min <= node->median) findRangeRec(node->left, min, max, neighbors, depth+1);
            if (v_max  > node->median) findRangeRec(node->right, min, max, neighbors, depth+1);
        }
    }


    Point3D *recClosestNeighbour(Point3D *tgt, KDNode *curr, int dim) {
        double value = 0;
        switch (dim % 3) {
            case X_AXIS:
                value = tgt->x;
                break;
            case Y_AXIS:
                value = tgt->y;
                break;
            case Z_AXIS:
                value = tgt->z;
                break;
            default:
                printf("Should never happen");
                return NULL;
        }
        if (curr->median > value && curr->left != NULL) {
            recClosestNeighbour(tgt, curr->left, dim + 1);
        } else if (curr->right != NULL) {
            return recClosestNeighbour(tgt, curr->right, dim + 1);
        }
        return curr->ptrFirstPoint;
    }
};

#endif //INDUSTRIAL3D_K3DTREE_H
