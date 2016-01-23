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

/**
 * KDNode represents a node in a KDTree
 */
class KDNode {
public:
    double median; /**< The median value which splits the space in the current dimension */
    KDNode *left = nullptr; /**< The left child of the node */
    KDNode *right = nullptr; /**< The right child of the node */
    Point3D *ptrFirstPoint; /**< The pointer to the first point */
    Point3D *ptrLastPoint; /**< The pointer to the last point */
};


/**
 * K3DTree defines a three dimensional KDTree using KDNode, and implements the typical KDTree algorithms -
 * closest neighbour, radius query, range query, ...
 */
class K3DTree {
public:
    KDNode *root; /**< The root node of the KDTree */

    /**
     * Constructor - Initialized the tree for a given Point vector
     */
    K3DTree(std::vector<Point3D> *points) {
        printf("Building KD Tree with %ld Points\n", points->size());
        this->root = this->recConstruct(&points->at(0), &points->at(points->size() - 1), 0);
    }

    /**
     * Recursive construction method - recursively builds up the KDTree
     */
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


    /**
     * Returns the closest neighbour in the KDTree of any given point.
     * @param tgt - the target Point3D
     * @return the closest point in the KDTree
     */
    Point3D *closestNeighbour(Point3D *tgt) {
        Point3D* close = recClosestNeighbour(tgt, root, X_AXIS);
        double distance = close->distance3d(tgt);

        std::vector<Point3D*> *evenCloser = new std::vector<Point3D*>();
        findRadiusNeighborsRec(root, tgt, distance, evenCloser, 0);

        if(evenCloser->size()!=0) {
            std::sort(evenCloser->begin(), evenCloser->end(),
                      [&](Point3D *a, Point3D *b) -> bool { return tgt->distance3d(a) < tgt->distance3d(b); });

            Point3D *closest = evenCloser->front();
            delete evenCloser;
            return closest;
        }else{
            return close;
        }
    }

    /**
     * Find all neighbours of a point which are in the range of a radius
     * @param queryPoint the point which is being queried
     * @param radius the maximum distance of a returned point to queryPoint
     */
    std::vector<Point3D*>* findRadiusNeighbors(Point3D* queryPoint, double radius){
        std::vector<Point3D*> *neighbors = new std::vector<Point3D*>();
        findRadiusNeighborsRec(root, queryPoint, radius, neighbors, 0);
        return neighbors;
    }

    /**
     * Finds all points which are in a diven range
     * @param min the point describing the minimum range
     * @param max the point describing the maximum range
     * @return all points which are within min - max
     */
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
