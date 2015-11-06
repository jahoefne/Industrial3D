//
// Created by Jan Moritz Hoefner on 23/10/15.
//

#include "PointCloud.h"


void PointCloud::print() {
    std::cout << "Read " << this->points.size();
    for (unsigned long i = 0; i < this->points.size(); i++) {
        Point3D *p = this->points.at(i);
        std::cout << "(" << p->x << ", " << p->y << ", " << p->z << ")\n";
    }
};


/**
 * Initializes the points from a .xyz ascii file.
 * Returns 0 if successful and != 0 if an error occured
 * Also calculates the center
 */
int PointCloud::loadPointsFromFile(std::string fileName) {
    ifstream pointFile(fileName, ifstream::in);
    if (pointFile.is_open()) {
        double x, y, z;
        int count = 0;
        while (pointFile >> x && pointFile >> y && pointFile >> z) {

            // Read point from file
            Point3D *point = new Point3D(x, y, z);
            points.push_back(point);

            // consider point for center calculation
            this->center.translate(point);
            count++;

            // reconsider min/max points denoting bounding box
            this->boundingBoxMin.x = (point->x >= this->boundingBoxMin.x) ? this->boundingBoxMin.x : point->x;
            this->boundingBoxMin.y = (point->y >= this->boundingBoxMin.y) ? this->boundingBoxMin.y : point->y;
            this->boundingBoxMax.y = (point->y <= this->boundingBoxMax.y) ? this->boundingBoxMax.y : point->y;
            this->boundingBoxMax.x = (point->x <= this->boundingBoxMax.x) ? this->boundingBoxMax.x : point->x;
        }

        /** Calculate actual center of point cloud*/
        this->center.x /= count;
        this->center.y /= count;
        this->center.z /= count;

        this->sceneRadius = this->center.distanceTo(this->boundingBoxMax);

        cout << "Read file " << fileName << " it contained " <<  this->points.size() << "points.\n";
        pointFile.close();
        return 0;
    } else {
        std::cout << "Can't read file!" << strerror(errno);;
    }
    return -1;
};
