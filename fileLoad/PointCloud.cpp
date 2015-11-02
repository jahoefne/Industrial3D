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
}

/**
 * Initializes the points from a .xyz ascii file.
 * Returns 0 if successful and != 0 if an error occured
 */
int PointCloud::loadPointsFromFile(std::string fileName) {
    ifstream pointFile(fileName, ifstream::in);
    if (pointFile.is_open()) {
        double x, y, z;
        while (pointFile >> x && pointFile >> y && pointFile >> z) {
            points.push_back(new Point3D(x, y, z));
        }
        pointFile.close();
        return 0;
    } else {
        std::cout << "Can't read file!" << strerror(errno);;
    }
    return -1;
};
