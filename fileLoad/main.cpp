//
// Created by Jan Moritz Hoefner on 23/10/15.
//

#include <iostream>
#include <fstream>
#include <vector>
#include "PointCloud.h"

using namespace std;


int main() {
    fflush(stdin);
    PointCloud cloud = PointCloud();
    cout << "Please enter the filename: ";
    std::string name;
    cin >> name;
    cout << "Reading " << name << "…" ;
    cloud.loadPointsFromFile(name);
    cloud.print();
    return 0;
}