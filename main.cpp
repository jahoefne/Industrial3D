#include "PointCloud.h"
#include "App.h"


int main(int argc, char **argv) {
    PointCloud *initialCloud = new PointCloud();
    initialCloud->loadPointsFromFile("/Users/jahoefne/Desktop/Industrial3D/data/bunny.xyz");

    App* app = new App();
    app->start(argc, argv, initialCloud);
}