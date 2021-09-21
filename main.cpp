//#include <QCoreApplication>
#include "test.h"
#include <iostream>
int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv);
//    Test::testConvexHull3DInputObj("data\\armadillo.obj");
//    Test::testConvexHull3DInputObj("data\\cow.obj");
//    Test::testConvexHull3DInputObj("data\\happy.obj");
//    Test::testConvexHull3DInputObj("data\\stanford-bunny.obj");
//    Test::testConvexHull3DInputObj("data\\teapot.obj");
    Test::testPlaneConvexHull3D();
    Test::testSphereConvexHull3D();
    Test::testDuplicateConvexHull3D();
    return 0;
    //return a.exec();
}
