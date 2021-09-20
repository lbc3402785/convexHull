//#include <QCoreApplication>
#include "test.h"
#include <iostream>
int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv);
    Test::testConvexHull3D();
    return 0;
    //return a.exec();
}
