#ifndef TEST_H
#define TEST_H
#include <string>

class Test
{
public:
    Test();
    static void testConvexHull();
    static void testConvexHull3D();
    static void testConvexHull3DInputObj(std::string objPath);
    static void testSphereConvexHull3D();
    static void testPlaneConvexHull3D();
    static void testDuplicateConvexHull3D();
    static void testRemove();
    static void testList();
};

#endif // TEST_H
