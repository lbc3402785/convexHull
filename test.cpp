#include "test.h"
#include <opencv2/opencv.hpp>
#include "Dlib.h"
#include "convexhull2d.h"
#include "convexhull3d.h"
#include <fstream>
Test::Test()
{

}

void Test::testConvexHull()
{
    using namespace std;
    using namespace cv;

    cv::Mat img=cv::imread("data//frame_0004.png");
    DlibInit("data//shape_predictor_68_face_landmarks.dat");
    vector<Rect>rectangles;
    vector<vector<Point2f>> keypoints;
    if(DlibFace(img,rectangles,keypoints)){
        ConvexHull2D<float,Point2f> convexHull2D(keypoints[0],true);
        std::vector<size_t> hullIndexes=convexHull2D.convexHull();
        for(size_t k=0;k<keypoints[0].size();k++){
            cv::circle(img,keypoints[0][k],4,cv::Scalar(255,0,0),cv::FILLED,CV_AA);
        }
        for(size_t k=0;k<hullIndexes.size();k++){
            size_t i=hullIndexes[k];
            size_t j=hullIndexes[(k+1)%hullIndexes.size()];

            cv::line(img,keypoints[0][i],keypoints[0][j],cv::Scalar(0,0,255),1,CV_AA);
        }
        cv::imshow("img",img);
        cv::imwrite("result\\test.png",img);
        cv::waitKey(0);
    }
}

void Test::testConvexHull3D()
{
    Point3<float> p0(0,0,0,0);
    Point3<float> p1(1,0,0,1);
    Point3<float> p2(0,1,0,2);
    Point3<float> p3(0,0,1,3);
    Point3<float> p4(0.2,0.2,0.2,4);
    Point3<float> p5(0.3,0.2,0.4,5);
    Point3<float> p6(1,1,1,6);
    Point3<float> p7(1,1,0,7);
    Point3<float> p8(1,0,1,8);
    Point3<float> p9(0,1,1,9);
    std::vector<Point3<float>> points;
    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
    points.push_back(p8);
    points.push_back(p9);
    ConvexHull3D<float> convexHull3D(points);
    std::vector<std::array<size_t,3>> faces=convexHull3D.convexHull();
    std::ofstream out("result\\test.obj");
    for(int k=0;k<points.size();k++){
        out<<"v "<<points[k].x<<" "<<points[k].y<<" "<<points[k].z<<std::endl;
    }
    for(int k=0;k<faces.size();k++){
        out<<"f "<<faces[k][0]+1<<" "<<faces[k][1]+1<<" "<<faces[k][2]+1<<std::endl;
    }
}

void Test::testRemove()
{
    std::vector<int> v;
    v.push_back(2);
    v.push_back(3);
    v.push_back(1);
    v.push_back(2);
    v.push_back(3);
    v.push_back(4);
    std::vector<int>::iterator ret = std::remove(v.begin(), v.end(), 2) ;
    ret = std::remove(v.begin(), ret, 3) ;
    for(int k=0;k<v.size();k++){
        std::cout<<v[k]<<std::endl;
    }
    std::cout<<"----"<<std::endl;
    v.erase(ret,v.end());
    std::vector<int>(v).swap(v);
    for(int k=0;k<v.size();k++){
        std::cout<<v[k]<<std::endl;
    }
}
