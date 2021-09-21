#include "test.h"
#include <opencv2/opencv.hpp>
#include "Dlib.h"
#include "convexhull2d.h"
#include "convexhull3d.h"
#include <fstream>
#include <list>
#define _USE_MATH_DEFINES
#include <math.h>
#include "tiny_obj_loader.h"
#include "objoperator.h"
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
    //points.push_back(p3);
    //points.push_back(p4);
    //points.push_back(p5);
    //points.push_back(p6);
    //points.push_back(p7);
    //points.push_back(p8);
    //points.push_back(p9);
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
//************************************
// Method:    string_replace
// FullName:  string_replace
// Access:    public
// Returns:   void
// Qualifier: 把字符串的strsrc替换成strdst
// Parameter: std::string & strBig
// Parameter: const std::string & strsrc
// Parameter: const std::string & strdst
//————————————————
//版权声明：本文为CSDN博主「jota」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
//原文链接：https://blog.csdn.net/shaoyiju/article/details/78377132
//************************************
void string_replace( std::string &strBig, const std::string &strsrc, const std::string &strdst)
{
    std::string::size_type pos = 0;
    std::string::size_type srclen = strsrc.size();
    std::string::size_type dstlen = strdst.size();

    while( (pos=strBig.find(strsrc, pos)) != std::string::npos )
    {
        strBig.replace( pos, srclen, strdst );
        pos += dstlen;
    }
}

//************************************
// Method:    GetFileOrURLShortName
// FullName:  GetFileOrURLShortName
// Access:    public
// Returns:   std::string
// Qualifier: 获取路径或URL的文件名（包括后缀，如 C:\Test\abc.xyz --> abc.xyz）
// Parameter: std::string strFullName
//————————————————
//版权声明：本文为CSDN博主「jota」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
//原文链接：https://blog.csdn.net/shaoyiju/article/details/78377132
//************************************
std::string GetPathOrURLShortName(std::string strFullName)
{
    if (strFullName.empty())
    {
        return "";
    }

    string_replace(strFullName, "/", "\\");

    std::string::size_type iPos = strFullName.find_last_of('\\') + 1;

    return strFullName.substr(iPos, strFullName.length() - iPos);
}
std::string GetPathOrURLDir(std::string strFullName)
{
    if (strFullName.empty())
    {
        return "";
    }

    string_replace(strFullName, "/", "\\");

    std::string::size_type iPos = strFullName.find_last_of('\\') + 1;

    return strFullName.substr(0,iPos);
}

void Test::testConvexHull3DInputObj(std::string objPath)
{
    tinyobj::attrib_t attribDst;
    std::vector<tinyobj::shape_t> shapesDst;
    std::vector<tinyobj::material_t> materialsDst;

    std::string warn;
    std::string err;

    bool ret = tinyobj::LoadObj(&attribDst, &shapesDst, &materialsDst, &warn, &err, objPath.c_str());

    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        exit(1);
    }
    std::vector<size_t> originFaces;
    size_t index_offset=0;
    for(size_t i=0;i<shapesDst[0].mesh.num_face_vertices.size();i++){
        size_t faceSize=shapesDst[0].mesh.num_face_vertices[i];
        if(faceSize!=3){
            std::cerr<<"error!"<<std::endl;
        }
        size_t idx0 = shapesDst[0].mesh.indices[index_offset+0].vertex_index;
        size_t idx1 = shapesDst[0].mesh.indices[index_offset+1].vertex_index;
        size_t idx2 = shapesDst[0].mesh.indices[index_offset+2].vertex_index;
        originFaces.emplace_back(idx0);
        originFaces.emplace_back(idx1);
        originFaces.emplace_back(idx2);
        index_offset+=3;
    }
    std::string name=GetPathOrURLShortName(objPath);
    std::string originSaveName="result\\"+name;




    ConvexHull3D<float> convexHull3D(attribDst.vertices);
    std::vector<std::array<size_t,3>> faces=convexHull3D.convexHull();
    std::string convexHullSaveName="result\\convexHull"+name;

    float ratio=1.0f/convexHull3D.range;
    std::for_each(attribDst.vertices.begin(),attribDst.vertices.end(),[&](float& v){
        v*=ratio;
    });
    ObjOperator<float>::saveObjOutput(attribDst.vertices,originFaces,originSaveName);
    ObjOperator<float>::saveObjOutput(attribDst.vertices,faces,convexHullSaveName);
}

void Test::testSphereConvexHull3D()
{
    int hNum=20;
    int vNum=20;
    float r=1.0f;
    std::vector<float> vertices;
    for(int i=0;i<=vNum;i++){
        if(i==0){
            vertices.push_back(0);
            vertices.push_back(0);
            vertices.push_back(r);
            continue;
        }else if(i==vNum){
            vertices.push_back(0);
            vertices.push_back(0);
            vertices.push_back(-r);
            continue;
        }
        for(int j=0;j<hNum;j++){
            float x=r*sin(M_PI*(float)i/vNum)*cos(2*M_PI*(float)j/hNum);
            float y=r*sin(M_PI*(float)i/vNum)*sin(2*M_PI*(float)j/hNum);
            float z=r*cos(M_PI*(float)i/vNum);
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
        }
    }

    ConvexHull3D<float> convexHull3D(vertices);
    std::vector<std::array<size_t,3>> faces=convexHull3D.convexHull();
    ObjOperator<float>::saveObjOutput(vertices,std::vector<size_t>(),"result\\Sphere.obj");
    ObjOperator<float>::saveObjOutput(vertices,faces,"result\\convexHullSphere.obj");
}

void Test::testPlaneConvexHull3D()
{
    std::array<float,3> p0={2,0,0};
    std::array<float,3> p1={1,5,0};
    std::array<float,3> p2={0,1,0};
    std::array<float,3> p3={2,3,0};
    std::array<float,3> p4={0,9,0};
    std::array<float,3> p5={6,0,0};
    std::array<float,3> p6={-7,0,0};
    std::array<float,3> p7={1,6,0};
    std::vector<std::array<float,3>> vertices;
    vertices.push_back(p0);
    vertices.push_back(p1);
    vertices.push_back(p2);
    vertices.push_back(p3);
    vertices.push_back(p4);
    vertices.push_back(p5);
    vertices.push_back(p6);
    vertices.push_back(p7);
    ConvexHull3D<float> convexHull3D(vertices);
    std::vector<std::array<size_t,3>> faces=convexHull3D.convexHull();
    ObjOperator<float>::saveObjOutput(vertices,std::vector<size_t>(),"result\\Plane.obj");
    ObjOperator<float>::saveObjOutput(vertices,faces,"result\\convexHullPlane.obj");
}

void Test::testDuplicateConvexHull3D()
{
    std::array<float,3> p0={0,0,0};
    std::array<float,3> p1={1,0,0};
    std::array<float,3> p2={0,1,0};
    std::array<float,3> p3={0,0,1};
    std::array<float,3> p4={0,0,1};
    std::array<float,3> p5={1,0,0};
    std::array<float,3> p6={0,1,0};
    std::array<float,3> p7={1,1,0};
    std::vector<std::array<float,3>> vertices;
    vertices.push_back(p0);
    vertices.push_back(p1);
    vertices.push_back(p2);
    vertices.push_back(p3);
    vertices.push_back(p4);
    vertices.push_back(p5);
    vertices.push_back(p6);
    vertices.push_back(p7);
    ConvexHull3D<float> convexHull3D(vertices);
    std::vector<std::array<size_t,3>> faces=convexHull3D.convexHull();
    ObjOperator<float>::saveObjOutput(vertices,std::vector<size_t>(),"result\\Duplicate.obj");
    ObjOperator<float>::saveObjOutput(vertices,faces,"result\\convexHullDuplicate.obj");

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
std::list<int>::iterator findIt(int t,std::list<int>& ll){
    for(std::list<int>::iterator it=ll.begin();it!=ll.end();++it){
        if(*it==t)return it;
    }
    return ll.end();
}
void delRecursive(std::list<int>::iterator& tb,std::vector<int>& toDel,std::list<int>& ll){
    int t=*tb;
    if(*tb%2==0){
        toDel.push_back(t);
    }
    std::list<int>::iterator it=findIt(t*2,ll);
    if(it!=ll.end()){
        delRecursive(it,toDel,ll);
    }
}
void Test::testList()
{
    std::list<int> ll;
    ll.push_back(1);
    ll.push_back(8);
    ll.push_back(11);
    ll.push_back(22);
    ll.push_back(33);
    ll.push_back(44);
    ll.push_back(55);
    ll.push_back(4);
    ll.push_back(77);
    ll.push_back(88);
    
    //    for(std::list<int>::iterator tb=ll.begin();tb!=ll.end();++tb){
    //			std::vector<int> toDel;
    //            delRecursive(tb,toDel,ll);
    //            for(int k=0;k<toDel.size();k++){
    //                std::list<int>::iterator it=findIt(toDel[k],ll);
    //                if(it!=ll.end())ll.erase(it);
    //            }
    //    }
    std::list<int>::iterator te=ll.end();
    ll.erase(--ll.end());
    std::cout<<(te==ll.end())<<std::endl;
}
