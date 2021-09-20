#ifndef DATA_H
#define DATA_H
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>

template <typename T>
class Vector3
{
public:
    T x,y,z;
    Vector3(){
    }
    Vector3(T x,T y,T z,size_t id=-1):x(x),y(y),z(z){

    }
    void operator=(const Vector3& other){
        x=other.x;
        y=other.y;
        z=other.z;
    }
    bool operator==(const Vector3& other){
        return (other.x==x&&other.y==y&&other.z==z);
    }
    T length(){
        return std::sqrt(x*x+y*y+z*z);
    }
};
template <typename T>
class Point3
{
public:
    T x,y,z;
    size_t id;
    Point3(){
       id=0;
    }
    Point3(T x,T y,T z,size_t id=-1):x(x),y(y),z(z),id(id){

    }
    Vector3<T> operator-(const Point3& other){
        return Vector3<T>(x-other.x,y-other.y,z-other.z);
    }
    void operator=(const Point3& other){
        x=other.x;
        y=other.y;
        z=other.z;
        id=other.id;
    }
    bool operator==(const Point3& other){
        return other.id==id;
    }
    friend inline bool operator< (const Point3<T>& lhs, const Point3<T>& rhs){
        return lhs.id<rhs.id;
    }
};
template <typename T>
class Edge
{
public:
    typedef Point3<T> Point3T;
    typedef Vector3<T> Vector3T;
    Point3T pointA;
    Point3T pointB;
    std::array<size_t,2> indexes;
    std::array<size_t,2> sortedIndexes;
    Edge(){}
    Edge(Point3T pointA,Point3T pointB):pointA(pointA),pointB(pointB)
    {
        indexes[0]=(pointA.id);
        indexes[1]=(pointB.id);
        sortedIndexes=indexes;
        std::sort(sortedIndexes.begin(),sortedIndexes.end());
    }
    friend inline bool operator!=(const Edge<T>& lhs, const Edge<T>& rhs){ return !(lhs == rhs); }
    friend bool operator==(const Edge<T>& lhs,const Edge<T>& rhs){
        if(lhs.sortedIndexes[0]!=rhs.sortedIndexes[0]||lhs.sortedIndexes[1]!=rhs.sortedIndexes[1]){
            return false;
        }
        return true;
    }
};
template <typename T>
class Plane
{
public:
    typedef Point3<T> Point3T;
    typedef Vector3<T> Vector3T;
    Point3T pointA;
    Point3T pointB;
    Point3T pointC;
    Vector3T normal;
    Edge<T> edge1;
    Edge<T> edge2;
    Edge<T> edge3;
    std::vector<Point3T> to_do;
    std::array<size_t,3> indexes;
    std::array<size_t,3> sortedIndexes;
    T dis;
    Plane(Point3T pointA,Point3T pointB,Point3T pointC):pointA(pointA),pointB(pointB),pointC(pointC)
    {
        init();
    }
    void init(){
        calcNorm();
        edge1 = Edge<T>(pointA, pointB);
        edge2 = Edge<T>(pointB, pointC);
        edge3 = Edge<T>(pointC, pointA);
        indexes[0]=(pointA.id);
        indexes[1]=(pointB.id);
        indexes[2]=(pointC.id);
        sortedIndexes=indexes;
        std::sort(sortedIndexes.begin(),sortedIndexes.end());
    }


    void calcNorm(){
        Vector3T point1 = pointA - pointB;
        Vector3T point2 = pointB - pointC;
        Vector3T normVector = cross(point1,point2);
        T length = normVector.length();
        normVector.x = normVector.x/length;
        normVector.y = normVector.y/length;
        normVector.z = normVector.z/length;
        normal = normVector;
        dis = dotProduct(normal,pointA-Point3T(0,0,0));
    }

    T distance(Point3T &pointX){
        return (dotProduct(normal,pointX - pointA));
    }

    std::array<Edge<T>,3> get_edges(){
        return {edge1, edge2, edge3};
    }

    void calculate_to_do(std::vector<Point3T>& points){

        for( Point3T p : points){
            T dist = distance(p);
            if (dist > 1.0e-10){
                to_do.push_back(p);
            }
        }


    }
    friend inline bool operator!=(const Plane<T>& lhs, const Plane<T>& rhs){ return !(lhs == rhs); }
    friend bool operator==(const Plane<T>& lhs,const Plane<T>& rhs){
        if(lhs.sortedIndexes[0]!=rhs.sortedIndexes[0]||lhs.sortedIndexes[1]!=rhs.sortedIndexes[1]||lhs.sortedIndexes[2]!=rhs.sortedIndexes[2]){
            return false;
        }
        return true;
    }

};
template <typename T>
Vector3<T> cross(Vector3<T> &vecA, Vector3<T> &vecB){
    T x = (vecA.y*vecB.z) - (vecA.z*vecB.y);
    T y = (vecA.z*vecB.x) - (vecA.x*vecB.z);
    T z = (vecA.x*vecB.y) - (vecA.y*vecB.x);
    return Vector3<T>(x, y, z);
}
template <typename T>
T dotProduct(Vector3<T>& vecA, Vector3<T> vecB){
    return (vecA.x*vecB.x + vecA.y*vecB.y + vecA.z*vecB.z);
}



template <typename T>
T distLine(Point3<T> &pointA, Point3<T>& pointB,Point3<T>& pointX)// #Calculate the distance of a point from a line
{
    Vector3<T> vec1 = pointX - pointA;
    Vector3<T> vec2 = pointX - pointB;
    Vector3<T> vec3 = pointB - pointA;
    Vector3<T> vec4 = cross<T>(vec1, vec2);
    if (vec3.length() == 0){
        return vec1.length();
    }

    else{
        return vec4.length()/vec3.length();
    }
}
template <typename T>
Point3<T> max_dist_line_point(std::vector<Point3<T>>&points,Point3<T>& pointA,Point3<T>& pointB)
{
    // #Calculate the maximum distant point from a line for initial simplex
    T maxDist = 0;
    Point3<T> maxDistPoint;
    for(Point3<T> point : points){
        if (!(pointA == point) && !(pointB == point))
        {
            T dist = std::abs(distLine<T>(pointA,pointB,point));
            if (dist>maxDist){
                maxDistPoint = point;
                maxDist = dist;
            }
        }
    }
    return maxDistPoint;
}
template <typename T>
Point3<T> max_dist_plane_point(std::vector<Point3<T>>&points,Plane<T>&plane)//: # Calculate the maximum distance from the plane
{
    T maxDist = 0;
    Point3<T> maxDistPoint;
    for (Point3<T> point : points){
        T dist = abs(plane.distance(point));
        if (dist > maxDist){
            maxDist = dist;
            maxDistPoint = point;
        }
    }
    return maxDistPoint;
}
template <typename T>
Point3<T> find_eye_point(Plane<T>& plane,std::vector<Point3<T>>& to_do_list)//: # Calculate the maximum distance from the plane
{
    T maxDist = 0;
    Point3<T> maxDistPoint;
    for (Point3<T> point : to_do_list){
        T dist = plane.distance(point);
        if (dist > maxDist){
            maxDist = dist;
            maxDistPoint = point;
        }
    }

    return maxDistPoint;
}
#endif // DATA_H
