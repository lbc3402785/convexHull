#ifndef CONVEXHULL3D_H
#define CONVEXHULL3D_H
#include <vector>
#include <array>
#include <list>
#include <algorithm>
#include <numeric>
#include <iostream>
#include "data.h"
#include <limits>
#include <fstream>
#include <exception>
template <typename T>
class ConvexHull3D
{
private:
    typedef Point3<T> Point3T;
    typedef Vector3<T> Vector3T;
    std::vector<Point3T> points;
    Point3T xmax,xmin;
    Point3T ymax,ymin;
    Point3T zmax,zmin;
    std::list<Plane<T>>list_of_planes;
	typename std::list<Plane<T>>::iterator cur;
    std::vector<bool> visited;
    T distance(Point3T&p, Point3T&q)// # Gives the Euclidean distance
    {
        return std::sqrt((p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z));
    }
    std::array<Point3T,2> extremeLine(){
        std::array<Point3T,6> extremes={xmax,xmin,ymax,ymin,zmax,zmin};
        std::array<Point3T,2> result;
        T maxi = 0;
        for (size_t i=0;i<6;++i){
            for (size_t j=i+1;j<6;++j){
                T dist = distance(extremes[i], extremes[j]);
                if (dist > maxi){
                    maxi = dist;
                    result = {extremes[i], extremes[j]};
                }
            }
        }
        return result;
    }
    Plane<T> adjacent_plane(std::list<Plane<T>>& list_of_planes,Plane<T>&main_plane,Edge<T>&edge)// Finding adjacent planes to an edge
    {
        for (size_t k=0;k<list_of_planes.size();k++){
            if (visited[k])continue;
            Plane<T> plane =list_of_planes[k];
            std::array<Edge<T>,3> edges = plane.get_edges();
            if (!(plane == main_plane) && (std::count(edges.begin(),edges.end(),edge)>0)){
                return plane;
            }
        }
    }
    typename std::list<Plane<T>>::iterator adjacentPlaneIndex(std::list<Plane<T>>& list_of_planes,Plane<T>& mainPlane,Edge<T>&edge)// Finding adjacent planes to an edge
    {
        for (typename std::list<Plane<T>>::iterator it=list_of_planes.begin();it!=list_of_planes.end();++it){
            Plane<T> plane =*it;
            std::array<Edge<T>,3> edges = plane.get_edges();
            if (!(plane == mainPlane) && (std::count(edges.begin(),edges.end(),edge)>0)){
                return it;
            }
        }
        return list_of_planes.end();
    }
    int calc_horizon(typename std::list<Plane<T>>::iterator& tb,std::vector<Plane<T>>&visited_planes,Plane<T>& plane,Point3<T>& eye_point,std::vector<Edge<T>>&edge_list)//Calculating the horizon for an eye to make new faces
    {
        if (plane.distance(eye_point) > 1e-10){
            visited_planes.push_back(plane);
			if (cur == tb) {
				cur = list_of_planes.erase(cur);
			}
			else {
				list_of_planes.erase(tb);
			}
            std::array<Edge<T>,3> edges = plane.get_edges();
            for (Edge<T> edge : edges){
                typename std::list<Plane<T>>::iterator it=adjacentPlaneIndex(list_of_planes,plane,edge);
                if(it==list_of_planes.end())continue;
                Plane<T> neighbour =*it;
                if (std::count(visited_planes.begin(), visited_planes.end(), neighbour)==0 ){
                    int  result=calc_horizon(it,visited_planes,neighbour,eye_point,edge_list);
                    if(result == 0)
                        edge_list.push_back(edge);


                }
            }
            return 1;
        }
        else{
            return 0;
        }
    }

public:
    T x_min_temp,x_max_temp,y_min_temp,y_max_temp,z_min_temp,z_max_temp ;
    T range;
    ConvexHull3D(std::vector<T>& vertices){
        for(size_t k=0;k<vertices.size()/3;++k){
            points.push_back(Point3<float>(vertices[3*k+0],vertices[3*k+1],vertices[3*k+2],k));
        }
        init();
    }
    ConvexHull3D(std::vector<std::array<T,3>>& vertices){
        for(size_t k=0;k<vertices.size();++k){
            points.push_back(Point3<float>(vertices[k][0],vertices[k][1],vertices[k][2],k));
        }
        init();
    }
    ConvexHull3D(std::vector<Point3T>& points):points(points){
        init();
    }
    void init(){
        x_max_temp=y_max_temp=z_max_temp=std::numeric_limits<T>::min();
        x_min_temp=y_min_temp=z_min_temp=std::numeric_limits<T>::max();
        size_t num=points.size();
        for (size_t i=0;i<num;++i){
            if (points[i].x > x_max_temp){
                x_max_temp = points[i].x;
                xmax = points[i];
            }
            if (points[i].x < x_min_temp){
                x_min_temp = points[i].x;
                xmin = points[i];
            }
            if (points[i].y > y_max_temp) {
                y_max_temp = points[i].y;
                ymax = points[i];
            }
            if (points[i].y < y_min_temp) {
                y_min_temp = points[i].y;
                ymin = points[i];
            }
            if (points[i].z > z_max_temp) {
                z_max_temp = points[i].z;
                zmax = points[i];
            }
            if (points[i].z < z_min_temp) {
                z_min_temp = points[i].z;
                zmin = points[i];
            }

        }
        T xRange=std::max(std::fabs(x_min_temp),std::fabs(x_max_temp));
        T yRange=std::max(std::fabs(y_min_temp),std::fabs(y_max_temp));
        T zRange=std::max(std::fabs(z_min_temp),std::fabs(z_max_temp));
        range=std::max(std::max(xRange,yRange),zRange);
    }
    void setCorrectNormal(std::vector<Point3<T>>& possible_internal_points,Plane<T>& plane){
        for (Point3T point : possible_internal_points){
			if (point.id == plane.pointA.id || point.id == plane.pointB.id || point.id == plane.pointC.id)continue;
            T dist1 = dotProduct<T>(plane.normal,point - plane.pointA);
            T dist2 = dotProduct<T>(plane.normal, point - plane.pointB);
            T dist3 = dotProduct<T>(plane.normal, point - plane.pointC);
            T dist = std::min(std::min(dist1, dist2), dist3);
            if(dist!=0) {
                if(dist > 1e-7){
                    Point3<T> tmp=plane.pointA;
                    plane.pointA=plane.pointB;
                    plane.pointB=tmp;
                    plane.init();
                    return;
                }
            }
        }
    }
    std::vector<std::array<size_t,3>> convexHull(){
        return Quickhull();
    }
    void savePlane(Plane<T> &p,std::string name) {
        std::ofstream out(name);
        out << "v " << p.pointA.x << " " << p.pointA.y << " " << p.pointA.z << std::endl;
        out << "v " << p.pointB.x << " " << p.pointB.y << " " << p.pointB.z << std::endl;
        out << "v " << p.pointC.x << " " << p.pointC.y << " " << p.pointC.z << std::endl;
        out << "f " << 1 << " " << 2 << " " << 3 << std::endl;
    }
    void saveCurrentOutput(std::list<Plane<T>>&list_of_planes, std::vector<Point3<float>>&points, std::string name) {
        std::vector<std::array<size_t, 3>> faces;
        std::list<Plane<T>>::iterator tb = list_of_planes.begin(), te = list_of_planes.end();
        for (; tb != te; ++tb) {
            Plane<T>plane = *tb;
            std::array<size_t, 3> face = { plane.pointA.id,plane.pointB.id,plane.pointC.id };
            faces.push_back(face);
        }
        std::ofstream out(name);
        for (int k = 0; k < points.size(); k++) {
            out << "v " << points[k].x << " " << points[k].y << " " << points[k].z << std::endl;
        }
        for (int k = 0; k < faces.size(); k++) {
            out << "f " << faces[k][0] + 1 << " " << faces[k][1] + 1 << " " << faces[k][2] + 1 << std::endl;
        }
    }
    std::vector<std::array<size_t,3>> Quickhull(){
        assert(points.size()>=3);
        std::array<Point3T,2> line=extremeLine();
        Point3T thirdPoint=max_dist_line_point<T>(points,line[0],line[1]);
        Plane<T> firstPlane(line[0],line[1],thirdPoint);


        if (points.size() == 3) {
            list_of_planes.push_back(firstPlane);
        }else{
            Point3T fourthPoint=max_dist_plane_point<T>(points,firstPlane);
            std::vector<Point3T> possible_internal_points={line[0],line[1],thirdPoint,fourthPoint};
            Plane<T> secondPlane(line[0],line[1],fourthPoint);
            Plane<T> thirdPlane(line[0],fourthPoint,thirdPoint);
            Plane<T> fourthPlane(line[1],thirdPoint,fourthPoint);

            setCorrectNormal(possible_internal_points,firstPlane);
            setCorrectNormal(possible_internal_points,secondPlane);
            setCorrectNormal(possible_internal_points,thirdPlane);
            setCorrectNormal(possible_internal_points,fourthPlane);
            savePlane(firstPlane, "firstPlane.obj");
            savePlane(secondPlane, "secondPlane.obj");
            savePlane(thirdPlane, "thirdPlane.obj");
            savePlane(fourthPlane, "fourthPlane.obj");
            firstPlane.calculate_to_do(points);
            secondPlane.calculate_to_do(points);
            thirdPlane.calculate_to_do(points);
            fourthPlane.calculate_to_do(points);

            list_of_planes.push_back(firstPlane);
            list_of_planes.push_back(secondPlane);
            list_of_planes.push_back(thirdPlane);
            list_of_planes.push_back(fourthPlane);
            bool any_left=true;
            visited.assign(list_of_planes.size(),false);
            while(any_left){
                any_left = false;
                std::list<Plane<T>>::iterator tb = list_of_planes.begin(), te= list_of_planes.end();
                for(;tb!=te;){
                    Plane<T> working_plane=*tb;
                    if(working_plane.to_do.size()>0){
                        any_left = true;
                        Point3T eye_point = find_eye_point(working_plane, working_plane.to_do); // Calculate the eye point of the face
                        std::vector<Edge<T>> edge_list;
                        std::vector<Plane<T>> visited_planes;
						cur = tb;
                        if(calc_horizon(tb,visited_planes,working_plane, eye_point, edge_list)==0){
                            ++tb;
						}
						else {
							tb = cur;
						}
                        std::vector<Point3T> temp_to_do;
                        for (Plane<T> internal_plane : visited_planes) {
                            temp_to_do.insert(temp_to_do.end(), internal_plane.to_do.begin(), internal_plane.to_do.end());
                        }
                        std::sort(temp_to_do.begin(), temp_to_do.end());
                        temp_to_do.erase(std::unique(temp_to_do.begin(), temp_to_do.end()), temp_to_do.end());
                        for (Edge<T> edge : edge_list)//# Make new planes
                        {
                            Plane<T> new_plane(edge.pointA, edge.pointB, eye_point);
                            //setCorrectNormal(possible_internal_points,new_plane);
                            new_plane.calculate_to_do(temp_to_do);
                            list_of_planes.push_back(new_plane);
                        }
                        //saveCurrentOutput(list_of_planes, points, "cur.obj");
                    }
                    else {
                        ++tb;
                    }
                }//end for
                std::cout << "1111" << std::endl;
            }//end while
        }

        std::vector<std::array<size_t,3>> result;
        std::list<Plane<T>>::iterator tb = list_of_planes.begin(), te= list_of_planes.end();
        for(;tb!=te;++tb){
            Plane<T>plane = *tb;
            std::array<size_t,3> face={plane.pointA.id,plane.pointB.id,plane.pointC.id};
            result.push_back(face);
        }
        return result;
    }//end
};

#endif // CONVEXHULL3D_H
