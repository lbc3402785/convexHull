#ifndef CONVEXHULL3D_H
#define CONVEXHULL3D_H
#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <iostream>
#include "data.h"
#include <limits>
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
    std::vector<Plane<T>>list_of_planes;
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
                    result = {extremes[i], extremes[j]};
                }
            }
        }
        return result;
    }
    Plane<T> adjacent_plane(std::vector<Plane<T>>& list_of_planes,Plane<T>&main_plane,Edge<T>&edge)// Finding adjacent planes to an edge
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
    size_t adjacentPlaneIndex(std::vector<Plane<T>>& list_of_planes,size_t mainIndex,Edge<T>&edge)// Finding adjacent planes to an edge
    {
        for (size_t k=0;k<list_of_planes.size();k++){
			if (visited[k])continue;
            Plane<T> plane =list_of_planes[k];
            std::array<Edge<T>,3> edges = plane.get_edges();
            if (!(plane == list_of_planes[mainIndex]) && (std::count(edges.begin(),edges.end(),edge)>0)){
                return k;
            }
        }
    }
    int calc_horizon(size_t index,std::vector<Plane<T>>&visited_planes,Plane<T>& plane,Point3<T>& eye_point,std::vector<Edge<T>>&edge_list)//Calculating the horizon for an eye to make new faces
    {
        if (plane.distance(eye_point) > 1e-10){
            visited_planes.push_back(plane);
            visited[index]=true;
            std::array<Edge<T>,3> edges = plane.get_edges();
            for (Edge<T> edge : edges){
                size_t neighbourIndex=adjacentPlaneIndex(list_of_planes,index,edge);
				if (neighbourIndex >= list_of_planes.size())continue;
//                Plane<T> neighbour = adjacent_plane(list_of_planes,plane,edge);
                Plane<T> neighbour=list_of_planes[neighbourIndex];
                if (std::count(visited_planes.begin(), visited_planes.end(), neighbour)==0 ){

                    int  result = calc_horizon(neighbourIndex,visited_planes,neighbour,eye_point,edge_list);
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
    ConvexHull3D(std::vector<Point3T>& points):points(points){

        T x_min_temp,x_max_temp,y_min_temp,y_max_temp,z_min_temp,z_max_temp ;
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
    }
    void setCorrectNormal(std::vector<Point3<T>>& possible_internal_points,Plane<T>& plane){
        for (Point3T point : possible_internal_points){
            T dist = dotProduct<T>(plane.normal,point - plane.pointA);
            if(dist != 0) {
                if(dist > 1e-10){
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
    std::vector<std::array<size_t,3>> Quickhull(){
        std::array<Point3T,2> line=extremeLine();
        Point3T thirdPoint=max_dist_line_point<T>(points,line[0],line[1]);
        Plane<T> firstPlane(line[0],line[1],thirdPoint);
        Point3T fourthPoint=max_dist_plane_point<T>(points,firstPlane);
        std::vector<Point3T> possible_internal_points={line[0],line[1],thirdPoint,fourthPoint};
        Plane<T> secondPlane(line[0],line[1],fourthPoint);
        Plane<T> thirdPlane(line[0],fourthPoint,thirdPoint);
        Plane<T> fourthPlane(line[1],thirdPoint,fourthPoint);
        setCorrectNormal(possible_internal_points,firstPlane);
        setCorrectNormal(possible_internal_points,secondPlane);
        setCorrectNormal(possible_internal_points,thirdPlane);
        setCorrectNormal(possible_internal_points,fourthPlane);
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
            for(size_t k=0;k<list_of_planes.size();++k){
                if(visited[k])continue;
				//visited[k] = true;
                Plane<T> working_plane=list_of_planes[k];
                if(working_plane.to_do.size()>0){
                    any_left = true;
                    Point3T eye_point = find_eye_point(working_plane, working_plane.to_do); // Calculate the eye point of the face
					std::vector<Edge<T>> edge_list;
					std::vector<Plane<T>> visited_planes;
                    calc_horizon(k,visited_planes,working_plane, eye_point, edge_list);
//                    std::vector<Plane<T>>::iterator ret=list_of_planes.end();
//                    for (Plane<T> internal_plane : visited_planes)// # Remove the internal planes
//                    {
//                        ret=std::remove(list_of_planes.begin(), ret,internal_plane);
//                    }
//                    list_of_planes.erase(ret,list_of_planes.end());
//                    std::vector<Plane<T>>(list_of_planes).swap(list_of_planes);
					std::vector<Point3T> temp_to_do;
					for (Plane<T> internal_plane : visited_planes) {
						temp_to_do.insert(temp_to_do.end(), internal_plane.to_do.begin(), internal_plane.to_do.end());
					}
					std::sort(temp_to_do.begin(), temp_to_do.end());
					temp_to_do.erase(std::unique(temp_to_do.begin(), temp_to_do.end()), temp_to_do.end());
                    for (Edge<T> edge : edge_list)//# Make new planes
                    {
                        Plane<T> new_plane(edge.pointA, edge.pointB, eye_point);
                        setCorrectNormal(possible_internal_points,new_plane);
                        new_plane.calculate_to_do(temp_to_do);
                        list_of_planes.push_back(new_plane);
                        visited.push_back(false);
                    }
                }
            }//end for
			std::cout << "1111" << std::endl;
		}//end while
        std::vector<std::array<size_t,3>> result;
        for(size_t k=0;k<list_of_planes.size();++k){
            if(visited[k])continue;
            Plane<T>plane = list_of_planes[k];
            std::array<size_t,3> face={plane.pointA.id,plane.pointB.id,plane.pointC.id};
            result.push_back(face);
        }
        return result;
    }//end
};

#endif // CONVEXHULL3D_H
