#ifndef CONVEXHULL2D_H
#define CONVEXHULL2D_H
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#ifdef SHOW
#include <opencv2/opencv.hpp>
#endif
template <typename T,typename Point2T>
class ConvexHull2D
{
private:
    T EPSILON;
    //计算叉积，小于0说明p1在p2的逆时针方向，即p0p1的极角大于p0p2的极角
    T cross_product(Point2T p0, Point2T p1, Point2T p2)
    {
        return ((p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y));
    }

    //计算距离
    T dis(Point2T p1, Point2T p2)
    {
        return (T)sqrt((T)(p1.x - p2.x)*(p1.x - p2.x) +(T)(p1.y - p2.y)*(p1.y - p2.y));
    }
    bool com(const size_t &i, const size_t &j)
    {
        T temp = cross_product(points[indices[0]], points[i], points[j]);
        if (fabs(temp) < EPSILON)//极角相等按照距离从小到大排序
        {
            return dis(points[indices[0]], points[i]) < dis(points[indices[0]], points[j]);
        }
        else
        {
            if(useOpenCVCoordinate){
                return temp<0;
            }else{
                return temp > 0;
            }
        }
    }
    bool useOpenCVCoordinate;
public:
    std::vector<Point2T> points;
    std::vector<size_t> indices;
    ConvexHull2D(std::vector<Point2T>& points,bool useOpenCVCoordinate=false)
        :points(points),useOpenCVCoordinate(useOpenCVCoordinate)
    {
        assert(points.size()>3);
        indices.resize(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        size_t index = 0;
        for(size_t i=1;i<points.size();i++)
        {
            if(useOpenCVCoordinate){
                if (points[i].y > points[index].y || (points[i].y ==points[index].y && points[i].x < points[index].x))
                {
                    index = i;
                }
            }else{
                if (points[i].y < points[index].y || (points[i].y ==points[index].y && points[i].x < points[index].x))
                {
                    index = i;
                }
            }

        }
        indices[0]=index;
        indices[index]=0;
        init();

    }
    void init(){
        EPSILON =std::numeric_limits<T>::epsilon();
    }
    std::vector<size_t> convexHull(){
        std::vector<size_t> result;
        result.push_back(indices[0]);
//        std::sort(indices.begin()+1, indices.end(),&ConvexHull2D<T,Point2T>::com);
        std::sort(indices.begin()+1, indices.end(),[&](const size_t&i,const size_t&j){
            T temp = cross_product(points[indices[0]], points[i], points[j]);

            if (fabs(temp) < EPSILON)//极角相等按照距离从小到大排序
            {
                //std::cout<<"i,j,temp:"<<i<<","<<j<<","<<temp<<std::endl;
                return dis(points[indices[0]], points[i]) < dis(points[indices[0]], points[j]);
            }
            else
            {
                if(useOpenCVCoordinate){
                    return temp<0;
                }else{
                    return temp > 0;
                }
            }
        });

        result.push_back(indices[1]);
        size_t n=points.size();
        size_t top = 1;
        for (size_t i = 2; i < n; ++i)
        {
            T temp=cross_product(points[result[top - 1]], points[indices[i]], points[result[top]]);
            while ((!useOpenCVCoordinate&&temp>= 0)||(useOpenCVCoordinate&&temp<=0))
            {
                --top;
                result.pop_back();
                if(top<=0)break;
                temp=cross_product(points[result[top - 1]], points[indices[i]], points[result[top]]);

            }
            result.push_back(indices[i]);
            //std::cout<<"indices["<<i<<"]:"<<indices[i]<<std::endl;
            ++top;
        }
        return result;
    }
};
#endif // CONVEXHULL2D_H
