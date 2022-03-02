#ifndef CPP_TRACKING_H
#define CPP_TRACKING_H


#include "BoundingBox.h"
#include <opencv2/highgui/highgui_c.h>
#include <ostream>
#include <dlib/filtering.h>
#include "FilterState.h"
class Tracking {
 public:   
    Tracking(int label, int ID, const BoundingBox &bb,cv::Scalar color,const int dir, bool flag, double cx ,
        double cy ,
        double vx ,
        double vy ,
        double area,
        double ratio ,
        double da_dt ,
        double cx_ ,
        double cy_ ,
        double vx_ ,
        double vy_ ,
        double area_ ,
        double ratio_ ,
        double da_dt_ );

    Tracking(const Tracking &rhs);

    Tracking(Tracking &&rhs);

    // Prevent assignment
    Tracking &operator=(const Tracking &rhs) = delete;

    Tracking &operator=(Tracking &&rhs) = delete;
    const int label;
    const int ID;
    const BoundingBox bb;
    cv::Scalar color;
    const int dir;
    const bool flag;
    const double cx ;
    const double cy ;
    const double vx ;
    const double vy ;
    const double area;
    const double ratio ;
    const double da_dt ;
    const double cx_ ;
    const double cy_ ;
    const double vx_ ;
    const double vy_ ;
    const double area_ ;
    const double ratio_ ;
    const double da_dt_ ;
   
};

std::ostream &operator<<(std::ostream &os, const Tracking &t);


#endif //CPP_TRACKING_H