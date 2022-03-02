#ifndef CPP_FILTERSTATE_H
#define CPP_FILTERSTATE_H


#include "BoundingBox.h"
#include <opencv2/highgui/highgui_c.h>
#include <ostream>
#include <dlib/filtering.h>
class FilterState {
  public:
    FilterState(    
        double cx ,
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
        double da_dt_ 
);
 
    FilterState(const FilterState &rhs);

    FilterState(FilterState &&rhs);
 
    // Prevent assignment
   FilterState &operator=(const FilterState &rhs) = delete;

   FilterState &operator=(FilterState &&rhs) = delete;

    double cx ;
    double cy ;

    double vx ;
    double vy ;
    double area;
    double ratio ;
    double da_dt ;
    double cx_ ;
    double cy_ ;
    double vx_ ;
    double vy_ ;
    double area_ ;
    double ratio_ ;
    double da_dt_ ;

};
std::ostream &operator<<(std::ostream &os, const FilterState &t);

#endif //CPP_TRACKING_H