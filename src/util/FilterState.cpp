#include "FilterState.h"

// Constructors

FilterState::FilterState(    
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
        ): 
        cx(cx) ,
        cy(cy) ,
        vx(vx) ,
        vy(vy) ,
        area(area),
        ratio(ratio) ,
        da_dt(da_dt) ,
        cx_(cx_) ,
        cy_(cy_) ,
        vx_(vx_) ,
        vy_ (vy_),
        area_(area_) ,
        ratio_(ratio_) ,
        da_dt_(da_dt_){}

FilterState::FilterState(const FilterState &rhs)
        : cx(rhs.cx) ,
        cy(rhs.cy) ,
        vx(rhs.vx) ,
        vy(rhs.vy) ,
        area(rhs.area),
        ratio(rhs.ratio) ,
        da_dt(rhs.da_dt) ,
        cx_(rhs.cx_) ,
        cy_(rhs.cy_) ,
        vx_(rhs.vx_) ,
        vy_ (rhs.vy_),
        area_(rhs.area_) ,
        ratio_(rhs.ratio_) ,
        da_dt_(rhs.da_dt_) {}

FilterState::FilterState(FilterState &&rhs)
        : cx(std::move(rhs.cx)) ,
        cy(std::move(rhs.cy)) ,
        vx(std::move(rhs.vx)) ,
        vy(std::move(rhs.vy)) ,
        area(std::move(rhs.area)),
        ratio(std::move(rhs.ratio)) ,
        da_dt(std::move(rhs.da_dt)) ,
        cx_(std::move(rhs.cx_)) ,
        cy_(std::move(rhs.cy_)) ,
        vx_(std::move(rhs.vx_)) ,
        vy_ (std::move(rhs.vy_)),
        area_(std::move(rhs.area_)) ,
        ratio_(std::move(rhs.ratio_)) ,
        da_dt_(std::move(rhs.da_dt_)){ }

// Functions
std::ostream &operator<<(std::ostream &os, const FilterState &t){
    os << "cx: " << t.cx << " cy: " << t.cy<< " vx: " << t.vx <<" vy: "<<t.vy<<" area: "<<t.area<<" ratio: "<<t.ratio<<" da_dt: "<<t.da_dt;//<< "cx_: " << t.cx_ << " cy_: " << t.cy_<< " vx_: " << t.vx_ <<" vy_: "<<t.vy_<<" area_: "<<t.area_<<" ratio_: "<<t.ratio_<<" da_dt_: "<<t.da_dt_;
    return os;
}