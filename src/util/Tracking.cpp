#include "Tracking.h"

// Constructors

Tracking::Tracking(int label, int ID, const BoundingBox &bb, cv::Scalar color,const int dir,bool flag, double cx,
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
        double da_dt_ )
        : label(label), ID(ID), bb(bb), color(color),dir(dir),flag(flag), cx(cx) ,
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
        da_dt_(da_dt_){
               
        }

Tracking::Tracking(const Tracking &rhs)
        : label(rhs.label), ID(rhs.ID), bb(rhs.bb),color(rhs.color),dir(rhs.dir),flag(rhs.flag),
        cx(rhs.cx) ,
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
        ratio_(rhs.ratio_),
        da_dt_(rhs.da_dt_){}

Tracking::Tracking(Tracking &&rhs)
        : label(std::move(rhs.label)), ID(std::move(rhs.ID)), bb(std::move(rhs.bb)),color(std::move(rhs.color)),dir(std::move(rhs.dir)),flag(std::move(rhs.flag)), cx(std::move(rhs.cx)) ,
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
        da_dt_(std::move(rhs.da_dt_)) {}

// Functions

std::ostream &operator<<(std::ostream &os, const Tracking &t) {
    os << "Label: " << t.label << " ID: " << t.ID << " " << t.bb<<" "<< "cx: " << t.cx << " cy: " << t.cy<< " vx: " << t.vx <<" vy: "<<t.vy<<" area: "<<t.area<<" ratio: "<<t.ratio<<" da_dt: "<<t.da_dt;
    return os;
}