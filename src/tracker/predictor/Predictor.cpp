#include "Predictor.h"

// Constructors

Predictor::Predictor(int label, int ID, cv::Scalar color,int setup)
        : label(label), ID(ID), timeSinceUpdate(0), hitStreak(0),color(color),setup(setup) {}

Predictor::Predictor(Predictor &&rhs)
        : label(rhs.label), ID(rhs.ID), timeSinceUpdate(rhs.timeSinceUpdate), hitStreak(rhs.hitStreak) {}

Predictor &Predictor::operator=(Predictor &&rhs) {
    label = rhs.label;
    ID = rhs.ID;
    timeSinceUpdate = rhs.timeSinceUpdate;
    hitStreak = rhs.hitStreak;
    return *this;
}

// Methods

int Predictor::getSetup() {
    return setup;
}
void Predictor::setSetup(int setup) {
    this->setup = setup;
}

void Predictor::update() {
    timeSinceUpdate++;
    hitStreak = 0;
}

void Predictor::update(const Detection &det) {
    timeSinceUpdate = 0;
    hitStreak++;
}

// Getters

int Predictor::getTimeSinceUpdate() const {
    return timeSinceUpdate;
}

int Predictor::getHitStreak() const {
    return hitStreak;
}

int Predictor::getLabel() const {
    return label;
}

int Predictor::getID() const {
    return ID;
}

cv::Scalar Predictor::getColor() const {
    return color;
}


void Predictor::setAlgovelocity(int algo){
    algoVelocity = algo;
}
int Predictor::getAlgoVelocity(){
    return algoVelocity;
}

// Functions

BoundingBox Predictor::stateToBoundingBox(const dlib::matrix<double, numStates, 1> &state) {
    double rectifiedArea = std::max(state(2), 0.);
    double width = std::sqrt(rectifiedArea * state(3));
    double height = rectifiedArea / width;
   // std::cout<<" vel ("<<state(4)<<", "<<state(5)<<", "<<state(0)<<")"<<std::endl;

    return BoundingBox(state(0), state(1), width, height);
}


void Predictor::saveLastBB(BoundingBox bb){
    lastBB.cx = bb.cx;
    lastBB.cy = bb.cy;
    lastBB.width = bb.width;
    lastBB.height = bb.height;
}
BoundingBox Predictor::getLastBB(){
    return lastBB;
}



dlib::matrix<double, Predictor::numObservations, 1> Predictor::boundingBoxToMeas(const BoundingBox &bb) {
    dlib::matrix<double, Predictor::numObservations, 1> z;
    z = bb.cx, bb.cy, bb.area(), bb.ratio();
    return z;
}

std::ostream &operator<<(std::ostream &os, const Predictor &p) {
    os << p.getPredictedNextDetection();
    return os;
}

