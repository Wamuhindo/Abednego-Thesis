#include "KalmanPredictor.h"
#include "FilterState.h"
#include <math.h>
// Constructors

KalmanPredictor::KalmanPredictor(const Detection &initialState, int ID, cv::Scalar color,int setup)
        : Predictor(initialState.label, ID, color,setup), filter(nullptr) {
    dlib::matrix<double, numStates, numStates> F; // System dynamics matrix
    dlib::matrix<double, numObservations, numStates> H; // Output matrix
    dlib::matrix<double, numStates, numStates> Q; // Process noise covariance
    dlib::matrix<double, numObservations, numObservations> R; // Measurement noise covariance
    dlib::matrix<double, numStates, numStates> P; // Estimate error covariance

    // define constant velocity model
    F = 1, 0, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 1;

    H = 1, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0;

    // Covariance values from SORT
    Q = 1, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, .01, 0, 0,
            0, 0, 0, 0, 0, .01, 0,
            0, 0, 0, 0, 0, 0, .0001;

    R = 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 10, 0,
            0, 0, 0, 10;

    P = 10, 0, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0, 0,
            0, 0, 0, 10, 0, 0, 0,
            0, 0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0, 10000;

    filter = std::make_shared<dlib::kalman_filter<numStates, numObservations>>(
            dlib::kalman_filter<numStates, numObservations>());
    filter->set_transition_model(F);
    filter->set_observation_model(H);
    filter->set_process_noise(Q);
    filter->set_measurement_noise(R);
    filter->set_estimation_error_covariance(P);
    dlib::matrix<double, numObservations, 1> x0(boundingBoxToMeas(initialState.bb));
    filter->update(x0);
}

KalmanPredictor::KalmanPredictor(KalmanPredictor &&rhs)
        : Predictor(std::move(rhs)), filter(std::move(rhs.filter)) {}


KalmanPredictor &KalmanPredictor::operator=(KalmanPredictor &&rhs) {
    Predictor::operator=(std::move(rhs));
    filter = std::move(rhs.filter);
    return *this;
}

// Methods



void KalmanPredictor::update() {
    Predictor::update();
    filter->update();
}

void KalmanPredictor::update(const Detection &det) {
    Predictor::update(det);
    filter->update(boundingBoxToMeas(det.bb));
}

//extract direction from velocity

//extract direction from velocity
int KalmanPredictor::velToDir(std::shared_ptr<dlib::kalman_filter<7L, 4L>> filter_, double vx, double vy){
        //std::cout<<" vel here("<<std::to_string(vx)<<", "<<std::to_string(vy)<<")"<<std::endl;

        double nx = filter_->get_predicted_next_state()(0);
        double ny = filter_->get_predicted_next_state()(1);
        double area = filter_->get_predicted_next_state()(2);
        double nvx = filter_->get_predicted_next_state()(4);
        double nvy = filter_->get_predicted_next_state()(5);


   /*      if(std::abs(vy-nvy)>0.3){
            return 0;
        }

        if((vx==0.0 && vy==0.0) || std::abs(vx)<2 || std::abs(vx)>15 || std::abs(vy)>4 || (vx<0 && vy<0) || pow((vx*vx)+(vy*vy),0.5)<3){
            std::cout<<" return 0"<<std::endl;
            return 0;
        } */
          if((vx==0.0 && vy==0.0) ){
            
            return 0;
        }
        if (getSetup()==1|| getSetup()==2 || getSetup()==3){
            if(pow((vx*vx)+(vy*vy),0.5)>0.6){
                if(vx<0 && pow((vx*vx)+(vy*vy),0.5)>0.6) return -1;
                else if(vx>0 && pow((vx*vx)+(vy*vy),0.5)>0.6) return 1;
                else return 0;
            }else return 0;
        }else {
            if(pow((vx*vx)+(vy*vy),0.5)>0.6){
                if(  vy>0.5 ) return 2;
                else if ( vy<-0.5 ) return -2;
                else if(vx<0 && pow((vx*vx)+(vy*vy),0.5)>0.6) return -1;
                else if(vx>0 && pow((vx*vx)+(vy*vy),0.5)>0.6) return 1;
                else return 0;
            }else return 0;
        }
        
}
// Getters
int KalmanPredictor::computeDir(const dlib::matrix<double, Predictor::numStates, 1> &state, double vx, double vy){
    double rectifiedArea = std::max(state(2), 0.);
    double width = std::sqrt(rectifiedArea * state(3));
    double height = rectifiedArea / width;
    
    double lastWidth = getLastBB().width;
    double lastHeight = getLastBB().height;


     //std::cout<<" lastwidth("<<std::to_string(lastWidth)<<", "<<std::to_string(lastHeight)<<")"<<std::endl;   
     // std::cout<<" width("<<std::to_string(width)<<", "<<std::to_string(height)<<")"<<std::endl;   
    double vel = pow((vx*vx)+(vy*vy),0.5);
         saveLastBB(BoundingBox(state(0),state(1),width,height));
    if(getSetup()==0){

        if ((lastWidth<=width && lastHeight<=height) && vy>0.1 ) return 2;
        else if ((lastWidth>=width && lastHeight>=height) && vy<-0.1 ) return -2;
        else if (vx<0 && vel>0.6) return -1;
        else if (vx>0 && vel>0.6) return 1;
        else return 0;

    }else if(getSetup()==1){

        if(lastWidth>width && vx<0)return -1;
        else if(lastWidth<width && vx>0)return 1;
        else return 0;

    }else if (getSetup()==2){

        if(lastWidth>width  && vx<0)return -1;
        else if(lastWidth<width && vx>0)return 1;
        else return 0;

    }else if(getSetup()==3){

        if(vx<0 && vel>0.6) return -1;
        else if(vx>0 && vel>0.6) return 1;
        else return 0;

    }

   

}

Detection KalmanPredictor::getPredictedNextDetection() const {
    return Detection(getLabel(), 1, stateToBoundingBox(filter->get_predicted_next_state()));
}

Tracking KalmanPredictor::getTracking()  {


    double rectifiedArea = std::max(filter->get_current_state()(2), 0.) ;
    double width = std::sqrt(rectifiedArea * filter->get_current_state()(3));
    double height = rectifiedArea/width;
    double x = filter->get_current_state()(0);
    double y = filter->get_current_state()(1);
    auto a = (*filter);

    if(getAlgoVelocity()==0){
        if(velToDir(filter,filter->get_current_state()(4),filter->get_current_state()(5))!=0){
            
            //std::cout<<"state "<<a.get_current_state()(4)<<std::endl; computeDir(filter->get_current_state(),filter->get_current_state()(4),filter->get_current_state()(5)),
            return Tracking(
                getLabel(), 
                getID(), 
                stateToBoundingBox(filter->get_current_state()),
                getColor(), 
                velToDir(filter,filter->get_current_state()(4),filter->get_current_state()(5)),
                true, 
                filter->get_current_state()(0),
                filter->get_current_state()(1),
                filter->get_current_state()(4),
                filter->get_current_state()(5),
                filter->get_current_state()(2),
                filter->get_current_state()(3),
                filter->get_current_state()(6),
                filter->get_predicted_next_state()(0),
                filter->get_predicted_next_state()(1),
                filter->get_predicted_next_state()(4),
                filter->get_predicted_next_state()(5),
                filter->get_predicted_next_state()(2),
                filter->get_predicted_next_state()(3),
                filter->get_predicted_next_state()(6));
            }
        else{
            return Tracking(
                getLabel(), 
                getID(), 
                stateToBoundingBox(filter->get_current_state()),
                getColor(), 
                velToDir(filter,filter->get_current_state()(4),filter->get_current_state()(5)),
                false, 
                filter->get_current_state()(0),
                filter->get_current_state()(1),
                filter->get_current_state()(4),
                filter->get_current_state()(5),
                filter->get_current_state()(2),
                filter->get_current_state()(3),
                filter->get_current_state()(6),
                filter->get_predicted_next_state()(0),
                filter->get_predicted_next_state()(1),
                filter->get_predicted_next_state()(4),
                filter->get_predicted_next_state()(5),
                filter->get_predicted_next_state()(2),
                filter->get_predicted_next_state()(3),
                filter->get_predicted_next_state()(6));
        }
    }else {
        if(computeDir(filter->get_current_state(),filter->get_current_state()(4),
                filter->get_current_state()(5))!=0){
            
            //std::cout<<"state "<<a.get_current_state()(4)<<std::endl; computeDir(filter->get_current_state(),filter->get_current_state()(4),filter->get_current_state()(5)),
            return Tracking(
                getLabel(), 
                getID(), 
                stateToBoundingBox(filter->get_current_state()),
                getColor(), 
                computeDir(filter->get_current_state(),filter->get_current_state()(4),
                filter->get_current_state()(5)),
                true, 
                filter->get_current_state()(0),
                filter->get_current_state()(1),
                filter->get_current_state()(4),
                filter->get_current_state()(5),
                filter->get_current_state()(2),
                filter->get_current_state()(3),
                filter->get_current_state()(6),
                filter->get_predicted_next_state()(0),
                filter->get_predicted_next_state()(1),
                filter->get_predicted_next_state()(4),
                filter->get_predicted_next_state()(5),
                filter->get_predicted_next_state()(2),
                filter->get_predicted_next_state()(3),
                filter->get_predicted_next_state()(6));
            }
        else{
            return Tracking(
                getLabel(), 
                getID(), 
                stateToBoundingBox(filter->get_current_state()),
                getColor(), 
                computeDir(filter->get_current_state(),filter->get_current_state()(4),
                filter->get_current_state()(5)),
                false, 
                filter->get_current_state()(0),
                filter->get_current_state()(1),
                filter->get_current_state()(4),
                filter->get_current_state()(5),
                filter->get_current_state()(2),
                filter->get_current_state()(3),
                filter->get_current_state()(6),
                filter->get_predicted_next_state()(0),
                filter->get_predicted_next_state()(1),
                filter->get_predicted_next_state()(4),
                filter->get_predicted_next_state()(5),
                filter->get_predicted_next_state()(2),
                filter->get_predicted_next_state()(3),
                filter->get_predicted_next_state()(6));
        }
    }
}

 std::shared_ptr<dlib::kalman_filter<7L, 4L>> KalmanPredictor::getFilter(){
         return filter;
 }

