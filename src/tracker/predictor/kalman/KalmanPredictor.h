#ifndef CPP_KALMANPREDICTOR_H
#define CPP_KALMANPREDICTOR_H


#include "../Predictor.h"
#include "../../../util/Tracking.h"
#include <dlib/filtering.h>
#include <opencv2/highgui/highgui_c.h>
#include <vector>
#include <memory>

class KalmanPredictor : public Predictor {
public:
    KalmanPredictor(const Detection &initialState, int ID, cv::Scalar color,int setup);

    KalmanPredictor(KalmanPredictor &&rhs);

    KalmanPredictor &operator=(KalmanPredictor &&rhs);

    /**
     * Advances the state vector using current estimation.
     */
    void update() override;

    /**
     * Updates and advances the state vector using given Detection as observation.
     */
    void update(const Detection &det) override;

    /**
     * Returns the predicted next state as Detection.
     */
    Detection getPredictedNextDetection() const override;

    /**
     * Returns the current state as Tracking.
     */
    Tracking getTracking() override;

    std::shared_ptr<dlib::kalman_filter<numStates, numObservations>> getFilter();
   
    void saveLastBoundingBox(double x, double y, double width, double height);
    int computeDir(const dlib::matrix<double, Predictor::numStates, 1> &state, double vx, double vy);
     int velToDir(std::shared_ptr<dlib::kalman_filter<7L, 4L>> filter_, double vx, double vy);

private:
    std::shared_ptr<dlib::kalman_filter<numStates, numObservations>> filter;
   

};


#endif //CPP_KALMANPREDICTOR_H