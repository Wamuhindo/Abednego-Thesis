#include "TrackingEngine.h"

#include "Affinity.h"
#include "predictor/kalman/KalmanPredictor.h"

#include <dlib/optimization.h>


TrackingEngine::TrackingEngine(int maxAge):maxAge(maxAge){
    
}
// Methods
cv::RNG rng(12345);
std::vector<Tracking> TrackingEngine::track(const std::vector<Detection> &detections,int setup,bool model_free, int algo) {
    frameCount++;

    // Filter detections on confidence
    std::vector<Detection> strongDetections;
    for (const auto &detection : detections) {
        if (detection.confidence > detectionThreshold) {
            strongDetections.push_back(detection);
        }
    }

    Association association = associateDetectionsToPredictors(strongDetections, predictors,
                                                              Affinity::expCost, affinityThreshold);

    // Update matched predictors with assigned detections

    for (const auto &match : association.matching) {
        predictors.at(match.second)->update(strongDetections.at(match.first));
        predictors.at(match.second)->setSetup(setup);
        predictors.at(match.second)->setAlgovelocity(algo);
    }

    for (const auto p : association.unmatchedPredictors) {
        predictors.at(p)->update();
        predictors.at(p)->setSetup(setup);
        predictors.at(p)->setAlgovelocity(algo);
    }

    // Create and initialise new predictors for unmatched detections
    for (const auto id : association.unmatchedDetections) {
        cv::Scalar color = cv::Scalar(rng.uniform(0,256),rng.uniform(0,256),rng.uniform(0,256));
        auto predictor = std::make_shared<KalmanPredictor>(strongDetections.at(id), ++trackCount, color,setup);
        predictor->setSetup(setup);
         predictor->setAlgovelocity(algo);
        if(model_free){
            if(setup!=0){
                
                if((predictor->getTracking().cx<detect_left && predictor->getTracking().dir>=0) || predictor->getTracking().cx>(640-detect_right) && predictor->getTracking().dir<=0)
                    predictors.push_back(predictor);
            }else{
             
                if((predictor->getTracking().cy<detect_top && predictor->getTracking().dir>=0) || predictor->getTracking().cy>(480-detect_bottom) && predictor->getTracking().dir<=0)
                    predictors.push_back(predictor);}
        }else{
            predictors.push_back(predictor);
        }

    }

    // Remove predictors that have been inactive for too long
    predictors.erase(std::remove_if(
            predictors.begin(), predictors.end(),
            [this](const std::shared_ptr<Predictor> &predictor) {
                return predictor->getTimeSinceUpdate() > maxAge;
            }), predictors.end());

    //remove if reached the end of the frame
    predictors.erase(std::remove_if(
            predictors.begin(), predictors.end(),
            [this](const std::shared_ptr<Predictor> &predictor) {
                return (predictor->getPredictedNextDetection().bb.x2()>630 && predictor->getTracking().dir>0 && predictor->getPredictedNextDetection().bb.area()<20000);//||(predictor->getPredictedNextDetection().bb.x1()<15 && predictor->getTracking().dir<0 && predictor->getPredictedNextDetection().bb.area()<7000)
            }), predictors.end());

    predictors.erase(std::remove_if(
            predictors.begin(), predictors.end(),
            [this](const std::shared_ptr<Predictor> &predictor) {
                return (predictor->getPredictedNextDetection().bb.x2()<20 && predictor->getTracking().dir<0 && predictor->getPredictedNextDetection().bb.area()<20000);//||(predictor->getPredictedNextDetection().bb.x1()<15 && predictor->getTracking().dir<0 && predictor->getPredictedNextDetection().bb.area()<7000)
            }), predictors.end());

    // Return trackings from active predictors
    std::vector<Tracking> trackings;
    for (auto it = predictors.begin(); it != predictors.end(); ++it) {
        if ((*it)->getTimeSinceUpdate() < 1 &&
            ((*it)->getHitStreak() >= minHits || frameCount <= minHits)) {
            auto t = (*it)->getTracking();
            //std::cout<<"in TrackingEngine "<<t<<std::endl;
            trackings.push_back(t);
        }
    }
    return trackings;
}

void TrackingEngine::setDetectAreaRight(int detect_right_){
    detect_right = detect_right_;
}
void TrackingEngine::setDetectAreaLeft(int detect_left_){
    detect_left = detect_left_;
}
void TrackingEngine::setDetectAreaTop(int detect_top_){
    detect_top = detect_top_;
}
void TrackingEngine::setDetectAreaBottom(int detect_bottom_){
    detect_bottom = detect_bottom_;
}


TrackingEngine::Association TrackingEngine::associateDetectionsToPredictors(
        const std::vector<Detection> &detections,
        const std::vector<std::shared_ptr<Predictor>> &predictors,
        double (*affinityMeasure)(const BoundingBox &a, const BoundingBox &b),
        double affinityThreshold) {

    const int DOUBLE_PRECISION = 100;
    std::vector<std::pair<int, int>> matches;
    std::vector<int> unmatchedDetections;
    std::vector<int> unmatchedPredictors;

    if (predictors.empty()) {
        for (int i = 0; i < detections.size(); ++i)
            unmatchedDetections.push_back(i);
        return TrackingEngine::Association{matches, unmatchedDetections, unmatchedPredictors};
    }

    dlib::matrix<int> cost(detections.size(), predictors.size());
    for (size_t row = 0; row < detections.size(); ++row) {
        for (size_t col = 0; col < predictors.size(); ++col) {
            cost(row, col) = int(DOUBLE_PRECISION * affinityMeasure(
                    detections.at(row).bb,
                    predictors.at(col)->getPredictedNextDetection().bb));
        }
    }

    // Zero-pad to make it square
    if (cost.nr() > cost.nc()) {
        cost = dlib::join_rows(cost, dlib::zeros_matrix<int>(1, cost.nr() - cost.nc()));
    } else if (cost.nc() > cost.nr()) {
        cost = dlib::join_cols(cost, dlib::zeros_matrix<int>(cost.nc() - cost.nr(), 1));
    }

    std::vector<long> assignment = dlib::max_cost_assignment(cost);

    // Filter out matches with low IoU, including those for indices from padding
    for (int d = 0; d < assignment.size(); ++d) {
        if (cost(d, assignment[d]) < affinityThreshold * DOUBLE_PRECISION) {
            if (d < detections.size()) {
                unmatchedDetections.push_back(d);
            }
            if (assignment[d] < predictors.size()) {
                unmatchedPredictors.push_back(int(assignment[d]));
            }
        } else {
            matches.push_back(std::pair<int, int>(d, assignment[d]));
        }
    }
    return TrackingEngine::Association{matches, unmatchedDetections, unmatchedPredictors};
}