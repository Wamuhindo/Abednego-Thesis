#ifndef CPP_TRACKINGENGINE_H
#define CPP_TRACKINGENGINE_H

#include "Tracker.h"
#include "predictor/Predictor.h"
#include "predictor/kalman/KalmanPredictor.h"
#include <opencv2/highgui/highgui_c.h>
#include <memory>
#include <vector>

class TrackingEngine : public Tracker {
    struct Association;

public:
    TrackingEngine() = default;
    TrackingEngine(int maxAge);
    virtual ~TrackingEngine() = default;

    /**
      * Uses a linear velocity Kalman filters to predict locations of objects from previous frame.
      * Associates detections to Kalman filters using an Affinity measure and the Hungarian Algorithm.
      */
    std::vector<Tracking> track(const std::vector<Detection> &detections,int setup, bool model_free, int algo) override;
    void setDetectAreaRight(int detect_right);
    void setDetectAreaLeft(int detect_left);
    void setDetectAreaTop(int detect_top);
    void setDetectAreaBottom(int detect_bottom);
private:
    const int maxAge = 5; // Original: 1
    const int minHits = 0; // Original: 3
    const double detectionThreshold = 0.4;
    const double affinityThreshold = 0.1;
    std::vector<std::shared_ptr<Predictor>> predictors;
    int trackCount = 0;
    int frameCount = 0;

    /**
     * Uses an Affinity measure and Hungarian algorithm to determine which detection corresponds to which Kalman filter.
     */
    static Association associateDetectionsToPredictors(
            const std::vector<Detection> &detections,
            const std::vector<std::shared_ptr<Predictor>> &predictors,
            double (*affinityMeasure)(const BoundingBox &a, const BoundingBox &b),
            double affinityThreshold);

    struct Association {
        std::vector<std::pair<int, int>> matching;
        std::vector<int> unmatchedDetections;
        std::vector<int> unmatchedPredictors;
    };
    int detect_right = 0;
    int detect_left = 0;
    int detect_top = 0;
    int detect_bottom = 0;
};

#endif //CPP_MCSORT_H