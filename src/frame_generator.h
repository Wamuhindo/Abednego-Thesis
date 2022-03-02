
#ifndef CD_FRAME_GENERATOR_H_
#define CD_FRAME_GENERATOR_H_

#include <opencv2/core/types.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <functional>
#include <condition_variable>

#include <opencv2/opencv.hpp>

#include "events/event_cd.h"

namespace Prophesee {

/// \brief Utility class to display CD events
class CDFrameGenerator {
public:
    /// \brief Default constructor
    ///
    /// \param width, height Size of the image
    /// \param process_all_frames If true, it will process all frames, not just the latest one. Remark that setting
    /// this to true can slow down the process.
    CDFrameGenerator(long width, long height, bool process_all_frames = false);
    CDFrameGenerator();

    /// \brief Destructor
    ~CDFrameGenerator();

    /// \brief Sets the color used to generate the frame
    ///
    /// By default, the frame generated will be 8 bits single channel with the default colors grey, white, black as
    /// background, on and off colors respectively.
    /// If the parameter \a colored is false, then the generated frame will be 8 bits single channel and the first
    /// channel will be used to define each colors (i.e the other channels will be ignored).
    /// If the parameter \a colored is true, then the generated frame will be 8 bits three channels.
    ///
    /// \param background_color The color used as background, when no events were received for
    ///                         a pixel, default: grey cv::Scalar::all(128)
    /// \param on_color         The color used for on events, default: white cv::Scalar::all(255)
    /// \param off_color        The color used for off events, default: black cv:Scalar::all(0)
    /// \param colored          If the generated frame should be single or three channels
    void set_colors(const cv::Scalar &background_color, const cv::Scalar &on_color, const cv::Scalar &off_color,
                    bool colored = false);

    /// \brief Adds the buffer of events to be displayed
    ///
    /// \param begin,end Buffer's range
    void add_events(const Prophesee::EventCD *begin, const Prophesee::EventCD *end);

    /// \brief Sets the time interval to display events
    ///
    /// The events shown at each refresh are such that their timestamps are in the last 'display_accumulation_time_us'
    /// microseconds from the last received event timestamp.
    /// \param display_accumulation_time_us The time interval to display events from up to now, in us.
    void set_display_accumulation_time_us(Prophesee::timestamp display_accumulation_time_us);

    /// \brief Starts the generator thread
    ///
    /// \param fps Frame rate
    /// \param cb Function to call every time a new frame is available. It takes in input the time (in us) of the new
    /// frame and the frame. The frame passed as a parameter is guaranteed to be available and left untouched until the
    /// next time the callback is called. This means that you don't need to make a deep copy of it, if you only intend
    /// to use the frame until the next one is made available by the callback.
    /// \return True if the thread started successfully, false otherwise. Also returns false, if the thread is already
    /// started.
    bool start(std::uint16_t fps, const std::function<void(const Prophesee::timestamp &, const cv::Mat &)> &cb);

    /// \brief Stops the generator thread
    ///
    /// \return True if the thread has been stopped successfully, false otherwise. Return false if the thread had not
    /// been previously started
    //  If the thread is not started, this function returns false.
    bool stop();
    void setMask(int w_l, int wr, int h_u, int h_d);
    void setGeometry(long width, long height,bool process_all_frames);
    void addMask(cv::Rect mask);
    std::vector<cv::Rect> getMasks();
    void removeMask(cv::Rect rect);
    bool checkEventArea(int x, int y);
    void setMask_(std::vector<std::vector<cv::Point>> masks_);
    std::vector<std::vector<cv::Point>> getMasks_();
    bool checkInsidePolygon(cv::Point p);
    bool insideSinglePolygon(std::vector<cv::Point> &polyPoints,cv::Point p);

private:
    void generate();

    void update_frame(std::vector<Prophesee::EventCD>::iterator it_begin,
                      std::vector<Prophesee::EventCD>::iterator it_end, Prophesee::timestamp ts_begin,
                      Prophesee::timestamp ts_end);

    // Vector of timestamps
    cv::Mat_<std::int32_t> ts_history_;
    Prophesee::timestamp ts_offset_ = 0;

    // Image to display
    cv::Mat frame_, frame_swap_;
    Prophesee::timestamp frame_period_ = -1;
    std::function<void(const Prophesee::timestamp &, const cv::Mat &)> frame_cb_;
    cv::Scalar bg_color_, on_color_, off_color_;
    bool colored_;

    // Time interval to display events
    uint32_t display_accumulation_time_us_ = 5000;

    // Is frame dropping allowed ?
    bool process_all_frames_ = false;

    // Next processing timestamp according to frame_period value
    Prophesee::timestamp next_process_ts_ = 0;

    // Events to display
    std::vector<Prophesee::EventCD> events_queue_front_, events_queue_back_;
    std::vector<cv::Rect> masks;
    std::vector<std::vector<cv::Point>> masks_;
    std::mutex mask_mutex_;
    std::mutex processing_mutex_;
    bool thread_should_process_;

    // The worker thread
    std::thread thread_;
    std::mutex thread_cond_mutex_;
    std::condition_variable thread_cond_;
    std::atomic<bool> thread_should_stop_{true};

    int event_number;
    int height_mask_up;
    int height_mask_down;
    int width_mask_left;
    int width_mask_right;


};

} // namespace Prophesee

#endif /* CD_FRAME_GENERATOR_H_ */