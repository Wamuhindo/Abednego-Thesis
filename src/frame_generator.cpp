
#include "frame_generator.h"

namespace Prophesee {
CDFrameGenerator::CDFrameGenerator(long width, long height, bool process_all_frames) :
    process_all_frames_(process_all_frames),width_mask_left(0),width_mask_right(0),height_mask_down(0),height_mask_up(0){
    ts_history_ = cv::Mat_<std::int32_t>(height, width, -1);
    frame_      = cv::Mat(height, width, CV_8U);
    frame_swap_ = cv::Mat(height, width, CV_8U);
    bg_color_   = cv::Scalar::all(128);
    on_color_   = cv::Scalar::all(255);
    off_color_  = cv::Scalar::all(0);
    colored_    = false;
    frame_.setTo(bg_color_);
   
}

CDFrameGenerator::CDFrameGenerator(){
 
}

void CDFrameGenerator::setGeometry(long width, long height,bool process_all_frames){
    ts_history_ = cv::Mat_<std::int32_t>(height, width, -1);
    frame_      = cv::Mat(height, width, CV_8U);
    frame_swap_ = cv::Mat(height, width, CV_8U);
    bg_color_   = cv::Scalar::all(128);
    on_color_   = cv::Scalar::all(255);
    off_color_  = cv::Scalar::all(0);
    colored_    = false;
    frame_.setTo(bg_color_);
    process_all_frames_ = process_all_frames;
    width_mask_left = 0;
    width_mask_right = 0;
    height_mask_down = 0;
    height_mask_up = 0;
   
}

CDFrameGenerator::~CDFrameGenerator() {
    stop();
}
void CDFrameGenerator::set_colors(const cv::Scalar &background_color, const cv::Scalar &on_color,
                                  const cv::Scalar &off_color, bool colored) {
    bg_color_  = background_color;
    on_color_  = on_color;
    off_color_ = off_color;
    colored_   = colored;
}

void CDFrameGenerator::setMask(int wl, int wr, int h_u, int h_d){

    height_mask_up=h_u;
    height_mask_down=h_d;
    width_mask_right=wr;
    width_mask_left=wl;
}

void CDFrameGenerator::addMask(cv::Rect mask){
    {
        std::lock_guard<std::mutex> lock(mask_mutex_);
        masks.push_back(mask);
    }
    
}

std::vector<cv::Rect> CDFrameGenerator::getMasks(){
        {
        std::lock_guard<std::mutex> lock(mask_mutex_);
        return masks;
    }
    
}


void CDFrameGenerator::setMask_(std::vector<std::vector<cv::Point>> _masks){
     
    masks_ = _masks;
    //std::cout<<"Mask size: "<<std::to_string(masks_.size())<<std::endl;

}

std::vector<std::vector<cv::Point>> CDFrameGenerator::getMasks_(){
        
        return masks_;
    }



void CDFrameGenerator::removeMask(cv::Rect rect){
    
    std::vector<cv::Rect>::iterator it3;
    {
        for (it3 = masks.begin(); it3 != masks.end(); ++it3) {
            if (it3->x == rect.x && it3->y == rect.y && it3->width == rect.width && it3->height == rect.height) {
                it3 = masks.erase(it3); // After erasing, it3 is now pointing the next location.
                --it3; // Go to the prev location because of ++it3 in the end of for loop.
                break;
            }
        }
    }
}

void CDFrameGenerator::add_events(const Prophesee::EventCD *begin, const Prophesee::EventCD *end) {
    if (thread_should_stop_ && !process_all_frames_) {
        // if the generator is not yet started and we don't have to process all frames, we don't need to keep the events
        return;
    }
    {
        std::lock_guard<std::mutex> lock(processing_mutex_);
        events_queue_front_.insert(events_queue_front_.end(), begin, end);
    }
    if ((end-1)->t >= next_process_ts_) {
        {
            do {
                next_process_ts_ += frame_period_;
            } while ((end-1)->t >= next_process_ts_);
            std::unique_lock<std::mutex> lock(thread_cond_mutex_);
            thread_should_process_ = true;
        }
        thread_cond_.notify_one();
    }
}
void CDFrameGenerator::generate() {
    Prophesee::timestamp display_accumulation_time_us;
    Prophesee::timestamp next_frame_time = frame_period_;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(thread_cond_mutex_);
            thread_cond_.wait(lock, [this] { return thread_should_process_ || thread_should_stop_; });
            thread_should_process_ = false;
            if (thread_should_stop_)
                break;
            // get current values while we have the lock acquired
            display_accumulation_time_us = display_accumulation_time_us_;
        }
        {
            std::unique_lock<std::mutex> lock(processing_mutex_);
            std::swap(events_queue_front_, events_queue_back_);
        }
        if (events_queue_back_.empty()) {
            continue;
        }
        // update pixel intensities with events in the time window
        if (process_all_frames_) {
            auto last_ev_it = events_queue_back_.end();
            auto it_end     = events_queue_back_.begin();
            event_number = events_queue_back_.size();
            Prophesee::timestamp ts_begin;
            while (true) {
                ts_begin      = next_frame_time - display_accumulation_time_us;
                auto it_begin = std::lower_bound(
                    it_end, last_ev_it, ts_begin,
                    [](const Prophesee::EventCD &ev, const Prophesee::timestamp &t) { return ev.t < t; });
                it_end = std::lower_bound(
                    it_begin, last_ev_it, next_frame_time,
                    [](const Prophesee::EventCD &ev, const Prophesee::timestamp &t) { return ev.t < t; });
                update_frame(it_begin, it_end, ts_begin, next_frame_time);
                if (it_end != last_ev_it) { // if it_end == last_ev_it it means we are not finished processing the
                                            // events for last frame
                    std::swap(frame_, frame_swap_);
                    // call the callback with updated frame
                    frame_cb_(next_frame_time, frame_swap_);
                    next_frame_time += frame_period_;
                } else {
                    break;
                }
            }
        } else {
            Prophesee::timestamp ts_end   = events_queue_back_.back().t;
            Prophesee::timestamp ts_begin = ts_end - display_accumulation_time_us;
            auto it_end                   = events_queue_back_.end();
            auto it_begin =
                std::lower_bound(events_queue_back_.begin(), events_queue_back_.end(), ts_begin,
                                 [](const Prophesee::EventCD &ev, const Prophesee::timestamp &t) { return ev.t < t; });
            event_number = events_queue_back_.size();  
            update_frame(it_begin, it_end, ts_begin, ts_end);
            std::swap(frame_, frame_swap_);
            // call the callback with updated frame
            frame_cb_(ts_end, frame_swap_);
        }
        events_queue_back_.clear();
    }
}

bool CDFrameGenerator::checkEventArea(int x, int y){

    std::vector<cv::Rect>::iterator it3;
    bool ret = false;
    {
        for (it3 = masks.begin(); it3 != masks.end(); ++it3) {
            if (it3->x <= x && (it3->x+it3->width)>=x && it3->y<=y && (it3->y +it3->height)>=y) {
                ret = true;
                break;
        }
    }
    }
}

bool CDFrameGenerator::checkInsidePolygon(cv::Point p)
{

        std::vector<std::vector<cv::Point>>::iterator it;
        bool inside = false;
        for (it = masks_.begin(); it != masks_.end(); ++it) {
            
                if (insideSinglePolygon(*it,p)){
                    inside = true;
                    break;
                }
            
        }
        //std::cout<<"Mask size: "<<std::to_string(masks_.size())<<std::endl;
        //std::cout<<"Point ( "<<p.x<<", "<<p.y<<" ) inside polygone?: "<<std::to_string(inside)<<std::endl; 
        return inside;
}

bool CDFrameGenerator::insideSinglePolygon(std::vector<cv::Point> &polyPoints,cv::Point p)
{
  int counter = 0;
  int i;
  double xinters;
  cv::Point p1,p2;
  int N = polyPoints.size();
  
  p1 = polyPoints[0];
  for (i=1;i<=N;i++) {
    p2 = polyPoints[i % N];
    if (p.y > MIN(p1.y,p2.y)) {
      if (p.y <= MAX(p1.y,p2.y)) {
        if (p.x <= MAX(p1.x,p2.x)) {
          if (p1.y != p2.y) {
            xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
            if (p1.x == p2.x || p.x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }

  if (counter % 2 == 0)
    return false;
  else
    return true;
}

void CDFrameGenerator::update_frame(std::vector<Prophesee::EventCD>::iterator it_begin,
                                    std::vector<Prophesee::EventCD>::iterator it_end, Prophesee::timestamp ts_begin,
                                    Prophesee::timestamp ts_end) {
    // update frame type if needed
    frame_.create(frame_.rows, frame_.cols, colored_ ? CV_8UC3 : CV_8U);
    //std::cout<<"Number events"<<std::to_string(event_number)<<std::endl;
    // if the timestamps we want to store overflow a 32 bits integer value
    // we increase the offset and update the stored timestamps
    while (ts_end > ts_offset_ + std::numeric_limits<std::int32_t>::max()) {
        ts_offset_ += std::numeric_limits<std::int32_t>::max();
        ts_history_ -= std::numeric_limits<std::int32_t>::max();
    }

    auto start = std::chrono::high_resolution_clock::now();
    if (colored_) {
        cv::Vec3b on_color(on_color_[0], on_color_[1], on_color_[2]),
            off_color(off_color_[0], off_color_[1], off_color_[2]);
        
        for (auto it = it_begin; it != it_end; ++it) {
            if(it->y>height_mask_up && it->y<(frame_.rows-height_mask_down) && it->x>width_mask_left && it->x<(frame_.cols-width_mask_right)){
                if(!checkInsidePolygon(cv::Point(it->x, it->y))){
                    ts_history_(it->y, it->x)          = static_cast<std::int32_t>(it->t - ts_offset_);
                    frame_.at<cv::Vec3b>(it->y, it->x) = it->p ? on_color : off_color;
                }
            }
        }
    } else {
        uint8_t on_color(on_color_[0]), off_color(off_color_[0]);
        
        for (auto it = it_begin; it != it_end; ++it) {
            if(it->y>height_mask_up && it->y<(frame_.rows-height_mask_down) && it->x>width_mask_left && it->x<(frame_.cols-width_mask_right)){
                if(!checkInsidePolygon(cv::Point(it->x, it->y))){
                    ts_history_(it->y, it->x)        = static_cast<std::int32_t>(it->t - ts_offset_);
                    frame_.at<uint8_t>(it->y, it->x) = it->p ? on_color : off_color;
                }   
            }
        }
    }
     auto finish = std::chrono::high_resolution_clock::now();
     std::chrono::duration<double> elapsed = finish-start;
     //std::cout<<"Event_processing "<<elapsed.count()<<std::endl;
    // reset pixels for too old events
    frame_.setTo(bg_color_, ts_history_ < static_cast<std::int32_t>(ts_begin - ts_offset_));
}
void CDFrameGenerator::set_display_accumulation_time_us(Prophesee::timestamp display_accumulation_time_us) {
    std::lock_guard<std::mutex> lock(thread_cond_mutex_);
    display_accumulation_time_us_ = display_accumulation_time_us;
}
bool CDFrameGenerator::start(std::uint16_t fps,
                             const std::function<void(const Prophesee::timestamp &, const cv::Mat &)> &cb) {
    if (thread_.joinable()) { // Already started
        return false;
    }
    std::unique_lock<std::mutex> lock(thread_cond_mutex_);
    frame_period_          = static_cast<Prophesee::timestamp>(1.e6 / fps + 0.5);
    frame_cb_              = cb;
    next_process_ts_       = frame_period_;
    thread_should_process_ = false;
    thread_should_stop_    = false;
    thread_                = std::thread([this] { generate(); });
    return true;
}
bool CDFrameGenerator::stop() {
    if (!thread_.joinable()) {
        return false;
    }
    {
        std::unique_lock<std::mutex> lock(thread_cond_mutex_);
        thread_should_stop_ = true;
    }
    thread_cond_.notify_one();
    thread_.join();
    return true;
}
} // namespace Prophesee
