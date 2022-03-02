#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <prophesee_driver.h>
#include <thread>
#include <chrono>
#if CV_MAJOR_VERSION >= 4
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/dnn.hpp>
#endif

#include "opencv2/opencv.hpp" 
#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include "./util/Tracking.h"
#include "./util/MyTime.h"

#include <queue>
#include <iterator>
#include <iomanip>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <math.h>
#include <time.h>

#include "frame_generator.h"
#include "./util/Detection.h"
#include "./util/BoundingBox.h"
#include "./util/Tracking.h"
#include "./tracker/TrackingEngine.h"
#include <future>
#include <string>
#include <fstream>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <Eigen/Dense>
#include <atomic>
#include <queue>
#include <boost/property_tree/ptree.hpp>                                        
#include <boost/property_tree/json_parser.hpp> 

namespace po = boost::program_options;
namespace pt = boost::property_tree; 

constexpr float CONFIDENCE_THRESHOLD = 0;
constexpr float NMS_THRESHOLD = 0.4;
constexpr int NUM_CLASSES = 4;
constexpr int FPS = 30;

Prophesee::Camera camera;
std::string biases_file; // camera biases path
std::string filename; //device filename
std::string serial; //device serial number
cv::VideoWriter oVideoWriter;
std::ofstream myLogFile;
std::string logfile;
std::string maskfile;
bool gui = false;
bool modelFree =false;
bool maskSetup = false;

//Variables for video setup
bool video = false;
int frames_per_second_video = 30;
int timeVideo = 10;
std::string videoPath;
std::chrono::high_resolution_clock::time_point last_time;
std::chrono::high_resolution_clock::time_point last_time_video;
int time_after_video;
bool stop_video_recording=false;
std::chrono::high_resolution_clock::time_point time_stop_video;

//algrithm to compute the direction
int algo_direction = 0;

//Variables for image output setup 
bool image = false;
double timeImage = 0.1;
std::string imagePath;
std::chrono::high_resolution_clock::time_point last_time_cnt;
std::chrono::high_resolution_clock::time_point last_time_image;

//alarm variables

std::string alarmType;
bool is_alarm_set = false;
int alarm_value;

//Varaibles used for the morphological filter
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int max_dim_size = 500;
int setup = 0;
int max_setup = 3;
int const max_elem = 2;
int const max_kernel_size = 21;

//pseudo-frames used for processing
cv::Mat cd_frame;
cv::Mat out_img;
cv::Mat fore;
cv::Mat img; //used in detectAndTrackObject function

cv::RNG rangeColor(12345);

//Threshold for Bounding Box filtering by geometry
int w_threshold = 5;
int h_threshold = 5;
int a_threshold = 25;

//model-free object filter
int max_width_pedestrian = 0;
int min_width_pedestrian = 0;
int max_height_pedestrian = 0;
int min_height_pedestrian =0;
int max_width_bike = 0;
int min_width_bike = 0;
int max_height_bike = 0;
int min_height_bike =0;
int max_width_car = 2000;
int min_width_car = 0;
int max_height_car = 2000;
int min_height_car = 00;

//detect gaps
int detect_right = 0;
int detect_left = 0;
int detect_top = 0;
int detect_bottom = 0;

std::chrono::duration<double> elapsed; //keep the elapsed time from the object processing to the object tracking
std::string cd_window_name("OBJECT TRACKING");
std::string cd_window_name2("original");
 
bool camera_running = true; // true if the camera is running

int frameID = 0;

//Tracking Engines

TrackingEngine trackerCar(5);
TrackingEngine trackerPedestrian(16);
TrackingEngine trackerBikes(5);

std::vector<int> ids_car(10);
std::vector<int> ids_bikes(10);
std::vector<int> ids_pedestrians(10);
std::vector<int> ids_car_alarm(10);
std::vector<int> ids_bikes_alarm(10);
std::vector<int> ids_pedestrians_alarm(10);
static int number_cars = 0;
static int number_pedestrians = 0;
static int number_bikes = 0;
static int number_cars_alarm = 0;
static int number_pedestrians_alarm = 0;
static int number_bikes_alarm = 0;

// deep learning network
cv::dnn::Net net; 
std::vector< std::__cxx11::basic_string<char> > output_names; //keep the output names for the object detection model
std::map<int, std::vector<Detection>> _detections;

//Variables for managing the drawing of the masks
bool mouseMoved = false;
int x1_mouse = 0;
int x2_mouse = 0;
int y1_mouse = 0;
int y2_mouse = 0;
bool left_clicked = false;
bool btn_down = false;
int value_btn_mask = 0;
std::vector<std::vector<cv::Point>> masks(0);
std::vector<std::pair<std::vector<cv::Point>,bool>> masks_(0);

//Instatiation of a frame generator
Prophesee::CDFrameGenerator cd_frame_generator;

cv::Mat mergeFrame(cv::Mat &img_orig, cv::Mat &img_track);
cv::Mat detectAndTrackObject(cv::Mat &img);
void mergeOverlappingBoxes(const std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);
void trackingProcess(int,void*);
void trackObject(std::vector<cv::Rect> &boxes,cv::Mat &img_frame);
void discardInsiderBoxes(std::vector<cv::Rect> &boxes);

cv::Point kalmanPredict() ;
cv::Point kalmanCorrect(float x, float y); 
void option90();
void option45(cv::Mat kernel1);
void option0();
void option15(cv::Mat kernel1);
void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata);
void btnCallBackFunc(int event, void* userdata);
static const int ESCAPE = 27;
static const int SPACE  = 32;


Detection boxToDetection(cv::Rect box, double conf, int classe);
std::map<int, std::vector<Detection>> detectBoxes(std::vector<cv::Rect_<int>> (&boxes)[NUM_CLASSES], std::vector<float>(&scores)[NUM_CLASSES], std::vector<int>(&indices)[NUM_CLASSES], int frameId);
//std::map<int, std::vector<Detection>> detectBoxes(std::vector<cv::Rect> &boxes, int frameId);
void detectAndTrack(std::vector<cv::Rect> &boxes, int frameId, cv::Mat &frame);
void drawTrackingsCar(std::vector<Tracking> &trackings, cv::Mat &frame);
void drawTrackingsBikes(std::vector<Tracking> &trackings,cv::Mat &frame);
void drawTrackingsPedestrian(std::vector<Tracking> &trackings,cv::Mat &frame);
void writeVectorOfVector(cv::FileStorage &fs, std::string name, std::vector<std::pair<std::vector<cv::Point>,bool>> &data);
bool insidePolygon(std::vector<cv::Point> &polyPoints,cv::Point p);
//void readVectorOfVector(cv::FileStorage &fns, std::string name, std::vector<std::pair<std::vector<cv::Point>,bool>> &data);
void readVectorOfVector(cv::FileStorage &fns, std::string name, std::vector<std::vector<cv::Point>> &data);
void drawMasks_(cv::Mat &frm);
void drawMasks_w(cv::Mat &frm);
void detectYolo(cv::Mat &frame_);
void bb_callback(int,void*);

//function that allow to display the window with a particular frame rate
int process_ui_for(int delay_ms) {
    auto then = std::chrono::high_resolution_clock::now();
    int key   = cv::waitKey(delay_ms);
    auto now  = std::chrono::high_resolution_clock::now();
    // cv::waitKey will not wait if no window is opened, so we wait for him, if needed
    std::this_thread::sleep_for(std::chrono::milliseconds(
        delay_ms - std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count()));

    return key;
}


// save mask in file
void writeVectorOfVector(cv::FileStorage &fs, std::string name, std::vector<std::pair<std::vector<cv::Point>,bool>> &data)
{
    fs << name;
    fs << "{";
    for (int i = 0; i < data.size(); i++)
    {
        if(data[i].second==true){
        fs << name + "_" + std::to_string(i);
        std::vector<cv::Point> tmp = data[i].first;
        fs << tmp;
        }
    }
    fs << "}";
}
//Function to read the vector of vectors (Maskfile)
void readVectorOfVector(cv::FileStorage &fns, std::string name, std::vector<std::vector<cv::Point>> &data)
{
    data.clear();
    cv::FileNode fn = fns[name];
    if (fn.empty()){
        return;
    }

    cv::FileNodeIterator current = fn.begin(), it_end = fn.end();
    for (; current != it_end; ++current)
    {
        std::vector<cv::Point> tmp;
        cv::FileNode item = *current;
        item >> tmp;
        data.push_back(tmp);
    }
}


std::string getVideoFilename(){
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::string s(32, '\0');
    std::strftime(&s[0], s.size(), "output_%Y_%m_%d_%H_%M_%S.avi", std::localtime(&now));
    return s;
}

std::string getTimeStr(){
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y_%m_%d_%H:%M:%S", std::localtime(&now));
    return s;
}

std::string getTimeFromTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp){
     std::time_t now = std::chrono::system_clock::to_time_t(timestamp);

    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y_%m_%d_%H:%M:%S", std::localtime(&now));
    return s;
}

void write_header(std::ofstream &myFile){
    myFile << "Timestamp" << " Time"<<" cx"<<" cy"<<" vx"<<" vy"<<" area"<<" ratio"<<" da_dt"<<" cx_"<<" cy_"<<" vx_"<<" vy_"<<" area_"<<" dir"<<" label"<<" ID"<<"\n";
}

void write_csv(std::string filename, Tracking tracking, std::string label, std::chrono::time_point<std::chrono::system_clock> timestamp){

    // Create an output filestream object
     if(!myLogFile.is_open()){
        std::cout<<"opening stream..."<<std::endl;
        myLogFile.open(filename, std::ofstream::out | std::ofstream::app);
        std::cout<<"writing header..."<<std::endl;
        write_header(myLogFile);
        std::cout<<"writing header finished"<<std::endl;
    }

    if(myLogFile.is_open()){
        auto timestamp = std::chrono::system_clock::now();
        auto const cache_time = timestamp.time_since_epoch().count();
        // Send the column name to the stream
        log_watch<std::chrono::microseconds> micro("%FT%T.");
        myLogFile <<timestamp.time_since_epoch().count()<<" ";
        myLogFile <<micro<<" ";
        //std::cout<<tracking<<std::endl;
        //std::cout<<tracking.filter<<std::endl;
        myLogFile <<tracking.cx<<" ";
        myLogFile <<tracking.cy<<" ";
        myLogFile <<tracking.vx<<" ";
        myLogFile <<tracking.vy<<" ";
        myLogFile <<tracking.area<<" ";
        myLogFile <<tracking.ratio<<" ";
        myLogFile <<tracking.da_dt<<" ";
        myLogFile <<tracking.cx_<<" ";
        myLogFile <<tracking.cy_<<" ";
        myLogFile <<tracking.vx_<<" ";
        myLogFile <<tracking.vy_<<" ";
        myLogFile <<tracking.area_<<" ";
        myLogFile <<tracking.dir<<" ";
        myLogFile <<label<<" ";
        myLogFile <<tracking.ID<<"\n";
    }
 
}


bool diskSpaceAvailable(std::string filename){
using namespace boost::filesystem;
space_info si = space(filename);
if(si.available/(1024*1024) >= 0){
    return true;
}
else return false;
}



int initVideoWriter(){
    int frame_width = camera.geometry().width(); //get the width of frames of the video
    int frame_height = camera.geometry().height(); //get the height of frames of the video
    
    cv::Size frame_size(frame_width, frame_height);
    
    //std::string filename = "output"+getTimeStr();
    //std::cout<<"Init video"<<std::endl;
    //Create and initialize the VideoWriter object 
    oVideoWriter.open(videoPath+getVideoFilename().c_str(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 
                                                               frames_per_second_video, frame_size, true);
    if (!oVideoWriter.isOpened()) {
        std::cerr << "Could not open the output video file for write\n";
        return -1;
    }
}

void writeVideoOutput(cv::Mat &frame){
    auto now  = std::chrono::high_resolution_clock::now();
    auto t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_video).count();
    //std::cout<<"Elapsed = "<<t_elapsed<<std::endl;
    if(t_elapsed>=timeVideo*1000){
    if (oVideoWriter.isOpened()) {
        oVideoWriter.release();
        stop_video_recording=true;
        time_stop_video = std::chrono::high_resolution_clock::now();
        std::cout<<"video written"<<std::endl;
    }
    initVideoWriter();
    
    } 
    oVideoWriter.write(frame);
}

//Compare event based on their timestamp
bool compareEvent(Prophesee::EventCD e1, Prophesee::EventCD e2)
{   
    return (e1.t < e2.t);
}


bool window_was_closed(const std::string &window_name) {
// Small hack: if the window has been closed, it is not visible anymore or property changes from the one we set
#if CV_MAJOR_VERSION >= 3 && CV_MINOR_VERSION >= 2
    if (cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) == 0) {
#else
    if (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) != 0) {
#endif
        return true;
    }
    return false;
}

//Callback for event acquisition and window setup
int setup_cd_callback_and_window(Prophesee::Camera &camera, cv::Mat &cd_frame,
                                 Prophesee::CDFrameGenerator &cd_frame_generator, const std::string &window_name) {
    auto &geometry = camera.geometry();
    auto id        = camera.cd().add_callback(
        [&cd_frame_generator](const Prophesee::EventCD *ev_begin, const Prophesee::EventCD *ev_end) {
  
            cd_frame_generator.add_events(ev_begin, ev_end);   
        });

    cd_frame_generator.start(
        FPS, [&cd_frame](const Prophesee::timestamp &ts, const cv::Mat &frame) { frame.copyTo(cd_frame); });
    if (gui){cv::namedWindow(window_name, CV_GUI_EXPANDED);
    cv::resizeWindow(window_name, geometry.width(), geometry.height());
    //cv::namedWindow(cd_window_name2, CV_GUI_EXPANDED);
    //cv::resizeWindow(cd_window_name2, geometry.width(), geometry.height());
    // move needs to be after resize on apple, otherwise the window stacks
    cv::moveWindow(window_name, 0, 0);
    cv::setMouseCallback(window_name,mouseCallBackFunc,NULL);}

    if(modelFree && gui){
        cv::createTrackbar( "Camera Setup:\t 0: 0° \t 1: 15° \t 2: 45° \t 3: 90°", window_name,
          &setup, max_setup,
          NULL); 

    cv::createTrackbar( "Erosion:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_name,
          &erosion_elem, max_elem,
          NULL );
    cv::createTrackbar( "Erosion Kernel size:\n 2n +1", window_name,
          &erosion_size, max_kernel_size,
          NULL );
    cv:: createTrackbar( "Dilation:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_name,
          &dilation_elem, max_elem,
          NULL);
    cv::createTrackbar( "Dilation Kernel size:\n 2n +1", window_name,
          &dilation_size, max_kernel_size,
          NULL);
  
    cv::createTrackbar( "BBox width Thresh:\n", window_name,
          &w_threshold, max_dim_size,
          NULL );
    cv::createTrackbar( "BBox height Thresh:\n", window_name,
          &h_threshold, max_dim_size,
          NULL );
    cv::createTrackbar( "BBox Area Thresh:\n", window_name,
          &a_threshold, max_dim_size,
          NULL );
 

    }
    if(gui){
    cv::createTrackbar( "Show masks: ", window_name,
          &value_btn_mask, 1,
          NULL , NULL); }
    return id;
}



//callback for mouse events
void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if(x>0 && x<640 && maskSetup){
        if  ( event == cv::EVENT_LBUTTONDOWN )
        {
            //std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
            int key = cv::waitKey(2000);
            //if( key == 'p') vPolygonPoint.push_back(cv::Point(x,y));
            if(key=='p') {
                int size = masks_.size();
                if(size != 0){
                    if (masks_[size-1].second==false){
                        masks_[size-1].first.push_back(cv::Point(x,y));
                    }else{
                        std::vector<cv::Point> vec(0);
                        std::pair<std::vector<cv::Point>,bool> mpair(vec,false);
                        masks_.push_back(mpair);
                        masks_[size].first.push_back(cv::Point(x,y)); 
                    }
                }else{
                    std::vector<cv::Point> vec(0);
                    std::pair<std::vector<cv::Point>,bool> mpair(vec,false);
                    masks_.push_back(mpair); 
                    masks_[size].first.push_back(cv::Point(x,y));
                }
 
                }else if(key=='v'){
                    int size = masks_.size();
                    if(size!=0){
                    masks_[size-1].first.push_back(cv::Point(x,y));
                    masks_[size-1].second=true;
                    }
                }

            x1_mouse = x;
            y1_mouse = y;
            left_clicked = true;
            btn_down = true;
            masks.clear();
            for (int i=0; i<masks_.size();i++){
            std::cout<<"mask size = "<<masks_.size()<<std::endl;
            if(masks_[i].second==true){
            std::vector<cv::Point> poly = masks_[i].first;
            /*for(int j=0; j<masks_[i].first.size();j++){
                cv::Point a(masks_[i].first[j].x,masks_[i].first[j].y);
                std::cout<<"mask x = "<<masks_[i].first[j].x<<std::endl;
                poly.push_back(a);
            }*/
            masks.push_back(poly);
            }
            
        }
         std::cout<<"final mask size = "<<masks.size()<<std::endl;
        }
        else if  ( event == cv::EVENT_RBUTTONDBLCLK )
        {
        std::vector<std::pair<std::vector<cv::Point>,bool>>::iterator it;
        bool inside = false;
        for (it = masks_.begin(); it != masks_.end(); ++it) {
            if (insidePolygon(it->first,cv::Point(x,y))){
                inside = true;
                break;
            }     
        }
        std::cout<<"Point ( "<<x<<", "<<y<<" ) inside polygone?: "<<std::to_string(inside)<<std::endl;

            
        }
        else if ( event == cv::EVENT_RBUTTONDOWN ){
            int size = masks_.size();
                if(size != 0){
                    if (masks_[size-1].second==false){
                        masks_.pop_back();
                    }
                }
       
        }


         cd_frame_generator.setMask_(masks);
        //writeFile();
        cv::FileStorage fs(maskfile, cv::FileStorage::WRITE);
        writeVectorOfVector(fs, "masks", masks_);
        fs.release();
    }else{
            left_clicked = false;
            mouseMoved = false;
    }
     
}

void bb_callback(int,void*){

}

//function for the tracking process. It also make transformation on the pseudo frame by capturing the change on the GUI.
void trackingProcess(int,void*){

    if(!modelFree){
        cv::Mat kernel2 = cv::getStructuringElement( cv::MORPH_RECT,
                       cv::Size( 1, 1 ),
                       cv::Point( 0, 0 ) );
        cv::dilate(out_img,out_img,kernel2); 
        auto start = std::chrono::high_resolution_clock::now();
        detectYolo(out_img);
        auto finish = std::chrono::high_resolution_clock::now();
        elapsed = finish - start;

    }else{
        int erosion_type = 0;
        if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
        else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
        else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

                
        cv::Mat kernel0 = cv::getStructuringElement( cv::MORPH_RECT,
                            cv::Size( 5, 5 ),
                            cv::Point( 1, 1) ); 

        cv::Mat kernel1 = cv::getStructuringElement( erosion_type,
                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       cv::Point( erosion_size, erosion_size ) );

        int dilation_type = 0;
        if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
        else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
        else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }
        cv::Mat kernel2 = cv::getStructuringElement( dilation_type,
                            cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                            cv::Point( dilation_size, dilation_size ) );
        cv::Ptr<cv::BackgroundSubtractor> bg; 
        bg = cv::createBackgroundSubtractorMOG2(500, 30, true);
        //bg = cv::createBackgroundSubtractorKNN(1,1,false);

        bg->apply(out_img,fore);
        
        cv::erode(fore,fore,cv::Mat());
        //cv::erode(fore,fore,kernel1);

        if (setup==0){
        option0();
        }
        else if(setup==1){
        option15(kernel1);
        }
        else if(setup==2){
        option45(kernel1);
        }
        else if(setup==3){
        option90();
        }
        
        //if(gui)cv::imshow( cd_window_name, fore);
        cv::dilate(fore,fore,kernel2);
  
        auto start = std::chrono::high_resolution_clock::now();
        
        auto img = detectAndTrackObject(fore);

        auto finish = std::chrono::high_resolution_clock::now();
        elapsed = finish - start;
        //std::cout<<"Processing takes "<<elapsed.count()<<" seconds"<<std::endl;




        //cv::imshow( cd_window_name,mergeFrame(cd_frame, img));//
        if(gui){
            std::ostringstream stats_objects;
            //stats_objects << std::fixed << std::setprecision(2);
            stats_objects << "#Cars: " << number_cars << ", #bikes: " << number_bikes<<", #Pedestrians: "<<number_pedestrians;
            auto stats_obj = stats_objects.str();
            int baseline2;
            auto stats_obj_bg_sz = cv::getTextSize(stats_obj.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline2);
            cv::rectangle(out_img, cv::Point(0, 30), cv::Point(stats_obj_bg_sz.width, stats_obj_bg_sz.height + 10), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(out_img, stats_obj.c_str(), cv::Point(0, stats_obj_bg_sz.height + 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
            std::ostringstream stats_alarm;
            stats_alarm<<"#Car Alarms: "<<number_cars_alarm<<", #Bike Alarms: "<<number_bikes_alarm;
            auto stats_alarm_obj = stats_alarm.str();
            int baseline3;
            auto stats_alarm_obj_bg_sz = cv::getTextSize(stats_alarm_obj.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline3);
            cv::rectangle(out_img, cv::Point(0, 90), cv::Point(stats_alarm_obj_bg_sz.width, stats_alarm_obj_bg_sz.height+ stats_obj_bg_sz.height + 35), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(out_img, stats_alarm_obj.c_str(), cv::Point(0, stats_alarm_obj_bg_sz.height + 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
            
            cv::imshow( cd_window_name, mergeFrame(out_img, img));
            }

        if(video){
            auto now  = std::chrono::high_resolution_clock::now();
            auto t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_stop_video).count();
            if(t_elapsed>=time_after_video*1000){
                if(stop_video_recording)last_time_video = std::chrono::high_resolution_clock::now();
                stop_video_recording=false;
                }
            if(!stop_video_recording)writeVideoOutput(out_img);
            }

        if(image){
            last_time_image = std::chrono::high_resolution_clock::now();

            auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(last_time_image - last_time_cnt);

            if(time_diff.count() >= timeImage*1000){
                cv::imwrite(imagePath,out_img);
                last_time_cnt = std::chrono::high_resolution_clock::now();
            }
        }

    }

        if(value_btn_mask==1){

            if(!maskSetup){

                drawMasks_(cd_frame);
            }else{
                drawMasks_w(cd_frame);
            }
      
        }

}


Detection boxToDetection(cv::Rect box, double conf, int classe){
    double cx = box.x + box.width/2;
    double cy = box.y + box.height/2;
    double width = box.width;
    double height = box.height;
    BoundingBox bb(cx, cy, width,height);
    //std::cout<<"Detection width, height "<<"( "+std::to_string(width)+" , "+std::to_string(height)+" )"<<std::endl;
    double confidence = conf;
    /* if(width <50 || height<50){
        confidence = 0.2;
    } */
    Detection detection(0, confidence, bb);

    return detection;

}
std::map<int, std::vector<Detection>> detectBoxes(std::vector<cv::Rect_<int>> (&boxes)[NUM_CLASSES], std::vector<float>(&scores)[NUM_CLASSES], std::vector<int>(&indices)[NUM_CLASSES], int frameId) {
  
    std::map<int, std::vector<Detection>> frameToDetections;
       for(int c=0; c<NUM_CLASSES; c++){ 
           
           for(auto idx:indices[c]) {
            //std::cout<<"classe "<<std::to_string(c)<<std::endl;
            //std::cout<<"index "<<std::to_string(idx)<<std::endl;
            if(scores[c][idx]>0.30){
            Detection frameDet = boxToDetection(boxes[c][idx], scores[c][idx], c);
            frameToDetections[c].push_back(frameDet);
            }
            
        }}
       
  
    return frameToDetections;
} 
std::map<int, std::vector<Detection>> detectBoxes(std::vector<cv::Rect> &boxes, int frameId) {
  
    std::map<int, std::vector<Detection>> frameToDetections;
        for(auto box:boxes) {
            Detection frameDet = boxToDetection(box,1,0);
            frameToDetections[frameId].push_back(frameDet);
        }
       
  
    return frameToDetections;
}


void detectAndTrack(std::vector<cv::Rect> &boxes, int frameId,cv::Mat &frame){
    std::vector<cv::Rect> boxesCar;
    std::vector<cv::Rect> boxesBikes;
    std::vector<cv::Rect> boxesPedestrian;
    if(boxes.size()>0){
            for (auto boxe : boxes){
                if (boxe.width>=min_width_pedestrian && boxe.width<=max_width_pedestrian && boxe.height <=max_height_pedestrian && boxe.height>=min_height_pedestrian){
                    boxesPedestrian.push_back(boxe);       
                }else if (boxe.width>min_width_bike && boxe.width<=max_width_bike && boxe.height <=max_height_bike && boxe.height>min_height_bike){
                    boxesBikes.push_back(boxe);
                }else if(boxe.width>=min_width_car && boxe.width<=max_width_car && boxe.height <=max_height_car && boxe.height>=min_height_car){
                     boxesCar.push_back(boxe);
                }
        }
    }
    std::map<int, std::vector<Detection>> detectionsCar = detectBoxes(boxesCar, frameId);
    std::map<int, std::vector<Detection>> detectionsBike = detectBoxes(boxesBikes, frameId);
    std::map<int, std::vector<Detection>> detectionsPedestrian = detectBoxes(boxesPedestrian, frameId);
    std::vector<Tracking> trackingsCar;
    std::vector<Tracking> trackingsBike;
    std::vector<Tracking> trackingsPedestrian;
    if(boxes.size()>0){

    auto startTime = std::chrono::high_resolution_clock::now();
    trackingsCar = trackerCar.track(detectionsCar[frameId],setup, true, algo_direction);
    trackingsBike = trackerBikes.track(detectionsBike[frameId],setup, true, algo_direction);
    trackingsPedestrian = trackerPedestrian.track(detectionsPedestrian[frameId],setup, true, algo_direction);
    auto endTime = std::chrono::high_resolution_clock::now();
    }
   drawTrackingsCar(trackingsCar, frame);
   drawTrackingsBikes(trackingsBike, frame);
   drawTrackingsPedestrian(trackingsPedestrian, frame);

} 

//check if point is inside polygon
bool insidePolygon(std::vector<cv::Point> &polyPoints,cv::Point p)
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


template<typename T>
void pop_front(std::vector<T>& vec)
{
    assert(!vec.empty());
    vec.erase(vec.begin());
}

//function to draw the bounding boxes for car
void drawTrackingsCar(std::vector<Tracking> &trackings,cv::Mat &frame){

 for(auto tracking : trackings){
     
     //std::cout<<"my track "<<tracking<<std::endl;
    if(tracking.flag == true){//tracking.flag == true

        if (!(std::find(ids_car.begin(), ids_car.end(), tracking.ID) != ids_car.end())) {
            if(ids_car.size()==10)pop_front(ids_car);
            ids_car.push_back(tracking.ID);
            number_cars++;
        }

        if(tracking.dir==alarm_value){
            if (!(std::find(ids_car_alarm.begin(), ids_car_alarm.end(), tracking.ID) != ids_car_alarm.end())) {
                if(ids_car_alarm.size()==10)pop_front(ids_car_alarm);
                ids_car_alarm.push_back(tracking.ID);
                number_cars_alarm++;
            }
        }

        cv::circle(frame, cv::Point(tracking.bb.cx, tracking.bb.cy), 6, cv::Scalar(0,255,0), -1);
        cv::putText(frame, "CAR "+std::to_string(tracking.ID), cv::Point(tracking.bb.cx-(tracking.bb.width/2) , tracking.bb.cy-(tracking.bb.height/2)-25),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,255), 1, cv::LINE_AA);
        cv::putText(out_img, "CAR "+std::to_string(tracking.ID), cv::Point(tracking.bb.cx-(tracking.bb.width/2) , tracking.bb.cy-(tracking.bb.height/2)-25),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,255), 1, cv::LINE_AA);
        
        cv::rectangle(frame, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), tracking.color,3);
        cv::rectangle(out_img, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), cv::Scalar(255,255,255)-tracking.color,3);
        
                                
        
            //std::cout << "velocity " << velos[i].x << " " << std::endl;
        if (tracking.dir==-1){
            cv::putText(frame, "RIGHT-LEFT", cv::Point(tracking.bb.x1() , tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);

            cv::putText(out_img, "RIGHT-LEFT", cv::Point(tracking.bb.x1() , tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
                                
        }else if(tracking.dir==1){
            cv::putText(frame, "LEFT-RIGHT", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "LEFT-RIGHT", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        }else if(tracking.dir==2){
            cv::putText(frame, "TOP-DOWN", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "TOP-DOWN", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        }else if(tracking.dir==-2){
            cv::putText(frame, "DOWN-TOP", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "DOWN-TOP", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        }
        //write system state in csv file
        write_csv(logfile, tracking, "CAR", std::chrono::system_clock::now());

    }else{
        //std::cout<<" flag false"<<std::endl;
       // cv::rectangle(frame, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()),cv::Scalar(0,0,0) ,3);
        //cv::rectangle(out_img, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), cv::Scalar(0,0,0),3);
        
    }
 }

}
//function to draw the bounding boxes for bikes
void drawTrackingsBikes(std::vector<Tracking> &trackings,cv::Mat &frame){

 for(auto tracking : trackings){
  
    if(tracking.flag == true){//tracking.flag == true
        if (!(std::find(ids_bikes.begin(), ids_bikes.end(), tracking.ID) != ids_bikes.end())) {
            if(ids_bikes.size()==10)pop_front(ids_bikes);
            ids_bikes.push_back(tracking.ID);
            number_bikes++;
        }

        if(tracking.dir==alarm_value){
            if (!(std::find(ids_bikes_alarm.begin(), ids_bikes_alarm.end(), tracking.ID) != ids_bikes_alarm.end())) {
                if(ids_bikes_alarm.size()==10)pop_front(ids_bikes_alarm);
                ids_bikes_alarm.push_back(tracking.ID);
                number_bikes_alarm++;
            }
        }
        cv::circle(frame, cv::Point(tracking.bb.cx, tracking.bb.cy), 6, cv::Scalar(0,255,0), -1);
        cv::putText(frame, "BIKE "+std::to_string(tracking.ID), cv::Point(tracking.bb.cx-(tracking.bb.width/2) , tracking.bb.cy-(tracking.bb.height/2)-25),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,255), 1, cv::LINE_AA);
        cv::putText(out_img, "BIKE "+std::to_string(tracking.ID), cv::Point(tracking.bb.cx-(tracking.bb.width/2) , tracking.bb.cy-(tracking.bb.height/2)-25),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,255), 1, cv::LINE_AA);
        
        cv::rectangle(frame, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), tracking.color,3);
        cv::rectangle(out_img, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), cv::Scalar(255,255,255)-tracking.color,3);                        
        
            //std::cout << "velocity " << velos[i].x << " " << std::endl;
        if (tracking.dir==-1){
            cv::putText(frame, "RIGHT-LEFT", cv::Point(tracking.bb.x1() , tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);

            cv::putText(out_img, "RIGHT-LEFT", cv::Point(tracking.bb.x1() , tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
                                
        }else if(tracking.dir==1){
            cv::putText(frame, "LEFT-RIGHT", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "LEFT-RIGHT", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        }else if(tracking.dir==2){
            cv::putText(frame, "TOP-DOWN", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "TOP-DOWN", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        }else if(tracking.dir==-2){
            cv::putText(frame, "DOWN-TOP", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "DOWN-TOP", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        } 
        //write system state in csv file
         write_csv(logfile, tracking, "BIKE", std::chrono::system_clock::now());  
    }else{
        //std::cout<<" flag false"<<std::endl;
        //cv::rectangle(frame, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()),cv::Scalar(0,0,0) ,3);
       // cv::rectangle(out_img, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), cv::Scalar(0,0,0),3);
        
    }
 }

}
//function to draw the bounding boxes for pedestrians
void drawTrackingsPedestrian(std::vector<Tracking> &trackings,cv::Mat &frame){

 for(auto tracking : trackings){

    if(tracking.flag == true){//tracking.flag == true
        if (!(std::find(ids_pedestrians.begin(), ids_pedestrians.end(), tracking.ID) != ids_pedestrians.end())) {
            if(ids_pedestrians.size()==10)pop_front(ids_pedestrians);
            ids_pedestrians.push_back(tracking.ID);
            number_pedestrians++;
        }

/*         if(tracking.dir==alarm_value){
            if (!(std::find(ids_pedestrians_alarm.begin(), ids_pedestrians_alarm.end(), tracking.ID) != ids_pedestrians_alarm.end())) {
                if(ids_pedestrians_alarm.size()==10)pop_front(ids_pedestrians_alarm);
                ids_pedestrians_alarm.push_back(tracking.ID);
                number_pedestrians_alarm++;
            }
        } */
        cv::circle(frame, cv::Point(tracking.bb.cx, tracking.bb.cy), 6, cv::Scalar(0,255,0), -1);
        cv::putText(frame, "Pedestrian"+std::to_string(tracking.ID), cv::Point(tracking.bb.cx-(tracking.bb.width/2) , tracking.bb.cy-(tracking.bb.height/2)-25),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,0), 1, cv::LINE_AA);
         cv::putText(out_img, "Pedestrian"+std::to_string(tracking.ID), cv::Point(tracking.bb.cx-(tracking.bb.width/2) , tracking.bb.cy-(tracking.bb.height/2)-25),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,0), 1, cv::LINE_AA);
        
        cv::rectangle(frame, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), tracking.color,3);
        cv::rectangle(out_img, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), cv::Scalar(255,255,255)-tracking.color,3);
        
                                
        
            //std::cout << "velocity " << velos[i].x << " " << std::endl;
        if (tracking.dir==-1){
            cv::putText(frame, "RIGHT-LEFT", cv::Point(tracking.bb.x1() , tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);

            cv::putText(out_img, "RIGHT-LEFT", cv::Point(tracking.bb.x1() , tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
                                
        }else if(tracking.dir==1){
            cv::putText(frame, "LEFT-RIGHT", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "LEFT-RIGHT", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        }else if(tracking.dir==2){
            cv::putText(frame, "TOP-DOWN", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "TOP-DOWN", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        }else if(tracking.dir==-2){
            cv::putText(frame, "DOWN-TOP", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, tracking.color, 1, cv::LINE_AA);
            cv::putText(out_img, "DOWN-TOP", cv::Point(tracking.bb.x1(), tracking.bb.y1()-10),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255)-tracking.color, 1, cv::LINE_AA);
        }
         //write system state in csv file
         write_csv(logfile, tracking, "PEDESTRIAN", std::chrono::system_clock::now());   
    }else{
        //std::cout<<" flag false"<<std::endl;
        //cv::rectangle(frame, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()),cv::Scalar(0,0,0) ,3);
        //cv::rectangle(out_img, cv::Point(tracking.bb.x1(), tracking.bb.y1()), cv::Point(tracking.bb.x2(), tracking.bb.y2()), cv::Scalar(0,0,0),3);
        
    }
 }

}

//Filtering option for 0° setup
void option0(){
      
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  

    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    //cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    //cv::dilate(fore,fore,cv::Mat());  
    //cv::dilate(fore,fore,cv::Mat());  
    //cv::dilate(fore,fore,cv::Mat());  
    //cv::dilate(fore,fore,cv::Mat());
     

   cv::normalize(~fore, fore, 0, 255, cv::NORM_MINMAX,-1, cv::noArray());  
    cv::threshold(fore, fore, 0, 255, CV_THRESH_BINARY); 
    cv::dilate(fore,fore,cv::Mat()); 
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    // cv::dilate(fore,fore,cv::Mat()); 
    // cv::dilate(fore,fore,cv::Mat());  
    // cv::dilate(fore,fore,cv::Mat());  


}

//Filtering option for 45° setup
void option45(cv::Mat kernel1){
      
    cv::dilate(fore,fore,cv::Mat()); 
    cv::erode(fore,fore,kernel1); 
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat()); 
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());
     

    cv::normalize(~fore, fore, 0, 255, cv::NORM_MINMAX,-1, cv::noArray());  
    cv::threshold(fore, fore, 0, 255, CV_THRESH_BINARY);  

}

//Filtering option for 15° setup
void option15(cv::Mat kernel1){
      
    //cv::dilate(fore,fore,cv::Mat());
    //cv::erode(fore,fore,cv::Mat());
    cv::GaussianBlur( fore, fore, cv::Size( 3, 3 ), 0, 0 );
     cv::erode(fore,fore,cv::Mat());
    cv::erode(fore,fore,kernel1);
    cv::dilate(fore,fore,cv::Mat()); 
    cv::erode(fore,fore,cv::Mat()); 
    cv::dilate(fore,fore,cv::Mat());
    cv::erode(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());
    cv::erode(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat()); 
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    //cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    //cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    //cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());   
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());
     

    cv::normalize(~fore, fore, 0, 255, cv::NORM_MINMAX,-1, cv::noArray());  
    cv::threshold(fore, fore, 0, 255, CV_THRESH_BINARY);  

}

//Filtering option for 90° setup
void option90(){
      
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat()); 
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_CLOSE, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    cv::morphologyEx(fore,fore, cv::MORPH_OPEN, cv::Mat()) ;
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat());
    cv::dilate(fore,fore,cv::Mat());  
    cv::dilate(fore,fore,cv::Mat()); 

    cv::normalize(~fore, fore, 0, 255, cv::NORM_MINMAX,-1, cv::noArray());  
    cv::threshold(fore, fore, 0, 255, CV_THRESH_BINARY);  

}

//function to merge the frames
cv::Mat mergeFrame(cv::Mat &img_orig, cv::Mat &img_track){

  
    cv::Mat matDst(cv::Size(img_orig.cols*2,img_orig.rows),img_track.type(),cv::Scalar::all(0));
    cv::Mat matRoi = matDst(cv::Rect(0,0,img_orig.cols,img_orig.rows));
    img_orig.copyTo(matRoi);
    matRoi = matDst(cv::Rect(img_orig.cols,0,img_orig.cols,img_track.rows));
    img_track.copyTo(matRoi);

    cv::Rect border(cv::Point(0, 0), img_orig.size());
    cv::Rect border2(cv::Point(img_orig.cols+1, 0), img_track.size());
    cv::Scalar color(255, 0, 255);
    cv::Scalar color2(0, 255,255);
    int thickness = 2;

    cv::rectangle(matDst, border, color, thickness);
    cv::rectangle(matDst, border2, color2, thickness);
    cv::putText(matDst, "ORIGINAL", cv::Point(20, 20),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, color, 1, cv::LINE_AA);
    cv::putText(matDst, "TRACKING", cv::Point(img_orig.cols+20, 20),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, color2, 1, cv::LINE_AA);
    
    return matDst;

}

//Function to extract bounding boxes
cv::Mat detectAndTrackObject(cv::Mat &mat){
    cv::Mat out;  
    mat.copyTo(img);

    //cv::cvtColor(img,out,cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Rect> selectedRect;
    std::vector<cv::Rect> mergedRect;
    std::vector<int> selected;

    cv::Mat drawing = cv::Mat::zeros( img.size(), CV_8UC3 );
    // filter contours according to their bounding box size
    for( std::size_t i = 0; i < contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rangeColor.uniform(0, 256), rangeColor.uniform(0,256), rangeColor.uniform(0,256) );
        cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
        boundRect[i] = cv::boundingRect( contours_poly[i] );
        if (boundRect[i].width > w_threshold && boundRect[i].height > h_threshold && (boundRect[i].width*boundRect[i].height) > a_threshold)
        {
            //selected.push_back(i);
            selectedRect.push_back(boundRect[i]);
            //std::cout<<"x : "<<std::to_string(boundRect[i].x)<<" y : "<<std::to_string(boundRect[i].y)<<" width: "<<std::to_string(boundRect[i].width)<<" height: "<<std::to_string(boundRect[i].height)<<std::endl;
            cv::drawContours(drawing, contours_poly, (int)i, color, 2, 8);
        }
        
    }

    discardInsiderBoxes(selectedRect);
    //std::cout<<selectedRect.size()<<std::endl;
    mergeOverlappingBoxes(selectedRect, drawing, mergedRect);

    detectAndTrack(selectedRect, frameID, drawing);
    return drawing;

}

//Discard the inner BB
void discardInsiderBoxes(std::vector<cv::Rect> &boxes){
    cv::Rect a;
    cv::Rect b;
    std::set<int,std::greater<int>> ins;
   // std::set<int> boxs;

        for( std::size_t i = 0; i< boxes.size(); i++)
    {
        a = boxes[i];
        //boxs.insert(i);
        
        for( std::size_t j = i+1; j< boxes.size(); j++)
    {   
        b = boxes[j];

        if((a & b) == a){
           ins.insert(i);
        }else if((a & b) == b){
             ins.insert(j);
        }
    }
}
    for (auto index: ins) {
    boxes[index] = boxes.back();
    boxes.pop_back();
    }

}

//Merge overlapping BB
void mergeOverlappingBoxes(const std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1); // Mask of original image
    cv::Size scaleFactor(30,5); // To expand rectangles, i.e. increase sensitivity to nearby rectangles. Doesn't have to be (10,10)--can be anything
    for (int i = 0; i < inputBoxes.size(); i++)
    {
        cv::Rect box = inputBoxes.at(i) + scaleFactor;
        cv::rectangle(mask, box, cv::Scalar(255), CV_FILLED); // Draw filled bounding boxes on mask
    }

    std::vector<std::vector<cv::Point>> contours;
    // Find contours in mask
    // If bounding boxes overlap, they will be joined by this function call
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int j = 0; j < contours.size(); j++)
    {
        outputBoxes.push_back(cv::boundingRect(contours.at(j)));
    }
}


//Draw the masks on the frame
void drawMasks_(cv::Mat &frm){
    
    int size = masks.size();
    //std::cout<<"Mask draw "<<size<<std::endl;
    if (size!=0){
           cv::circle(frm, masks[size-1][0], 4, cv::Scalar(0, 0, 255), -1);         
    }
    
    std::vector<std::vector<cv::Point>>::iterator it;
   
        for (it = masks.begin(); it != masks.end(); ++it) {
            polylines(frm, *it, true, cv::Scalar(0, 0, 255), 2, 8);         
    }
}
//Draw the mask when mask setup option
void drawMasks_w(cv::Mat &frm){
    
    int size = masks_.size();
    if (size!=0){
           cv::circle(frm, masks_[size-1].first[0], 4, cv::Scalar(0, 0, 255), -1);         
    }
    
    std::vector<std::pair<std::vector<cv::Point>,bool>>::iterator it;
   
        for (it = masks_.begin(); it != masks_.end(); ++it) {
            polylines(frm, it->first, it->second, cv::Scalar(0, 0, 255), 2, 8);         
    }
}

//Function to perform detection with the YOLO model
void detectYolo(cv::Mat &frame_){
 
    cv::Mat blob;
    std::vector<cv::Mat> detections;
  
        auto start = std::chrono::high_resolution_clock::now();
  

        auto total_start = std::chrono::steady_clock::now();
        cv::dnn::blobFromImage(frame_, blob, 0.00392, cv::Size(416, 416), cv::Scalar(), true, false, CV_32F);//0.00392
        net.setInput(blob);

        auto dnn_start = std::chrono::steady_clock::now();
        net.forward(detections, output_names);
        auto dnn_end = std::chrono::steady_clock::now();
       
        std::vector<int> indices[NUM_CLASSES];
        std::vector<cv::Rect> boxes[NUM_CLASSES];
        std::vector<float> scores[NUM_CLASSES];
        
        for (auto& output : detections)
        {
            const auto num_boxes = output.rows;
            for (int i = 0; i < num_boxes; i++)
            {
                auto x = output.at<float>(i, 0) * frame_.cols;
                auto y = output.at<float>(i, 1) * frame_.rows;
                auto width = output.at<float>(i, 2) * frame_.cols;
                auto height = output.at<float>(i, 3) * frame_.rows;
                cv::Rect rect(x - width/2, y - height/2, width, height);

                for (int c = 0; c < NUM_CLASSES; c++)
                {
                    auto confidence = *output.ptr<float>(i, 5 + c);
                    if (confidence >= CONFIDENCE_THRESHOLD)
                    {
                        //std::cout<<"classe "<<std::to_string(c)<<std::endl;
                        boxes[c].push_back(rect);
                        scores[c].push_back(confidence);
                    }
                }
            }
        }
        std::vector<cv::Rect> _boxes[NUM_CLASSES];
        std::vector<float> _scores[NUM_CLASSES];
        std::vector<int> _indices[NUM_CLASSES];
        for (int c = 0; c < NUM_CLASSES; c++){
            
            cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);
           
                _boxes[c] = boxes[c];
                _scores[c] = scores[c]; 
                _indices[c] = indices[c];
           
            }
        
        _detections = detectBoxes(_boxes, _scores, _indices, frameID); 

        std::vector<Tracking> trackings;
        std::vector<Tracking> trackingsBikes;
        std::vector<Tracking> trackingsPedestrians;
        if(_detections.size()>0){
        auto startTime = std::chrono::high_resolution_clock::now();
        
        if(_detections[0].size())
        trackings = trackerCar.track(_detections[0],setup,false, algo_direction);
         if(_detections[1].size())
        trackingsBikes = trackerBikes.track(_detections[1],setup,false, algo_direction);
         if(_detections[2].size())
        trackingsPedestrians = trackerPedestrian.track(_detections[2],setup,false, algo_direction);
        auto endTime = std::chrono::high_resolution_clock::now();
    }
        drawTrackingsCar(trackings, frame_);
        drawTrackingsBikes(trackingsBikes, frame_);
        drawTrackingsPedestrian(trackingsPedestrians, frame_);

        auto finish = std::chrono::high_resolution_clock::now();
        elapsed = finish - start;
     
        auto total_end = std::chrono::steady_clock::now();

        float inference_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(dnn_end - dnn_start).count();
        float total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
        std::ostringstream stats_ss;
        stats_ss << std::fixed << std::setprecision(2);
        stats_ss << "Inference FPS: " << inference_fps << ", Total FPS: " << total_fps;
        auto stats = stats_ss.str();
        //std::cout << stats<< std::endl;   
        int baseline;
        auto stats_bg_sz = cv::getTextSize(stats.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
        cv::rectangle(frame_, cv::Point(0, 0), cv::Point(stats_bg_sz.width, stats_bg_sz.height + 10), cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame_, stats.c_str(), cv::Point(0, stats_bg_sz.height + 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
        

        std::ostringstream stats_objects;
        //stats_objects << std::fixed << std::setprecision(2);
        stats_objects << "#Cars: " << number_cars<< ", #bikes: " << number_bikes<<", #Pedestrians: "<<number_pedestrians;
        auto stats_obj = stats_objects.str();
        int baseline2;
        auto stats_obj_bg_sz = cv::getTextSize(stats_obj.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline2);
        cv::rectangle(frame_, cv::Point(0, 30), cv::Point(stats_obj_bg_sz.width, stats_obj_bg_sz.height + 10), cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame_, stats_obj.c_str(), cv::Point(0, stats_obj_bg_sz.height + 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
        std::ostringstream stats_alarm;
        stats_alarm<<"#Car Alarms: "<<number_cars_alarm<<", #Bike Alarms: "<<number_bikes_alarm;
        auto stats_alarm_obj = stats_alarm.str();
        int baseline3;
        auto stats_alarm_obj_bg_sz = cv::getTextSize(stats_alarm_obj.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline3);
        cv::rectangle(frame_, cv::Point(0, 90), cv::Point(stats_alarm_obj_bg_sz.width, stats_alarm_obj_bg_sz.height+ stats_obj_bg_sz.height + 35), cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame_, stats_alarm_obj.c_str(), cv::Point(0, stats_alarm_obj_bg_sz.height + 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
        
        if(gui)cv::imshow( cd_window_name,frame_);//mergeFrame(,out_img,frame_)

        if(video){
            auto now  = std::chrono::high_resolution_clock::now();
            auto t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_stop_video).count();
            if(t_elapsed>=time_after_video*1000){
                if(stop_video_recording)last_time_video = std::chrono::high_resolution_clock::now();
                stop_video_recording=false;
                }
            if(!stop_video_recording)writeVideoOutput(out_img);
            }

        if(image){
            last_time_image = std::chrono::high_resolution_clock::now();

            auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(last_time_image - last_time_cnt);

            if(time_diff.count() >= timeImage*1000){
                cv::imwrite(imagePath,frame_);
                last_time_cnt = std::chrono::high_resolution_clock::now();
            }
        }
}


void modelLoader(){

    ///home/abednego/Thesis/new/yolo-tiny
    net = cv::dnn::readNetFromDarknet("../models/model.cfg", "../models/model.weights");
    
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);// DNN_BACKEND_CUDA DNN_BACKEND_OPENCV DNN_BACKEND_INFERENCE_ENGINE
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);//DNN_TARGET_CUDA DNN_TARGET_CPU DNN_TARGET_MYRIAD
    output_names = net.getUnconnectedOutLayersNames();
}


void configParser(std::string configFile){
    pt::ptree loadPtreeRoot;
    std::string erosion_element;
    std::string dilation_element;
    int camera_setup;

    pt::read_json(configFile, loadPtreeRoot);    
    maskfile = loadPtreeRoot.get_child("masks").get_value<std::string>();
    gui = loadPtreeRoot.get_child("gui").get_value<bool>();
    modelFree = !loadPtreeRoot.get_child("model").get_value<bool>();
    logfile = loadPtreeRoot.get_child("logfile").get_value<std::string>();
    videoPath = loadPtreeRoot.get_child("video_output").get_child("path").get_value<std::string>();
    timeVideo = loadPtreeRoot.get_child("video_output").get_child("time").get_value<int>();
    imagePath = loadPtreeRoot.get_child("image_output").get_child("path").get_value<std::string>();
    timeImage =  loadPtreeRoot.get_child("image_output").get_child("time").get_value<double>();
    filename = loadPtreeRoot.get_child("filename").get_value<std::string>();
    biases_file = loadPtreeRoot.get_child("biases").get_value<std::string>();
    serial = loadPtreeRoot.get_child("serial").get_value<std::string>();
    alarmType = loadPtreeRoot.get_child("alarm_type").get_value<std::string>();
    erosion_element = loadPtreeRoot.get_child("model_free").get_child("erosion_type").get_value<std::string>();
    erosion_size = loadPtreeRoot.get_child("model_free").get_child("erosion_kernel_size").get_value<int>();
    dilation_element = loadPtreeRoot.get_child("model_free").get_child("dilation_type").get_value<std::string>();
    dilation_size = loadPtreeRoot.get_child("model_free").get_child("dilation_kernel_size").get_value<int>();
    w_threshold = loadPtreeRoot.get_child("model_free").get_child("bbox_width_threshold").get_value<int>();
    h_threshold = loadPtreeRoot.get_child("model_free").get_child("bbox_height_threshold").get_value<int>();
    a_threshold = loadPtreeRoot.get_child("model_free").get_child("bbox_area_threshold").get_value<int>();
    a_threshold = loadPtreeRoot.get_child("model_free").get_child("bbox_area_threshold").get_value<int>();
    camera_setup = loadPtreeRoot.get_child("camera_setup").get_value<int>();
    max_width_pedestrian = loadPtreeRoot.get_child("model_free").get_child("pedestrian").get_child("max_width").get_value<int>();
    max_height_pedestrian = loadPtreeRoot.get_child("model_free").get_child("pedestrian").get_child("max_height").get_value<int>();
    max_width_bike = loadPtreeRoot.get_child("model_free").get_child("bike").get_child("max_width").get_value<int>();
    max_height_bike = loadPtreeRoot.get_child("model_free").get_child("bike").get_child("max_height").get_value<int>();
    min_width_pedestrian = loadPtreeRoot.get_child("model_free").get_child("pedestrian").get_child("min_width").get_value<int>();
    min_height_pedestrian = loadPtreeRoot.get_child("model_free").get_child("pedestrian").get_child("min_height").get_value<int>();
    min_width_bike = loadPtreeRoot.get_child("model_free").get_child("bike").get_child("min_width").get_value<int>();
    min_height_bike = loadPtreeRoot.get_child("model_free").get_child("bike").get_child("min_height").get_value<int>();
    min_width_car = loadPtreeRoot.get_child("model_free").get_child("car").get_child("min_width").get_value<int>();
    min_height_car = loadPtreeRoot.get_child("model_free").get_child("car").get_child("min_height").get_value<int>();
    detect_right = loadPtreeRoot.get_child("model_free").get_child("detect_area_right").get_value<int>();
    detect_left = loadPtreeRoot.get_child("model_free").get_child("detect_area_left").get_value<int>();
    detect_top = loadPtreeRoot.get_child("model_free").get_child("detect_area_top").get_value<int>();
    detect_bottom = loadPtreeRoot.get_child("model_free").get_child("detect_area_bottom").get_value<int>();
    time_after_video = loadPtreeRoot.get_child("video_output").get_child("record_after").get_value<int>();
    algo_direction = loadPtreeRoot.get_child("algo_direction").get_value<int>();

    
    trackerCar.setDetectAreaLeft(detect_left);
    trackerCar.setDetectAreaRight(detect_right);
    trackerCar.setDetectAreaTop(detect_top);
    trackerCar.setDetectAreaBottom(detect_bottom);
    trackerBikes.setDetectAreaLeft(detect_left);
    trackerBikes.setDetectAreaRight(detect_right);
    trackerBikes.setDetectAreaTop(detect_top);
    trackerBikes.setDetectAreaBottom(detect_bottom);
    trackerPedestrian.setDetectAreaLeft(detect_left);
    trackerPedestrian.setDetectAreaRight(detect_right);
    trackerPedestrian.setDetectAreaTop(detect_top);
    trackerPedestrian.setDetectAreaBottom(detect_bottom);

    if(logfile=="") logfile = "log.csv";
    if(videoPath=="") video = false;
    if(imagePath=="") image = false;
    if(alarmType=="none" || alarmType=="None") {
        is_alarm_set = false;
        alarm_value = 0;
    }else if(alarmType=="RIGHT-LEFT"){
        is_alarm_set = true;
        alarm_value=-1;

    }else if(alarmType=="LEFT-RIGHT"){
        is_alarm_set = true;
         alarm_value=1;

    }else if(alarmType=="UP-DOWN"){
        is_alarm_set = true;
         alarm_value=2;

    }else if(alarmType=="DOWN-UP"){
        is_alarm_set = true;
         alarm_value=-2;
    }
    if (camera_setup == 0 || camera_setup == 180){
        setup=0;
    }else if (camera_setup == 15){
        setup=1;
    }else if(camera_setup == 45){
        setup=2;
    }else {
        setup=3;
    }

    if(dilation_element=="RECT"){
        dilation_elem=0;
    }else if (dilation_element=="CROSS"){
        dilation_elem=1;
    }else if (dilation_element=="ELLIPSE"){
        dilation_elem=2;
    }

    if(erosion_element=="RECT"){
        erosion_elem=0;
    }else if (erosion_element=="CROSS"){
        erosion_elem=1;
    }else if (erosion_element=="ELLIPSE"){
        erosion_elem=2;
    }

    if(imagePath!="" && timeImage >0){
        image = true;
    }

    if(videoPath!="" && timeVideo >0){
        video = true;
    }


}

int main(int argc, char *argv[]) {
   
    std::vector<uint16_t> roi;
    std::string maskfileSetup;
    std::string configFile;


    uint32_t max_rate_kEV_s = 10000;

    bool do_retry = false;

    modelLoader();

    const std::string program_desc =
        "\nThis application track objects (cars, bikes, pedestrian) in a visual scene from a rawfile or camera device.\n";
    const std::string exec_line = "\n./vehicle_tracker [-f <input_file> [-b <bias_file>]] -o logfile_name [-v <video_folder> -t video_time] [-m mask_file] [-gui] [-mf] [-h] [-u <mask filename>]\n";
    po::options_description desc(program_desc + exec_line + "\nAllowed options");

    // clang-format off
    desc.add_options()
        ("help,h", "Print this help message")
		("serial,s", po::value<std::string>(&serial)->default_value(""),"Serial ID of the camera. Only while using the device not the 'filename'")
		("filename,f", po::value<std::string>(&filename)->default_value(""), "Path to a rawfile to read. If this option is not specified, the serial camera is used. If the camera is not plugged in the program, throw an exception.")
		("biases,b", po::value<std::string>(&biases_file)->default_value(""), "Path to a biases file. Otherwise the default biases are used.")
		("roi,r", po::value<std::vector<uint16_t>>(&roi)->multitoken(), "Hardware roi to set on the sensor in the format [x y width height]")
        ("output,o", po::value<std::string>(&logfile)->default_value("log.csv"), "The log file path. If the file exists, the program appends data; otherwise, it creates a new file. If this option is not provided, the program creates a default log file in the executable location folder with a name log.csv.")
        ("video,v", po::value<std::string>(&videoPath)->default_value(""), "Path of the folder where to save the output video. The video output name follows this pattern: output_year_month_date_hh:mm:ss.avi; in that way, each video has a unique name.If this option is specified, the option -t must be used to provide the number of seconds for each video recording.")
        ("timeVideo,t", po::value<int>(&timeVideo)->default_value(10), "Time in second of one video.")
        ("image,i", po::value<std::string>(&imagePath)->default_value(""), "Path of the folder where to save the image output.")
        ("timeImage,j", po::value<double>(&timeImage)->default_value(30), "Time in second after which the image should be saved.")
        ("mask,m", po::value<std::string>(&maskfile)->default_value(""), "Path to a file with masks to read otherwise no mask is used. The mask file should be a validn.yaml file generated the mask generator program; otherwise, the software will crash.")
        ("gui,gui","Option to enable the GUI to see what is going on in real-time.")
        ("model-free,d",  "to run the software in the model-free mode. In this case, the software does not use the trained model. If this option is not specified, the software loads the trained model located in the models folder.")
        ("mask-setup,u", po::value<std::string>(&maskfileSetup)->default_value(""), "Option to draw the mask and save it in the specified mask filename.")
        ("config,c", po::value<std::string>(&configFile)->default_value(""), "Config file name.");

    // clang-format on
    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

        po::notify(vm);
    } catch (...) {
        std::cout << desc << std::endl;
        return 1;
    }
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    if (vm.count("roi")) {
        if (roi.size() != 4) {
            std::cerr << "Warning: ROI as argument must be in the format 'x y width height '. Roi has not been set."
                      << std::endl;
            roi.clear();
        }
    }
    if (vm.count("gui")){
        gui = true;
         std::cout<<"GUI TRUE"<<std::endl;
    }

    if (vm.count("model-free")){
        modelFree = true;
        std::cout<<"MF TRUE"<<std::endl;
    }

    if (vm.count("mask-setup") && maskfileSetup!=""){
        maskSetup= true;
        std::cout<<"MS TRUE"<<std::endl;
        maskfile = maskfileSetup;

    }

    if (vm.count("video") && videoPath!=""){
        video= true;
    }

    if (vm.count("image") && imagePath!=""){
        image = true;
    }

    if (vm.count("config") && configFile!=""){
        auto start = std::chrono::high_resolution_clock::now();
        configParser(configFile);
        auto finish = std::chrono::high_resolution_clock::now();
        elapsed = finish - start;
        std::cout<<"Parsing takes "<<elapsed.count()<<" seconds"<<std::endl;

    }
    
    std::cout << program_desc << std::endl;

    do {

        bool camera_is_opened = false;

        // If the filename is set, then read from the file
        if (filename != "") {
            if (!serial.empty()) {
                std::cerr << "Error: flag --serial and --filename are not compatible." << std::endl;
                return 1;
            }

            try {
                camera           = Prophesee::Camera::from_file(filename);

                camera_is_opened = true;
            } catch (Prophesee::CameraException &e) { std::cerr << e.what() << std::endl; }
            // Otherwise, set the input source to the fist available camera
        } else {
            try {
                if (!serial.empty()) {
                    camera = Prophesee::Camera::from_serial(serial);
                } else {
                    camera = Prophesee::Camera::from_first_available();
                }

                if (biases_file != "") {
                    camera.biases().set_from_file(biases_file);
                }
                if (!roi.empty()) {
                    std::cout<<"ROI SET"<<std::endl;
                    camera.roi().set({roi[0], roi[1], roi[2], roi[3]});
                }
                camera_is_opened = true;
            } catch (Prophesee::CameraException &e) { std::cerr << e.what() << std::endl; }
        }

        if (!camera_is_opened) {
            if (do_retry) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                std::cout << "Trying to reopen camera..." << std::endl;
                continue;
            } else {
                return -1;
            }
        } else {
            std::cout << "Camera has been opened successfully." << std::endl;
        }

        // Add runtime error callback
        camera.add_runtime_error_callback([&do_retry](const Prophesee::CameraException &e) {
            std::cerr << e.what() << std::endl;
            do_retry = true;
        });

        // Get the geometry of the camera
        auto &geometry = camera.geometry(); // Get the geometry of the camera

        // All Prophesee cameras have at least CDs
        const bool cd_available = true;

        bool cd_frame_activated = cd_available;
        // Set the colors of the pseudo-frame
        const cv::Scalar background_color = cv::Scalar( 0,0, 0);
        const cv::Scalar on_color = cv::Scalar( 255, 255, 255);
        const cv::Scalar off_color = cv::Scalar( 255, 255, 255 );
        bool colored = true;
        //Initialize the frame generator parameters
        cd_frame_generator.setGeometry(geometry.width(), geometry.height(),true);
        cd_frame_generator.set_colors(background_color, on_color, off_color, colored);
        cd_frame_generator.set_display_accumulation_time_us(22000);
        
        //if the video option is provided initialise the videoWriter
        if(video){
            initVideoWriter();
        }
        //if the mask file is provided and the mask setup option is not provided, set the mask of the frame generator.
        if(maskfile!="" && !maskSetup){
            if(maskfile.substr(maskfile.length()-3,maskfile.length())!="yml" && maskfile.substr(maskfile.length()-4,maskfile.length())!="yaml" ){
                std::cerr<<"Unsupported Mask file extension.\nMask file should have .yml or .yaml extension"<<std::endl;
                return -1;
            }    
            cv::FileStorage fs2(maskfile, cv::FileStorage::READ);
        
            readVectorOfVector(fs2,"masks",masks); //reading the mask file. See function implementation of the function above
            fs2.release();
            //std::cout<<"size of maks : "<<masks_.size()<<std::endl;
            cd_frame_generator.setMask_(masks);
        }else {
            std::cout<<"No mask file provided, all the regions of the frame are considered"<<std::endl;
        }
        //set the callback to capture the events
        int cd_events_cb_id = setup_cd_callback_and_window(camera, cd_frame, cd_frame_generator, cd_window_name);

        // Start the camera streaming
        camera.start();

        bool recording             = false;
        bool is_roi_set            = true;
        bool max_event_rate_active = false;
        
        last_time_cnt = std::chrono::high_resolution_clock::now();

        while (camera.is_running()) {
            // Make sure that at least one window is always opened
            camera_running = true;
            if (!cd_frame_activated ) {
                cd_frame_activated = true;
                cd_events_cb_id    = setup_cd_callback_and_window(camera, cd_frame, cd_frame_generator, cd_window_name);
            }

            if (configFile!=""){
                configParser(configFile);
            }

            if (cd_frame_activated && !cd_frame.empty()){
                ++frameID;
                cd_frame.copyTo(out_img); 
                trackingProcess(0,0);
                
            }
            //make calculation to display at a specific frame rate
            int tempo = 30 - (int)(elapsed.count()*1000);
            if(tempo<=0)tempo = 5;
            //std::cout<<"tempo = "<<tempo<<std::endl;
            //that means that the display is refreshed at 30 FPS
            
                process_ui_for(tempo);   
            
 
        }
        camera_running = false;
        // unregister callbacks to make sure they are not called anymore
        if (cd_events_cb_id >= 0) {
            camera.cd().remove_callback(cd_events_cb_id);
        }
        // Stop the camera streaming

        camera.stop();
    } while (do_retry);

    //if logfile is opened close it
    if(myLogFile.is_open()){
        myLogFile.close();
    }
    return 0;
}