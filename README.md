# thesis_event_based_camera
This repository contains the source code of the thesis "**Event-based camera for object tracking**"

This branch holds the complete software that considers the YOLOv4-tiny model and the Model-free approach

It requires openCV 4.5 having the openCV dnn module.

The *src* folder contains all the source code,

The *models* folder is where to put the trained model config file and weights,

The *CMakeList.txt* file. 

To make it simple, the model config and weight file should have the names "*model.cfg*" and "*model.weights*" respectively. 

The *CMakeList.txt* file consist of a set of instructions and directives that describe the project's source files and targets.

## Code compilation
To compile the code :

- Open the terminal in the project location folder
- Type the following commands to compile:

    mkdir build && cd build
    
    cmake ..
    
    cmake --build . (or you can simply type "make")


## Software execution
If the code runs successfully, an executable file with the name "vehicle_tracker" should be located in the "build" folder.

To run the code one must type:

./vehicle\_tracker [-f(--filename)  *<input_file>*] -o(--output) *logfile_name* [-v(--video) *<video_folder>* -t(--timeVideo) *video_time*] [-i(--image) *<image_file>* -j(--timeImage) *image_time*] [-m(--mask) *mask_file*] [-g(--gui)] [-u(model-free)] [-c(--config) *<config_file>*] [-h(--help)]
 

- *-f* or *--filename* followed by the path to the input file (.raw). If this option is not specified, the serial camera is used. If the camera is not plugged in the program, throw an exception.
- *-o* or *--output* followed by the log file name path. If the file exists, the program appends data; otherwise, it creates a new file. If this option is not provided, the program creates a default log file in the executable location folder with a name *log.csv*.
- *-v* or *--video* followed by the path of the folder where to save the output video. The video output name follows this pattern: *output_year_month_date_hh:mm:ss.avi*; in that way, each video has a unique name. If this option is specified, the option *-t* or *--timeVideo* must be used to provide the number of seconds for each video recording.
- *-i* or *--image* followed by the path of the image output file. when this option is specified, the option *-j* or *--timeImage* must be used to specify the time (in second) after which the image should be saved. The image is saved at the same location so it is overwritten every time a new writing occurs.
- *-m* or *--mask* followed by the path to the mask file. The mask file should be a valid *.yaml* file generated while using the *--mask-setup* option; otherwise, the software will crash.
- *-u* or *--mask-setup* followed by the path where to save the mask file. The mask file should have a *.yaml* extension.
- *-g* or *--gui* option to enable the GUI to see what is going on in real-time.
- *-d* or *--model-free* to run the software in the model-free mode. In this case, the software does not use the trained model. If this option is not specified, the software loads the trained model located in the *models* folder.
- *-c* or *--config* option to specify the configuration files containing different program arguments. All the option mentioned above except the *--mask-setup* can be set inside the configuration file and in that way there is no need the command line option. The configuration file is a valid *json* file whose skeleton is provided in the *config.json* file in the source code package.
- *-h* or *--help* option to print out all this description for help.

## Configuration file
The configuration file contains the program options. It can be modified at runtime to update the arguments except the filename(That means the device).

{

    "filename":"/media/user/MY_DISK/recordings/recording_05_08_2021_12_47.raw", *This provides the device used a raw file or the camera. If empty the physical camera is used*
    
    "biases":"", *Provides the path to the biases file containing the camera settings for a specific visual scene*
    
    "serial":"", *Provide the serial number of the camera if many cameras are connected to a computer this can be used to select a specific one*
    
    "masks":"",  *Provides the path to the **.yaml** mask file*
    
    "gui":true,  *If true the GUI is shown otherwise the tracking info are saved in the log file*
    
    "model":true, *If true the model in the models folder is used otherwise the model-free setting is used*
    
    "logfile":"../mylogfile.csv", *Provide the logfile where to save the system information*
    
    "video_output":{
        "path":"../video/",  *Provides the path of the folder where to save the output videos, it must end with a "/"*
        "time":20,            *Provides the lasting time of one video*
        "record_after":25     *Provides time after which a new video should be recorded. For this example, each video lasts 20seconds and a new video is recorded after 25seconds*
    },
    
    "image_output":{          
        "path":"myImage.png", *Gives the image file name where to save the image output* 
        "time":0.1             *Provide the time after which to save the image(it overwrites the image). The value is of type double and it is in seconds*
    },
    
    "alarm_type":"RIGHT-LEFT", *Defines the alarm. It is the direction considered as wrong. It can be RIGHT-LEFT, LEFT-RIGHT, UP-DOWN, DOWN-UP*
    
    "model_free":{
    
        "erosion_type":"RECT", *Defines the erosion type it can be: RECT, CROSS and ELLIPSE*
        
        "erosion_kernel_size":0, *Define the size of the erosion kernel*
        
        "dilation_type":"RECT",  *Defines the dilation type it can be: RECT, CROSS and ELLIPSE*
        
        "dilation_kernel_size":2, *Defines the size of the dilation kernel*
        
        "bbox_width_threshold":10, *Defines the width threshold to filter the bounding box*
        
        "bbox_height_threshold":10, *Defines the height threshold to filter the bounding box*
        
        "bbox_area_threshold":20,   *Defines the area threshold to filter the bounding box*
        
        "car":{
        
            "min_width":0,        *Defines the minimum width to consider a moving object as a car. The same is done for the bikes and pedestrian*
            
            "max_width":2000,     *Defines the maximum width to consider a moving object as a car. The same is done for the bikes and pedestrian*
            
            "max_height":2000,    *Defines the maximum height to consider a moving object as a car. The same is done for the bikes and pedestrian*
            
            "min_height":0        *Defines the minimum height to consider a moving object as a car. The same is done for the bikes and pedestrian*
            
        },
        
        "bike":{
        
            "min_width":0,
            
            "max_width":0,
            
            "max_height":0,
            
            "min_height":0
            
        },
        
        "pedestrian":{
        
            "min_width":0,
            
            "max_width":0,
            
            "max_height":0,
            
            "min_height":0
            
        },

        "detect_area_left":20, *this define the distance from the left in which the object is considered as a detection. If 0 no area is considered. This help decrease the noisy detections*

        "detect_area_right":20, *this define the distance from the right in which the object is considered as a detection. If 0 no area is considered*

        "detect_area_top":0,    *this define the distance from the top in which the object is considered as a detection. If 0 no area is considered*

        "detect_area_bottom":0  *this define the distance from the bottom in which the object is considered as a detection. If 0 no area is considered*

        
    },
    
    "camera_setup":0,    *Provides the camera setup. It can be 0, 15, 45, 90, 180. This values represent the camera angles*

    "algo_direction":0   *Choose the algorithm to compute the direction as described in the report. The value should be 0 or 1*
    
}
