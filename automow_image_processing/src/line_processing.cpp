#include <ros/param.h>
#include <string>
#include <iostream>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/CvBridge.h>
#include <cstdio>
#include <cstdlib>
#include <boost/thread.hpp>
#include <highgui.h>
#include <cv.h>
#include <list>
#include "line_processing/LineProcessingControl.h"
#include <dynamic_reconfigure/server.h>
#include "line_processing/line_processingConfig.h"

#define PIX_2_M 0.000377 * 5

// Uncomment this line to have the lines drawn on the original image and transmitted
#define __TX_POINT_CLOUD
#define __TX_PROCESSED_IMAGE
#define __TX_DEBUG_IMAGE
#define __SECONDARY_HOUGHLINE
#define __STOCK_IMAGE

sensor_msgs::CvBridge bridge_;

// Load the bird's eye view conversion matrix 
CvMat *birdeye_mat;

IplImage *captured_img_bird, *b_plane, *g_plane, *r_plane, *img_lines;
CvMemStorage *storage;
CvSeq *lines;
CvPoint pt1, pt2;

#ifdef __TX_POINT_CLOUD
ros::Publisher point_cloud_publisher;
#endif

#ifdef __TX_PROCESSED_IMAGE
ros::Publisher processed_image_publisher;
#endif

#ifdef __TX_DEBUG_IMAGE
ros::Publisher blue_plane_image_publisher;
ros::Publisher lines_image_publisher;
#endif

ros::NodeHandle *n;
uint8_t enabled;
sensor_msgs::PointCloud::_points_type g_points;
boost::mutex write_mux;

ros::Time time_of_image;

// These are Dynamic Configurations
bool STOCK_IMAGE_ENABLED;
const char *STOCK_IMAGE_PATH;
int BRIGHTNESS_THRESHOLD;
bool HISTOGRAM_ENABLED;
int HOUGH1_MIN_POINTS;
int HOUGH1_MAX_SPACING;
bool HOUGH2_ENABLED;
int HOUGH2_MIN_POINTS;
int HOUGH2_MAX_SPACING;

struct window_density {
    uchar *index;
    int density;
};

struct make_point_cloud {
  const uchar* ptr;
  int start;
  int end;
  sensor_msgs::PointCloud::_points_type points;
  make_point_cloud(const uchar* ptr, int row_start, int row_end) : ptr(ptr), start(row_start), end(row_end) { }
  void operator() () {
    for (int n = start; n < end; n += 5) {
      for (int k = 1; k < img_lines->width; k += 5) {
        bool leave_loop = true;
        // ROS_INFO_STREAM("Row: " << n);
        for (int i = 5; leave_loop && i != 0; --i) {
          for (int g = 5; leave_loop && g != 0; --g) {
            if (ptr[n * img_lines->widthStep + k] != 0) {
              int col = k + g - 193;
              int row = img_lines->height - n + i;
              
              geometry_msgs::Point32 found_point;
              found_point.x = col * PIX_2_M;
              found_point.y = row * PIX_2_M;
              found_point.z = 0;
              points.push_back(found_point);
              leave_loop = false;
            }
          }
        }
      }
    }
    boost::mutex::scoped_lock lock(write_mux);
    while (!points.empty()) {
      g_points.push_back(points.back());
      points.pop_back();
    }
  }
};

void invertImage(IplImage *img) {
    for(int n=0; n < img->height; n++) {
        uchar *ptr = (uchar*)(img->imageData + n * img->widthStep);
        //max_height = n;
        for(int k=0; k < img->width; k++) {
            ptr[k] = 255 - ptr[k];
        //  max_width = k;
        }
    }
};

void binaryPixelDensityFinder(IplImage *img, int window_height, int window_width, struct window_density window_densities[]) {
    int density = 0;
    int index = 0;
    
    for(int n=0; n < img->height; n+=window_height) { // shift down rows to next row-window
        uchar *ptr1 = (uchar*)(img->imageData + n*img->widthStep);
        
        for(int m=0; m < img->width; m+=window_width) { // shift along columns to next window
            uchar *ptr2 = ptr1 + m;
            
            for(int k=0; k < window_height; k++) { // shift down by single rows within window
                uchar *ptr3 = ptr2 + k*img->widthStep; 
                
                for(int w=0; w < window_width; w++) // sum along columns in row
                    density+= *(ptr3+w);
            }
            
            window_densities[index].index = ptr2;
            window_densities[index++].density = density;
            
            density = 0;
        }
    }
};

void DensityFilter(struct window_density densities[], int max_density, int window_height, int window_width, int widthStep, int max_windows) {
    for(int n=0; n < max_windows; n++) {
        if(densities[n].density < max_density*0.6) {
            for(int k=0; k < window_height; k++) {
                uchar *ptr = densities[n].index + k*widthStep;
                
                for(int m=0; m < window_width; m++)
                    *(ptr+m) = 0;
            }
        }
    }
};

bool lineProcessingControl(line_processing::LineProcessingControl::Request &request, line_processing::LineProcessingControl::Response &response) {
    enabled = request.enable;
    response.result = true;
    return true;
}

inline void blackoutImage(IplImage *img) {
    for (int n = img->height; n != 0; --n) {
        for (int k = img->width; k != 0; --k) {
            uchar* val = (uchar*)img->imageData + (n * img->widthStep) + k;
            *val = 0;
        }
    }
}

/* countPixels() counts the number of pixels in an image that equal a given value
 */
long long countPixels(IplImage *img, int value, int target = 0) {
    long long count = 0;
    for (int n = img->height; n != 0; --n) {
      uchar *ptr = (uchar*)(img->imageData + n * img->widthStep);
      for (int k = img->width; k != 0; --k) {
        if (ptr[k] == value)
          count++;
        if (count > target) 
          return count;
      }
    }
    return count;
};

uchar maxPixelValue(IplImage *img) {
  uchar max = 0;
  for (int n = img->height; n != 0; --n) { 
    for (int k = img->width; k != 0; --k) {
        uchar pixel_value = *((uchar*)img->imageData + (n * img->widthStep) + k);
        if (max < pixel_value)
            max = pixel_value;
    }
  }
  
  return max;
};

// void reduceNoise(IplImage *img) {
//     assert(img->width%16 == 0 && img->height%16 == 0);
//     for(int i = img->width/16; i > 0; --i) {
//         
//     }
//     for (int n = 0; n < img->; n += 5) {
//       for (int k = 1; k < img_lines->width; k += 5) {
//         bool leave_loop = true;
//         // ROS_INFO_STREAM("Row: " << n);
//         for (int i = 5; leave_loop && i != 0; --i) {
//           for (int g = 5; leave_loop && g != 0; --g) {
//             if (ptr[n * img_lines->widthStep + k] != 0) {
//               int col = k + g - 193;
//               int row = img_lines->height - n + i;
//               
//               geometry_msgs::Point32 found_point;
//               found_point.x = col * PIX_2_M;
//               found_point.y = row * PIX_2_M;
//               found_point.z = 0;
//               points.push_back(found_point);
//               leave_loop = false;
//             }
//           }
//         }
//       }
//     }
// }

void findLinesInImage(IplImage *img) {
    if (!enabled)
        return;
    
    // window density variables
	int window_height = 20;
	int window_width = 20;
	int max_windows;
	float percent_coverage = 1;
	int min_density = ( window_height * window_width ) * percent_coverage * 255;
    struct window_density densities[1201];
    
    // values for determining coordinates for point cloud
    int max = 0; //img_small->widthStep * img_small->height;
    uchar *ptr;      // used to iterate through the small image
    
    /////////////////////////////////////// Begin Line Detection /////////////////////////////////////////
    
    // cvCvtPixToPlane(img, b_plane, g_plane, r_plane, 0); //split image into channel planes
    cvCvtColor(img, b_plane, CV_RGB2GRAY);
    if (HISTOGRAM_ENABLED)
        cvEqualizeHist(b_plane, b_plane);   //spread the values of the blue plane
    
    max_windows = ( b_plane->height / window_height ) * ( b_plane->width / window_width );
    
    sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(b_plane);
    processed_image_publisher.publish(*processed_ros_img);
    
    // threshold the blue plane to reduce the viable points for Hough
    cvThreshold(b_plane, b_plane, BRIGHTNESS_THRESHOLD, 255, CV_THRESH_BINARY);
    
    binaryPixelDensityFinder(b_plane, window_height, window_width, densities);
	
	DensityFilter(densities, min_density, window_height, window_width, b_plane->widthStep, max_windows);
    
    // perform probabalistic Hough transform:
    // 4th arg: rho resolution
    // 5th arg: theta resolution
    // 6th arg: # of colinear points needed for line
    // 7th arg: max pixel spacing allowed between points on a line
    // lines = cvHoughLines2(b_plane, storage, CV_HOUGH_PROBABILISTIC, 2, CV_PI/2, 2, HOUGH1_MIN_POINTS, HOUGH1_MAX_SPACING);
    // 
    // if (lines->total != 0) {
    //     for (int n=0; n < lines->total; n++) {
    //         int *line = (int*)cvGetSeqElem(lines, n);
    //         
    //         pt1.x = line[0];
    //         pt1.y = line[1];
    //         pt2.x = line[2];
    //         pt2.y = line[3];
    //         
    //         cvLine(img_lines, pt1, pt2, cvScalar(255, 0, 0, 0), 2, CV_AA, 0);
    //     }
    // }
    // if (HOUGH2_ENABLED) {
    //     cvClearSeq(lines);
    //     cvClearMemStorage(storage);
    //     lines = cvHoughLines2(img_lines, storage, CV_HOUGH_PROBABILISTIC, 2, CV_PI/4, 2, HOUGH2_MIN_POINTS, HOUGH2_MAX_SPACING);
    //     cvZero(img_lines);
    // 
    //     if (lines->total != 0) {
    //         for (int n=0; n < lines->total; n++) {
    //             int *line = (int*)cvGetSeqElem(lines, n);
    //         
    //             pt1.x = line[0];
    //             pt1.y = line[1];
    //             pt2.x = line[2];
    //             pt2.y = line[3];
    //         
    //             cvLine(img_lines, pt1, pt2, cvScalar(255, 0, 0, 0), 4, CV_AA, 0);
    //         }
    //     }
    // }
    
#ifdef __TX_DEBUG_IMAGE
{
    sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(img_lines);
    lines_image_publisher.publish(*processed_ros_img);
}
#endif
    
#ifdef __TX_DEBUG_IMAGE
{
    sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(b_plane);
    blue_plane_image_publisher.publish(*processed_ros_img);
}
#endif
    
    // img_lines = b_plane;
    
    ptr = (uchar*)b_plane->imageData;
    
    // max = img_lines->widthStep * img_lines->height;
    
    sensor_msgs::PointCloud point_cloud;
    
    point_cloud.header.frame_id = "camera_frame";
    point_cloud.header.stamp = time_of_image;
   
    // if (lines->total != 0) {
    if (1) {
        int one_fifth = 480 / 5;
        boost::thread calc1(make_point_cloud(ptr, 0, one_fifth - 1));
        boost::thread calc2(make_point_cloud(ptr, (one_fifth * 1), (one_fifth * 2) - 1));
        boost::thread calc3(make_point_cloud(ptr, (one_fifth * 2), (one_fifth * 3) - 1));
        boost::thread calc4(make_point_cloud(ptr, (one_fifth * 3), (one_fifth * 4) - 1));
        boost::thread calc5(make_point_cloud(ptr, (one_fifth * 4), 480));
        calc1.join();
        calc2.join();
        calc3.join();
        calc4.join();
        calc5.join();

        point_cloud.points = g_points;
    }

    ROS_DEBUG_STREAM("Point Count: " << g_points.size());
    
    point_cloud_publisher.publish(point_cloud);
    
    // cvZero(img_lines);
    // cvClearSeq(lines);
    // cvClearMemStorage(storage);
};

void imageReceived(const sensor_msgs::ImageConstPtr& ros_img) {
    time_of_image = ros_img->header.stamp;
    g_points.clear();
    
    // Convert the image received into an IPLimage
    IplImage *captured_img = bridge_.imgMsgToCv(ros_img);
    
    if (STOCK_IMAGE_ENABLED)
        captured_img = cvLoadImage(STOCK_IMAGE_PATH, CV_LOAD_IMAGE_UNCHANGED);
    
    cvWarpPerspective(captured_img, captured_img_bird, birdeye_mat,
                      CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS); 
    
    findLinesInImage(captured_img_bird);
    
#ifdef __TX_PROCESSED_IMAGE
    // Convert processed image into ros_img_msg
//////////////////////// > We need to make sure that these variables are recycled properly
    // sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(captured_img_bird);
//////////////////////// <
    // Publish image to topic /processed_image
    // processed_image_publisher.publish(*processed_ros_img);
#endif
}

void dynamicReconfigureCallback(line_processing::line_processingConfig &config, uint32_t level) {
    STOCK_IMAGE_ENABLED      = config.stock_image_enabled;
    STOCK_IMAGE_PATH        = config.stock_image.c_str();
    BRIGHTNESS_THRESHOLD    = (int)config.brightness_threshold;
    HISTOGRAM_ENABLED       = config.histogram_enabled;
    HOUGH1_MIN_POINTS       = (int)config.hough1_min_points;
    HOUGH1_MAX_SPACING      = (int)config.hough1_max_spacing;
    HOUGH2_ENABLED          = config.hough2_enabled;
    HOUGH2_MIN_POINTS       = (int)config.hough2_min_points;
    HOUGH2_MAX_SPACING      = (int)config.hough2_max_spacing;
}

int main(int argc, char** argv) {
    // Initialize the node
    ros::init(argc, argv, "automow_image_processing");
    n = new ros::NodeHandle;
    
    // Setup enable/disable Service
    ros::ServiceServer service = n->advertiseService("lineProcessingControl", lineProcessingControl);
        
    // load white grass image template
    char dir[FILENAME_MAX]; 
    std::string path;
    
    if (getcwd(dir, sizeof(dir))) { 
      path = std::string(dir) + "/birdeye_convert_mat.xml";
    }

    // Load the bird's eye view conversion matrix 
    birdeye_mat = (CvMat*)cvLoad(path.c_str());
    if (birdeye_mat == NULL) { 
        ROS_ERROR("Birds eye matrix not loaded properly. Set the birds_eye param");
        return -1;
    }
    // Register the node handle with the image transport
    image_transport::ImageTransport it(*n);
    // Set the image buffer to 1 so that we process the latest image always
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageReceived);
    point_cloud_publisher = n->advertise<sensor_msgs::PointCloud>("image_point_cloud", 5);
#ifdef __TX_PROCESSED_IMAGE
    processed_image_publisher = n->advertise<sensor_msgs::Image>("processed_image", 1);
#endif
#ifdef __TX_DEBUG_IMAGE
    blue_plane_image_publisher = n->advertise<sensor_msgs::Image>("blue_plane", 1);
    lines_image_publisher = n->advertise<sensor_msgs::Image>("lines_image", 1);
#endif
   
    if (getcwd(dir, sizeof(dir))) { 
      path = std::string(dir) + "/template_jpg.jpg";
    }
    
    // image pointers
    b_plane = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
    g_plane = cvCreateImage(cvGetSize(b_plane), IPL_DEPTH_8U, 1);
    r_plane = cvCreateImage(cvGetSize(b_plane), IPL_DEPTH_8U, 1);
    img_lines = cvCreateImage(cvGetSize(b_plane), IPL_DEPTH_8U, 1);
    
    // create bird's eye view
    captured_img_bird = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
                            
    // create memory storage for lines returned by Hough
    storage = cvCreateMemStorage(0);
    
    enabled = true;
    
    // Initialize dynamic defaults
    dynamic_reconfigure::Server<line_processing::line_processingConfig> srv;
    dynamic_reconfigure::Server<line_processing::line_processingConfig>::CallbackType f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    srv.setCallback(f);
    
    // Run until killed
    ros::spin();
    // Clean exit
    return 0;
}

