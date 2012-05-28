#include <string>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <dynamic_reconfigure/server.h>
#include "automow_filtering/automow_filteringConfig.h"

using std::string;
using sensor_msgs::PointCloud;
using sensor_msgs::PointCloud2;

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

namespace my_modes {
  typedef enum {
    NONE = 0,
    SACMODEL_PERPENDICULAR_PLANE = 1
  } modes_t;
}

using namespace my_modes;

class GroundFilter {
private:
  ros::NodeHandle n_;
  tf::TransformListener listener_;
  ros::Subscriber pc_sub_;
  ros::Publisher obstacles_pub_;
  ros::Publisher nonobstacles_pub_;
  dynamic_reconfigure::Server<automow_filtering::automow_filteringConfig> server_;

  int mode_;
  string vehicle_frame_;
  double min_z, max_z;
  PCLPointCloud last_pc;
  bool paused, first_cloud, z_filter, downsample;
  double resolution;
  int max_iterations;
  double distance_threshold, eps_angle;

public:
  GroundFilter(ros::NodeHandle n) : 
    n_(n),
    mode_(SACMODEL_PERPENDICULAR_PLANE),
    vehicle_frame_("base_footprint"),
    min_z(0.1f),
    max_z(0.75f),
    paused(false),
    first_cloud(true),
    z_filter(true),
    downsample(false),
    resolution(0.001f),
    max_iterations(400),
    distance_threshold(0.1),
    eps_angle(15.0)
  {
    pc_sub_ = n_.subscribe("/stereo/points", 1, &GroundFilter::pcCallback, this);
    obstacles_pub_ = n_.advertise<PointCloud2>("/ground_filter/obstacles", 1);
    nonobstacles_pub_ = n_.advertise<PointCloud2>("/ground_filter/non_obstacles", 1);

    server_.setCallback(boost::bind(&GroundFilter::onConfigure, this, _1, _2));

    n_.getParam(string("vehicle_fame"), vehicle_frame_);
    n_.getParam("resolution", resolution);
    n_.getParam("min_z", min_z);
    n_.getParam("max_z", max_z);
    n_.getParam("filtering_mode", mode_);
    n_.getParam("max_iterations", max_iterations);
    n_.getParam("distance_threshold", distance_threshold);
    n_.getParam("eps_angle", eps_angle);
  }

  void onConfigure(automow_filtering::automow_filteringConfig &config, uint32_t level) {
    vehicle_frame_ = config.vehicle_frame;
    downsample = config.downsample;
    resolution = config.resolution;
    z_filter = config.z_filter;
    min_z = config.min_z;
    max_z = config.max_z;
    paused = config.paused;
    mode_ = config.filtering_mode;
    max_iterations = config.max_iterations;
    distance_threshold = config.distance_threshold;
    eps_angle = config.eps_angle;
    if (paused && !first_cloud) {
      last_pc.header.stamp = ros::Time::now();
      PCLPointCloud pc = last_pc;
      processPC(pc);
    }
  }

  void pcCallback(const PointCloud::ConstPtr& pc_in)
  {
    if (paused) {
      return;
    }
    // First transform the point cloud to the vehicle frame
    PointCloud pc_tf;
    try {
      listener_.waitForTransform(vehicle_frame_, pc_in->header.frame_id,
                                 pc_in->header.stamp, ros::Duration(0.2));
      listener_.transformPointCloud(vehicle_frame_, *pc_in, pc_tf);
    }
    catch (tf::TransformException& e) {
        ROS_WARN("Failed to transform PointCloud: %s", e.what());
        return;
    }
    // Then convert it to a PointCloud2 and then a PCL point cloud
    PointCloud2 pc2_in;
    sensor_msgs::convertPointCloudToPointCloud2(pc_tf, pc2_in);
    PCLPointCloud pc;
    pcl::fromROSMsg(pc2_in, pc);
    last_pc = pc;
    if (first_cloud) {
      first_cloud = false;
    }
    processPC(pc);
  }

  void processPC(PCLPointCloud &pc) {
    // Do processing based on the settings
    if (downsample) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(pc.makeShared());
      vg.setLeafSize(resolution, resolution, resolution);
      vg.filter(pc);
    }
    if (z_filter) {
      filterByHeight(pc, min_z, max_z);
    }
    if (mode_ == SACMODEL_PERPENDICULAR_PLANE) {
      PCLPointCloud ground, nonground;
      segmentWithRANSAC(pc, ground, nonground);
      // Publish the nonground pc to obstacles
      publishPCLPointCloud(nonground, obstacles_pub_);
      // Publish ground to the nonobstacles
      publishPCLPointCloud(ground, nonobstacles_pub_);
    } else {
      publishPCLPointCloud(pc, obstacles_pub_);
    }
  }

  void publishPCLPointCloud(PCLPointCloud &pc, ros::Publisher &pub) {
    PointCloud2 pc_out;
    pcl::toROSMsg(pc, pc_out);
    pc_out.header.frame_id = vehicle_frame_;
    pub.publish(pc_out);
  }

  void segmentWithRANSAC(PCLPointCloud &pc, PCLPointCloud &ground, PCLPointCloud &nonground) {
    // Remove data way below or above the ground
    filterByHeight(pc, min_z, max_z);
    // Set the headers
    ground.header = pc.header;
    nonground.header = pc.header;
    // Create the coefficients and inliers storage
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create a SACSeg. object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations); // 400
    seg.setDistanceThreshold(distance_threshold); // 0.1
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(pcl::deg2rad(eps_angle)); // 0.15
    // Filter until the cloud is too small or I "find" the ground plane
    pcl::PointCloud<pcl::PointXYZ> filter_pc(pc);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    bool found_ground = false;
    while (filter_pc.size() > 10 && !found_ground) {
      seg.setInputCloud(filter_pc.makeShared());
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0) {
        ROS_WARN("No plane found in the pointcloud!");
        break;
      }
      // if (std::abs(coefficients->values.at(3)) < 0.07) {
      //   ROS_INFO("True");
      // } else {
      //   ROS_INFO("False");
      // }
      // Get the indices using the extractor
      extract.setInputCloud(filter_pc.makeShared());
      extract.setIndices(inliers);
      extract.setNegative(false);
      pcl::PointCloud<pcl::PointXYZ> cloud_out;
      extract.filter(cloud_out);
      ground += cloud_out;
      if(inliers->indices.size() != filter_pc.size()){
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloud_out2;
        extract.filter(cloud_out2);
        nonground += cloud_out2;
        filter_pc = cloud_out2;
      }
      found_ground = true;
    }
  }

  void filterByHeight(PCLPointCloud &pc, double &min, double &max) {
    // set up filter for height range, also removes NANs:
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min, max);

    // Filter the pc
    pass.setInputCloud(pc.makeShared());
    pass.filter(pc);
  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "ground_filter");
  ros::NodeHandle n;
  GroundFilter gf(n);
  
  ros::spin();
  
  return 0;
}