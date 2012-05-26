#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using sensor_msgs::PointCloud;

class GroundFilter {
private:
  ros::NodeHandle n_;
  tf::TransformListener listener_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
  ros::Publisher pc_pub_2;

public:
  GroundFilter(ros::NodeHandle n) : 
    n_(n)
  {
    pc_sub_ =
      n_.subscribe("/stereo/points", 1, &GroundFilter::pcCallback, this);
    pc_pub_ = n_.advertise<PointCloud>("/ground_filter/points", 1);
    pc_pub_2 = n_.advertise<PointCloud>("/ground_filter/points2", 1);
  }

  void pcCallback(const PointCloud::ConstPtr& pc_in)
  {
    // Convert the pc to a pc2
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(*pc_in, pc2);
    // PCL pc used for filtering
    pcl::PointCloud<pcl::PointXYZ> pc, pc_ground, pc_nonground;
    // Convert into a PCL pc
    pcl::fromROSMsg(pc2, pc);
    // Get tf info
    tf::StampedTransform sensorToBaseTF;
    try
    {
      listener_.waitForTransform("base_footprint", pc_in->header.frame_id,
                                 pc_in->header.stamp, ros::Duration(0.2));
      listener_.lookupTransform("base_footprint", pc_in->header.frame_id,
                                pc_in->header.stamp, sensorToBaseTF);
    }
    catch (tf::TransformException& e)
    {
        ROS_ERROR("%s", e.what());
        return;
    }
    
    // Convert the tf to an eigen
    Eigen::Matrix4f sensorToBase;
    pcl_ros::transformAsMatrix(sensorToBaseTF, sensorToBase);

    // Transform the pc
    pcl::transformPointCloud(pc, pc, sensorToBase);

    // set up filter for height range, also removes NANs:
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, 2.0);

    // Filter the pc
    pass.setInputCloud(pc.makeShared());
    pass.filter(pc);

    // Now estimate and remove the ground plane
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.1);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(0.35);

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
      if (std::abs(coefficients->values.at(3)) < 0.07) {
        ROS_INFO("True");
      } else {
        ROS_INFO("False");
      }
      // Get the indices using the extractor
      extract.setInputCloud(filter_pc.makeShared());
      extract.setIndices(inliers);
      extract.setNegative(false);
      pcl::PointCloud<pcl::PointXYZ> cloud_out;
      extract.filter(cloud_out);
      pc_ground += cloud_out;
      if(inliers->indices.size() != filter_pc.size()){
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloud_out2;
        extract.filter(cloud_out2);
        pc_nonground += cloud_out2;
        filter_pc = cloud_out2;
      }
      found_ground = true;
    }

    // Reconvert and publish the pc1
    PointCloud pc_out;
    pcl::toROSMsg(pc_nonground, pc2);
    sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc_out);
    pc_out.header.frame_id = "base_footprint";
    pc_pub_.publish(pc_out);
    // Reconvert and publish the pc1
    PointCloud pc_out2;
    pcl::toROSMsg(pc_ground, pc2);
    sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc_out2);
    pc_out2.header.frame_id = "base_footprint";
    pc_pub_2.publish(pc_out2);
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