#include <object_detect.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ObjectDetect::ObjectDetect(ros::NodeHandle *nh) : node_handle_(nh) {
  point_cloud_sub_ = node_handle_->subscribe ("/depth_registered/points", 1, &ObjectDetect::PointCloudCB, this);
  debug_pointcloud_pub_ = node_handle_->advertise<sensor_msgs::PointCloud2> ("obj_recognition/debug",1);

  object_markers_pub_.reset(new rviz_visual_tools::RvizVisualTools("camera_depth_optical_frame", "/rviz_visual_tools"));
  object_markers_pub_->loadMarkerPub();

  object_markers_pub_->deleteAllMarkers();
  object_markers_pub_->enableBatchPublishing();
}

void ObjectDetect::PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  object_markers_pub_->deleteAllMarkers();
  object_markers_pub_->publishWireframeCuboid(pose1, 0.02, 0.03, 0.04, rviz_visual_tools::GREEN);

  pcl::PCLPointCloud2 debugPCL;
  sensor_msgs::PointCloud2 debug_output;

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloudFilteredPtr);

  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass_x, pass_y, pass_z;
  pass_x.setInputCloud (xyzCloudPtr);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-0.2, 0.24);
  pass_x.filter (*xyzCloudPtrFiltered);

  pass_y.setInputCloud (xyzCloudPtrFiltered);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.17, 0.11);
  pass_y.filter (*xyzCloudPtrFiltered);

  pass_z.setInputCloud (xyzCloudPtrFiltered);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0, 0.55);
  pass_z.filter (*xyzCloudPtrFiltered);

  // pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,debugPCL);
  // pcl_conversions::fromPCL(debugPCL, debug_output);
  // m_debugPub.publish(debug_output);

  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.008);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);

  pcl::toPCLPointCloud2( *xyzCloudPtrRansacFiltered ,debugPCL);
  pcl_conversions::fromPCL(debugPCL, debug_output);
  debug_pointcloud_pub_.publish(debug_output);

  // perform euclidean cluster segmentation to seporate individual objects

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (18);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);

  // declare an instance of the SegmentedClustersArray message
  // obj_recognition::SegmentedClustersArray CloudClusters;

  // declare the output variable instances
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 outputs[7];
  pcl::PCLPointCloud2 outputPCL;

  // ROS_INFO_STREAM(cluster_indices.size());
  int i=0;
  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

    // create a new clusterData message object
    //obj_recognition::ClusterData clusterData;

    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*clusterPtr, centroid);

    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*clusterPtr, minPoint, maxPoint);

    // ROS_INFO("%lf, %lf, %lf, %lf", centroid[0], centroid[1], centroid[2], centroid[3]);
    pose1.translation() << (maxPoint[0]+minPoint[0])/2.0, (maxPoint[1]+minPoint[1])/2.0, minPoint[2];
    pose2.translation() << (maxPoint[0]+minPoint[0])/2.0, (maxPoint[1]+minPoint[1])/2.0, (maxPoint[2]+minPoint[2])/2.0;
    // object_markers_pub_->publishWireframeCuboid(pose1, 0.02, 0.03, 0.04, rviz_visual_tools::GREEN);
    object_markers_pub_->publishWireframeCuboid(pose2, maxPoint[0]-minPoint[0], maxPoint[1]-minPoint[1], maxPoint[2]-minPoint[2], rviz_visual_tools::GREEN);
    object_markers_pub_->publishAxis(pose1);

    // log the position of the cluster
    //clusterData.position[0] = (*cloudPtr).data[0];
    //clusterData.position[1] = (*cloudPtr).points.back().y;
    //clusterData.position[2] = (*cloudPtr).points.back().z;
    //std::string info_string = string(cloudPtr->points.back().x);
    //printf(clusterData.position[0]);

    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);

    // add the cluster to the array message
    //clusterData.cluster = output;
    // CloudClusters.clusters.push_back(output);

    if(i<7) {
      outputs[i] = output;
      i++;
    }
  }

  object_markers_pub_->trigger();
}
