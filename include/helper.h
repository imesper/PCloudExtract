//
// Created by ian on 26.11.2021.
//

#ifndef PCLOUDEXTRACT_HELPER_H
#define PCLOUDEXTRACT_HELPER_H

#include "pclhelper.h"

#include <librealsense2/rs.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// PCL Headers
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/pca.h>
#include <pcl/console/parse.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//#include "unet.h"

class Helper {

public:
  explicit Helper();
  ~Helper();

  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  RS2toPCLCuda(rs2::frame &depth, rs2::frame &RGB, cv::Mat mask);
  template <typename T>
  static void saveFileTXT(const std::string &filename, const T &cloud) {

    std::fstream fs;

    fs.open(filename, std::fstream::out);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      fs << cloud->points[i].x << ";" << cloud->points[i].y << ";"
         << cloud->points[i].z << "\n";
    }
    fs.close();
  };

  //    static cv::Affine3d readTransformation(std::string file);
  //    static void pcs_alignmentFPFH(
  //            pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
  //            pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
  //            pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
  //            pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
  //            const int samples,
  //            const int correspRandomness,
  //            const double similarity,
  //            const double maxCorrespDist,
  //            const double inlierFraction,
  //            Eigen::Matrix4d* transformation);
  //    static void computeFPFHFeatures(
  //            pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample,
  //            pcl::PointCloud<pcl::Normal>::Ptr normals,
  //            pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors,
  //            double feature_radius);
  //    static void computeSurfaceNormals(
  //            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  //            pcl::PointCloud<pcl::Normal>::Ptr normals,
  //            double normal_radius);

  //  static cv::Mat ToCvImage(at::Tensor tensor);

  //  static torch::Tensor ToTensor(cv::Mat img,
  //                                bool show_output = false,
  //                                bool unsqueeze = false,
  //                                int unsqueeze_dim = 0);

  //  static torch::nn::ModuleList get_encoder_blocks(
  //    vision::models::VGG16BN model);
  //  static cv::Mat processImageUNet(cv::Mat image);
  //  static UNet startUnet();
};

#endif // PCLOUDEXTRACT_HELPER_H
