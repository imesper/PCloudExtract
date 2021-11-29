//
// Created by ian on 26.11.2021.
//

#include "../include/helper.h"

Helper::Helper() {}

Helper::~Helper() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Helper::RS2toPCLCuda(rs2::frame &depth, rs2::frame &RGB, cv::Mat mask) {
#ifdef PRINT_TIME
  pcl::ScopeTime t("Processing Cloud Cuda");
#endif

  //  std::cout << "Start of RS2TOPCL";

  rs2_intrinsics intrinsics =
      RGB.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

  //  std::cout << "After CV MAT of RS2TOPCL";
  auto color = cv::Mat(RGB.as<rs2::video_frame>().get_height(),
                       RGB.as<rs2::video_frame>().get_width(), CV_8UC3,
                       const_cast<void *>(RGB.get_data()), cv::Mat::AUTO_STEP);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  float fx = intrinsics.fx;
  float fy = intrinsics.fy;
  float cx = intrinsics.ppx;
  float cy = intrinsics.ppy;
  cv::Mat depth_image(depth.as<rs2::depth_frame>().get_height(),
                      depth.as<rs2::depth_frame>().get_width(), CV_64F,
                      cv::Scalar(0.0));

  auto depthMat =
      cv::Mat(depth.as<rs2::depth_frame>().get_height(),
              depth.as<rs2::depth_frame>().get_width(), CV_16UC1,
              const_cast<void *>(depth.get_data()), cv::Mat::AUTO_STEP);

  depthMat.convertTo(depth_image, CV_32F);

  if (!depth_image.data) {
    std::cerr << "No depth data!!!" << std::endl;
    return nullptr;
  }

  cloud->width = static_cast<uint>(depth_image.cols);
  cloud->height = static_cast<uint>(depth_image.rows);
  cloud->reserve(cloud->width * cloud->height);

  // if (m_cloud->size() < (m_cloud->width * m_cloud->height)) {
  //   qDebug() << "Resizing Cloud";
  //   m_cloud->width = static_cast<uint>(depth_image.cols);
  //   m_cloud->height = static_cast<uint>(depth_image.rows);
  //   m_cloud->resize(m_cloud->width * m_cloud->height);
  // }

  int w = depth_image.cols;
  int h = depth_image.rows;

  int size = w * h;
  uchar *rgbM = color.data;
  {

#ifdef PRINT_TIME
    pcl::ScopeTime t("Processing Cuda");
#endif
    float Z;
    size_t bytes = size * sizeof(float);
    float *image = reinterpret_cast<float *>(depth_image.data);
    const double f_scale = 0.001;
    float *z = (float *)malloc(bytes);
    float *x = (float *)malloc(bytes);
    float *y = (float *)malloc(bytes);
    //    pcl::PointXYZRGB *points = cloud->points.data();

    // toXYZRGB(fx, fy, cx, cy, w, h, depth_image.ptr<float>(), rgbM, points);

    for (int y = 0; y < depth_image.rows; y++) {
      for (int x = 0; x < depth_image.cols; x++) {

        //        cv::Point2f point(x, y);
        if (!mask.at<uint8_t>(y, x))
          continue;
        int in = (y * depth_image.cols) + x;

        float d = image[in];

        Z = (f_scale * d);

        pcl::PointXYZRGB p;

        p.z = Z;
        p.x = ((x - cx) / fx) * Z;
        p.y = ((y - cy) / fy) * Z;

        int index = (color.step[0] * y) + (3 * x);
        p.r = rgbM[index];     // Reference tuple<2>
        p.g = rgbM[index + 1]; // Reference tuple<1>
        p.b = rgbM[index + 2]; // Reference tuple<0>
        p.x = floor(p.x * 144.444);
        p.y = floor(p.y * 144.444);
        p.z = floor(p.z * 144.444);

        cloud->points.emplace_back(p);
        // }
      }
    }

    free(x);
    free(y);
    free(z);
  }
  return cloud;
}
