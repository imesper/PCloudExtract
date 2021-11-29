//
// Created by ian on 26.11.2021.
//

#include "BagFile.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

BagFile::BagFile(std::string filename, std::string matrix)
    : _filename(filename) {

  rs2::align align_to_depth(RS2_STREAM_COLOR);
  rs2::pipeline pipe;
  rs2::config cfg;

  cfg.enable_device_from_file(_filename);

  rs2::frameset fs;

  uint64_t curPos;
  uint64_t lastPos = 0;
  pipe.start(cfg);
  auto device = pipe.get_active_profile().get_device();

  while (true) {

    fs = pipe.wait_for_frames();

    curPos = fs.get_frame_number();
    //    std::cout << "Cur: " << curPos << std::endl;
    //    std::cout << "Lss: " << lastPos << std::endl;
    if (curPos < lastPos)
      break;

    lastPos = curPos;

    if (fs.size() < 2) {
      //      std::cout << "Not 2 frames" << std::endl;
      //      std::cout << fs.size();
      continue;
    }

    try {
      fs = align_to_depth.process(fs);
    } catch (std::exception &e) {
      //      std::cout << "Error RS Align" << std::endl;
      //      std::cout << e.what();
      continue;
    }
    _depths.emplace_back(fs.get_depth_frame());
    _colors.emplace_back(fs.get_color_frame());

    //    std::cout << _depths.size();
  }

  _intrinsics =
      _colors[0].get_profile().as<rs2::video_stream_profile>().get_intrinsics();
  _serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
BagFile::getPointCloud(int frame, bool flipped, bool filter,
                       const std::vector<float> &filterX,
                       const std::vector<float> &filterZ) {

  if (_depths.size() <= frame)
    frame = _depths.size() - 1;

  auto color = cv::Mat(_colors[frame].as<rs2::video_frame>().get_height(),
                       _colors[frame].as<rs2::video_frame>().get_width(),
                       CV_8UC3, const_cast<void *>(_colors[frame].get_data()),
                       cv::Mat::AUTO_STEP);

  //  std::cout << "Color frame: " << color.size() << std::endl;

  cv::Mat mask(color.rows, color.cols, CV_8UC3, cv::Scalar(255, 255, 255));

  if (filter) {
    std::ostringstream python_path;
    python_path << "/home/ian/Development/supervisely/plugins/"
                   "nn/icnet/src";

    py::module_ sys = py::module_::import("sys");
    py::object sys_path = sys.attr("path");
    py::object sys_append = sys_path.attr("append");
    sys_append(python_path.str());

    py::module_ inf = py::module_::import("inf");
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rgb;
    cv::Mat bgr[3];        // destination array
    cv::split(color, bgr); // split source

    Eigen::Map<
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::RowMajor, Eigen::Stride<3, 1>>
        red(bgr[2].ptr<uint8_t>(), color.rows, color.cols);

    Eigen::Map<
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::RowMajor, Eigen::Stride<3, 1>>
        green(bgr[1].ptr<uint8_t>(), color.rows, color.cols);
    Eigen::Map<
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::RowMajor, Eigen::Stride<3, 1>>
        blue(bgr[0].ptr<uint8_t>(), color.rows, color.cols);

    py::array_t<uint8_t> res = inf.attr("getMask")(red, green, blue);

    auto buf = py::array_t<uint8_t>::ensure(res);
    //    if (!buf)
    //      std::cout << " Fuck 1" << std::endl;

    auto dims = buf.ndim();
    //    std::cout << "Dim buf: " << dims << std::endl;
    //
    //    if (dims < 1 || dims > 2)
    //      std::cout << " Fuck 2" << std::endl;

    auto fits = py::detail::EigenProps<Eigen::MatrixXi>::conformable(buf);
    //
    //    std::cout << "Fist rows and cols " << fits.rows << " " << fits.cols
    //              << std::endl;
    //    if (!fits)
    //      std::cout << " Fuck 3" << std::endl;

    mask = cv::Mat(fits.rows, fits.cols, CV_8UC1, buf.request().ptr,
                   cv::Mat::AUTO_STEP);

    cv::resize(mask, mask, cv::Size(color.cols, color.rows), 0, 0,
               cv::INTER_CUBIC);
  }
  auto cloud = Helper::RS2toPCLCuda(_depths[frame], _colors[frame], mask);

  //  std::cout << "Cloud Size: " << cloud->size() << std::endl;

  pcl::PassThrough<pcl::PointXYZRGB> Cloud_z_Filter;
  Cloud_z_Filter.setFilterFieldName("z");
  Cloud_z_Filter.setFilterLimits(filterZ[0], filterZ[1]);
  Cloud_z_Filter.setInputCloud(cloud);
  Cloud_z_Filter.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter;
  Cloud_Filter.setFilterFieldName("x");
  if (flipped)
    Cloud_Filter.setFilterLimits(-filterX[1], -filterX[0]);
  else
    Cloud_Filter.setFilterLimits(filterX[0], filterX[1]);
  Cloud_Filter.setInputCloud(cloud);
  Cloud_Filter.filter(*cloud);

  pcl::PointXYZRGB point1, point2;
  pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, point1, point2);

  pcl::PointXYZRGB translate;

  if (point1.x < point2.x && point1.z < point2.z)
    translate = point1;
  else
    translate = point2;

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  transform_1(0, 3) = -translate.x;
  transform_1(1, 3) = -translate.y;
  transform_1(2, 3) = -translate.z;

  pcl::transformPointCloud(*cloud, *cloud, transform_1);

  std::cout << cloud->points.size() << std::endl;

  // py::finalize_interpreter();
  return cloud;
}
