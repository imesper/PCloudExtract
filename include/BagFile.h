//
// Created by ian on 26.11.2021.
//

#ifndef PCLOUDEXTRACT_BAGFILE_H
#define PCLOUDEXTRACT_BAGFILE_H

#include <string>
#include <vector>

#include "helper.h"

class BagFile {
public:
  explicit BagFile(std::string filename, std::string matrix = "");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  getPointCloud(int frame = 10, bool flipped = false, bool filter = true,
                const std::vector<float> &filterX = {-0.5, 1.0},
                const std::vector<float> &filterZ = {0.5, 1.0});
  std::string getSerial() const;

private:
  std::string _serial;
  std::string _filename;

  std::vector<rs2::depth_frame> _depths;
  std::vector<rs2::video_frame> _colors;
  rs2_intrinsics _intrinsics;
};

#endif // PCLOUDEXTRACT_BAGFILE_H
