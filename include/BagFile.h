//
// Created by ian on 26.11.2021.
//

#ifndef PCLOUDEXTRACT_BAGFILE_H
#define PCLOUDEXTRACT_BAGFILE_H

#include <vector>
#include <string>

#include "helper.h"

class BagFile {
public:
  explicit BagFile(std::string filename,
                   std::string matrix = "");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(int frame = 10,
                                                       double threshold = 1.7,
                                                       bool filter = true);
  std::string getSerial() const;

  int getCameraPos() const;
  void setCameraPos(int cameraPos);

private:
  std::string _serial;
  std::string _filename;
  std::string _matrix;

  int _cameraPos;

  std::vector<rs2::depth_frame> _depths;
  std::vector<rs2::video_frame> _colors;
  rs2_intrinsics _intrinsics;

};

#endif //PCLOUDEXTRACT_BAGFILE_H
