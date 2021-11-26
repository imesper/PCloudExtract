//
// Created by ian on 26.11.2021.
//

#ifndef PCLOUDEXTRACT_PCLHELPER_H
#define PCLOUDEXTRACT_PCLHELPER_H

#include <pcl/point_types.h>

void
toXYZ(float fx,
      float fy,
      float cx,
      float cy,
      int w,
      int h,
      float* Z,
      float* z,
      float* x,
      float* y);

void
toXYZRGB(float fx,
         float fy,
         float cx,
         float cy,
         int w,
         int h,
         float* Z,
         unsigned char* rgb,
         pcl::PointXYZRGB* points);

#endif //PCLOUDEXTRACT_PCLHELPER_H
