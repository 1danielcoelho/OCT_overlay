#ifndef FILEMANAGER_H
#define FILEMANAGER_H

/**
Helper class for many I/O and conversion functions
*/

#include "defines.h"

#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <vector>

#include <memory>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkTypeUInt8Array.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "octinfo.h"

class Crossbar {
 public:
  Crossbar();
  // Deletes all of our cache files
  void clearAllFiles();

  // Byte vectors
  void writeVector(std::vector<uint8_t>& input, const char* filepath,
                   bool append = false);
  void writeVector(std::vector<uint32_t>& input, const char* filepath,
                   bool append = false);
  void writeVector(std::vector<float>& input, const char* filepath,
                   bool append = false);
  void readVector(const char* filepath, std::vector<uint8_t>& output);
  void readVector(const char* filepath, std::vector<uint32_t>& output);
  void readVector(const char* filepath, std::vector<float>& output);

  // VTK
  void writePolyData(vtkSmartPointer<vtkPolyData> input, const char* filepath,
                     bool append = false);
  void readPolyData(const char* filepath, vtkSmartPointer<vtkPolyData> input);

  // PCL
  void writePCL(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                const char* filepath);
  void writePCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
                const char* filepath);
  void readPCL(const char* filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr input);
  void readPCL(const char* filepath,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);

  // Simple conversions
  void ucharVectorToPolyData(std::vector<uint8_t>& input, OCTinfo& params,
                             vtkSmartPointer<vtkPolyData> output);
  void PCLxyzToVTKPolyData(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                           vtkSmartPointer<vtkPolyData> output);
  void VTKPolyDataToPCLxyz(vtkSmartPointer<vtkPolyData> input,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr output);

  void PCLxyzrgbToVTKPolyData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
                              vtkSmartPointer<vtkPolyData> output);
  void VTKPolyDataToPCLxyzrgb(vtkSmartPointer<vtkPolyData> input,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);

  void intVectorToImageData2D(std::vector<uint32_t>& input,
                              vtkSmartPointer<vtkImageData> output);

  void imageData2DtoIntVector(vtkSmartPointer<vtkImageData> input,
                              std::vector<uint32_t>& output);

  void floatVectorToCvMat(std::vector<float> input, cv::Mat& output);
};

#endif  // FILEMANAGER_H
