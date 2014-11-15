#ifndef FILEMANAGER_H
#define FILEMANAGER_H

#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <vector>

#include <memory>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#define CACHE_PATH "/home/daniel/Dropbox/Comum/Forschungsarbeit/ROS/Packages/OCT_overlay/"

#define OCT_RAW_CACHE_PATH CACHE_PATH "oct_raw.cache"
#define OCT_POLY_CACHE_PATH CACHE_PATH "oct_poly.cache"
#define OCT_MASS_CACHE_PATH CACHE_PATH "oct_mass.cache"
#define STEREO_PCL_CACHE_PATH CACHE_PATH "stereo_pcl.cache"
#define STEREO_POLY_CACHE_PATH CACHE_PATH "stereo_poly.cache"


class FileManager
{
  public:
  FileManager();

  //Byte vectors
  void writeVector(std::vector<uint8_t>& input, const char* filepath,
      bool append = false);
  void readVector(const char* filepath, std::vector<uint8_t>& output);

  //VTK
  void writePolyData(vtkSmartPointer<vtkPolyData> input, const char* filepath,
      bool append = false);
  void readPolyData(const char* filepath, vtkSmartPointer<vtkPolyData> input);

  //PCL
  void writePCL(pcl::PointCloud<pcl::PointXYZ>::Ptr input,const char* filepath);
  void writePCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
      const char* filepath);
  void readPCL(const char* filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr input);
  void readPCL(const char* filepath,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);
};

#endif // FILEMANAGER_H
