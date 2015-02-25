#ifndef FILEMANAGER_H
#define FILEMANAGER_H
#include "defines.h"

#include <stdio.h>
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


class FileManager
{
  public:
  FileManager();
  //Deletes all of our cache files
  void clearAllFiles();

  //Byte vectors
  void writeVector(std::vector<uint8_t>& input, const char* filepath,
      bool append = false);
  void writeVector(std::vector<uint32_t>& input, const char* filepath,
      bool append = false);
  void readVector(const char* filepath, std::vector<uint8_t>& output);
  void readVector(const char* filepath, std::vector<uint32_t>& output);

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
