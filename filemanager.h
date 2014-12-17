#ifndef FILEMANAGER_H
#define FILEMANAGER_H

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

//Size of the header in bytes that we use for the raw OCT vector
#define OCT_HEADER_BYTES 512

//Folder where the .cache files will end up. Leave empty for project bin folder
#define CACHE_PATH ""

#define OCT_RAW_CACHE_PATH CACHE_PATH "oct_raw.cache"
#define OCT_SURF_CACHE_PATH CACHE_PATH "oct_surf.cache"
#define OCT_MASS_CACHE_PATH CACHE_PATH "oct_mass.cache"
#define STEREO_LEFT_CACHE_PATH CACHE_PATH "stereo_left.cache"
#define STEREO_RIGHT_CACHE_PATH CACHE_PATH "stereo_right.cache"
#define STEREO_DISP_CACHE_PATH CACHE_PATH "stereo_disp.cache"
#define STEREO_DEPTH_CACHE_PATH CACHE_PATH "stereo_depth.cache"
#define VIS_TRANS_CACHE_PATH CACHE_PATH "vis_trans.cache"
        //"/opt/imesLS_ROS/laser_interface_new/octCamRegistration.yaml"


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
