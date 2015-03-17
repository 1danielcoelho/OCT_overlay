#ifndef FILEMANAGER_H
#define FILEMANAGER_H

/**
Helper class for many I/O and conversion functions
*/

#include "defines.h"

#include <stdio.h>
#include <iostream>
#include <iostream>
#include <fstream>
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
#include <vtkTransform.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "octinfo.h"

class Crossbar {
 public:
  Crossbar();
  // Deletes all of our cache files
  void clearAllFiles();

  // Vectors to binary files
  template<typename T>
  void writeVector(std::vector<T>& input, const char* filepath,
                   bool append = false)
  {
      assert("Can't write input vector: No elements!" && input.size() != 0);

      std::FILE *output_file;
      if (append) {
        output_file = std::fopen(filepath, "ab");  // append, binary
      } else {
        output_file = std::fopen(filepath, "wb");  // write, binary
      }

      if (output_file == NULL) {
        std::cerr << "Could not write file at " << filepath << std::endl;
        return;
      }

      uint32_t elements_written = fwrite(&(input[0]), sizeof(T), input.size(),
                                    output_file);

      fclose(output_file);

      // Check to see if we wrote everything
      if (elements_written != input.size()) {
        std::cerr << "Could not write everything to " << filepath << std::endl;
      }
  }

  template<typename T>
  void readVector(const char* filepath, std::vector<T>& output)
  {
      std::FILE *input_file;
      input_file = std::fopen(filepath, "rb");

      if (input_file == NULL) {
        std::cerr << "Could not open file at " << filepath << std::endl;
        return;
      }

      // Get the file size in bytes
      std::fseek(input_file, 0, SEEK_END);
      int file_size = std::ftell(input_file);
      std::rewind(input_file);

      // Get the number of elements in the file
      uint32_t element_count = file_size / sizeof(T);

      // Read the file into data
      output.resize(element_count);
      uint32_t elements_read = std::fread(&(output[0]), sizeof(T),
                                          element_count, input_file);

      std::fclose(input_file);

      // Check to see if we read everything
      if (elements_read != element_count) {
        std::cerr << "Could not read everything from " << filepath <<
                     std::endl;
      }
  }

  // VTK
  void writePolyData(vtkSmartPointer<vtkPolyData> input, const char* filepath,
                     bool append = false);
  void readPolyData(const char* filepath, vtkSmartPointer<vtkPolyData> input);
  void writeTransform(vtkSmartPointer<vtkTransform> input,
                      const char* filepath);
  void readTransform(const char* filepath,
                     vtkSmartPointer<vtkTransform> output);

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

  void floatVectorToCvMat(std::vector<float>& input, cv::Mat& output);

  void floatVectorToImageData2D(std::vector<float>& input,
                                vtkSmartPointer<vtkImageData> output);

  void imageData2DToFloatVector(vtkSmartPointer<vtkImageData> output,
                                std::vector<float>& input);
  void imageData2DToPolyData(vtkSmartPointer<vtkImageData> input,
                             vtkSmartPointer<vtkPolyData> output);
  void cvMatToPolyData(cv::Mat& input, vtkSmartPointer<vtkPolyData> output);
};

#endif  // FILEMANAGER_H
