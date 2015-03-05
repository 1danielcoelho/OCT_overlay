#include "crossbar.h"

Crossbar::Crossbar() {}

void Crossbar::clearAllFiles() {
  remove(OCT_RAW_CACHE_PATH);
  remove(OCT_SURF_CACHE_PATH);
  remove(OCT_MASS_CACHE_PATH);
  remove(STEREO_LEFT_CACHE_PATH);
  remove(STEREO_RIGHT_CACHE_PATH);
  remove(STEREO_DISP_CACHE_PATH);
  remove(STEREO_DEPTH_CACHE_PATH);
  remove(VIS_TRANS_CACHE_PATH);
}

void Crossbar::writeVector(std::vector<uint8_t> &input, const char *filepath,
                           bool append) {
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

  uint32_t bytes_written = fwrite(&(input[0]), 1, input.size(), output_file);

  fclose(output_file);

  // Check to see if we wrote everything
  if (bytes_written != input.size()) {
    std::cerr << "Could not write everything to " << filepath << std::endl;
  }
}

void Crossbar::writeVector(std::vector<uint32_t> &input, const char *filepath,
                           bool append) {
  std::FILE *output_file;
  if (append) {
    output_file = std::fopen(filepath, "ab");  // write, binary
  } else {
    output_file = std::fopen(filepath, "wb");  // append, binary
  }

  if (output_file == NULL) {
    std::cerr << "Could not write file at " << filepath << std::endl;
    return;
  }

  uint32_t bytes_written =
      fwrite(&(input[0]), sizeof(uint32_t), input.size(), output_file);

  fclose(output_file);

  // Check to see if we wrote everything
  if (bytes_written != input.size()) {
    std::cerr << "Could not write everything to " << filepath << std::endl;
  }
}

void Crossbar::writeVector(std::vector<float> &input, const char *filepath,
                           bool append) {
  std::FILE *output_file;
  if (append) {
    output_file = std::fopen(filepath, "ab");  // write, binary
  } else {
    output_file = std::fopen(filepath, "wb");  // append, binary
  }

  if (output_file == NULL) {
    std::cerr << "Could not write file at " << filepath << std::endl;
    return;
  }

  uint32_t bytes_written =
      fwrite(&(input[0]), sizeof(float), input.size(), output_file);

  fclose(output_file);

  // Check to see if we wrote everything
  if (bytes_written != input.size()) {
    std::cerr << "Could not write everything to " << filepath << std::endl;
  }
}

void Crossbar::readVector(const char *filepath, std::vector<uint8_t> &output) {
  std::FILE *input_file;
  input_file = std::fopen(filepath, "rb");

  if (input_file == NULL) {
    std::cerr << "Could not open file at " << filepath << std::endl;
    return;
  }

  // Get the file size
  std::fseek(input_file, 0, SEEK_END);
  int file_size = std::ftell(input_file);
  std::rewind(input_file);

  // Read the file into data
  output.resize(file_size);
  int bytes_read = std::fread(&(output[0]), 1, file_size, input_file);

  std::fclose(input_file);

  // Check to see if we read everything
  if (bytes_read != file_size) {
    std::cerr << "Could not read everything from " << filepath << std::endl;
  }
}

void Crossbar::readVector(const char *filepath, std::vector<uint32_t> &output) {
  std::FILE *input_file;
  input_file = std::fopen(filepath, "rb");

  if (input_file == NULL) {
    std::cerr << "Could not open file at " << filepath << std::endl;
    return;
  }

  // Get the file size
  std::fseek(input_file, 0, SEEK_END);
  int file_size = std::ftell(input_file);
  std::rewind(input_file);

  // Read the file into data
  output.resize(file_size);
  int bytes_read =
      std::fread(&(output[0]), sizeof(uint32_t), file_size, input_file);

  std::fclose(input_file);

  // Check to see if we read everything
  if ((bytes_read * sizeof(uint32_t)) != file_size) {
    std::cerr << "Could not read everything from " << filepath << std::endl;
  }
}


void Crossbar::readVector(const char *filepath, std::vector<float> &output) {
  std::FILE *input_file;
  input_file = std::fopen(filepath, "rb");

  if (input_file == NULL) {
    std::cerr << "Could not open file at " << filepath << std::endl;
    return;
  }

  // Get the file size
  std::fseek(input_file, 0, SEEK_END);
  int file_size = std::ftell(input_file);
  std::rewind(input_file);

  // Read the file into data
  output.resize(file_size);
  int bytes_read =
      std::fread(&(output[0]), sizeof(float), file_size, input_file);

  std::fclose(input_file);

  // Check to see if we read everything
  if ((bytes_read * sizeof(float)) != file_size) {
    std::cerr << "Could not read everything from " << filepath << std::endl;
  }
}

void Crossbar::writePolyData(vtkSmartPointer<vtkPolyData> input,
                             const char *filepath, bool append) {
  VTK_NEW(vtkXMLPolyDataWriter, writer);
  writer->SetFileName(filepath);
  writer->SetDataModeToBinary();

  if (append) {
    writer->SetDataModeToAppended();
  }

#if VTK_MAJOR_VERSION <= 5
  writer->SetInput(input);
#else
  writer->SetInputData(input);
#endif

  writer->Write();
}

void Crossbar::readPolyData(const char *filepath,
                            vtkSmartPointer<vtkPolyData> input) {
  VTK_NEW(vtkXMLPolyDataReader, reader);
  reader->SetFileName(filepath);
  reader->Update();

  input->ShallowCopy(reader->GetOutput());
}

void Crossbar::writePCL(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                        const char *filepath) {
  pcl::io::savePCDFileBinary(filepath, *input);
}

void Crossbar::writePCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
                        const char *filepath) {
  pcl::io::savePCDFileBinary(filepath, *input);
}

void Crossbar::readPCL(const char *filepath,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr input) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *input) == -1) {
    std::cerr << "Could not read " << filepath << std::endl;
  }
}

void Crossbar::readPCL(const char *filepath,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filepath, *input) == -1) {
    std::cerr << "Could not read " << filepath << std::endl;
  }
}

void Crossbar::ucharVectorToPolyData(std::vector<uint8_t> &input,
                                     OCTinfo &params,
                                     vtkSmartPointer<vtkPolyData> output) {
  assert("Input unsigned char vector is empty!" && input.size() > 0);
  assert("Output vtkPolyData smart pointer is NULL!" && output != NULL);

  // Determines the distance between consecutive points in each direction
  float length_incrm = 1;
  float width_incrm = 1;
  float depth_incrm = 2.762 / 1024.0;  // Our OCT has a fixed axial resolution.
                                       // If we have no depth range, this is our
                                       // best bet

  // In case we have some sample data or weird input, prevents null increments
  if (params.length_range != 0)
    length_incrm = params.length_range / params.length_steps;

  if (params.width_range != 0)
    width_incrm = params.width_range / params.width_steps;

  if (params.depth_range != 0)
    depth_incrm = params.depth_range / params.depth_steps;

  // Setup arrays to hold point coordinates and the scalars
  VTK_NEW(vtkPoints, points);
  VTK_NEW(vtkTypeUInt8Array, dataArray);
  points->SetNumberOfPoints(params.length_steps * params.width_steps *
                            params.depth_steps);
  dataArray->SetNumberOfValues(params.length_steps * params.width_steps *
                               params.depth_steps);

  // Set them into our polydata early, to potentially drop (and gracefully
  // delete) the arrays that previously were in those positions
  output->SetPoints(points);
  output->GetPointData()->SetScalars(dataArray);

  int id = 0;
  for (int i = 0; i < params.length_steps; i++) {
    for (int j = 0; j < params.width_steps; j++) {
      for (int k = 0; k < params.depth_steps; k++, id++) {
        points->SetPoint(id, i * length_incrm + params.length_offset,
                         j * width_incrm + params.width_offset,
                         k * depth_incrm);

        dataArray->SetValue(id, input[id]);
        //        std::cout << "id: " << id << "\t\tx: " << i*length_incrm <<
        // "\t\ty: "
        //        << j*width_incrm << "\t\tz: " << k*depth_incrm << "\t\tval: "
        // <<
        //        input[id] << std::endl;
      }
    }
  }
}

void Crossbar::PCLxyzToVTKPolyData(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                   vtkSmartPointer<vtkPolyData> output) {
  assert("Input XYZ point cloud is NULL!" && input != NULL);

  long num_pts = input->size();
  assert("Input XYZ point cloud has zero points!" && num_pts > 0);

  if (output == NULL) {
    output = vtkSmartPointer<vtkPolyData>::New();
  }

  VTK_NEW(vtkPoints, polydata_points);
  polydata_points->SetNumberOfPoints(num_pts);

  pcl::PointXYZ *pcl_point;

  for (long i = 0; i < num_pts; i++) {
    pcl_point = &(input->points[i]);

    // Intentionally inverts X and Y at this point. This is to compensate for
    // OCT_segmentation, at some point of the algorithm, inverting what it
    // considers "x" and "y". This is combined with another type of inversion
    // at QNode::requestSegmentation
    polydata_points->SetPoint(i, pcl_point->y, pcl_point->x, pcl_point->z);
  }

  output->SetPoints(polydata_points);
}

void Crossbar::VTKPolyDataToPCLxyz(vtkSmartPointer<vtkPolyData> input,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
  assert("Input vtkPolyData is NULL!" && input != NULL);

  long num_pts = input->GetNumberOfPoints();
  assert("Input vtkPolyData has zero points!" && num_pts > 0);

  if (output == NULL) {
    output =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }

  output->clear();
  output->resize(num_pts);

  double polydata_point[3];
  pcl::PointXYZ *pcl_point;

  for (long i = 0; i < num_pts; i++) {
    input->GetPoint(i, polydata_point);
    pcl_point = &(output->points[i]);

    pcl_point->x = polydata_point[0];
    pcl_point->y = polydata_point[1];
    pcl_point->z = polydata_point[2];
  }
}

void Crossbar::PCLxyzrgbToVTKPolyData(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    vtkSmartPointer<vtkPolyData> output) {
  assert("Input XYZRGB point cloud is NULL!" && input != NULL);

  long num_pts = input->size();
  assert("Input XYZRGB point cloud has zero points!" && num_pts > 0);

  if (output == NULL) {
    output = vtkSmartPointer<vtkPolyData>::New();
  }

  VTK_NEW(vtkPoints, polydata_points);
  polydata_points->SetNumberOfPoints(num_pts);

  VTK_NEW(vtkUnsignedCharArray, polydata_color_array);
  polydata_color_array->SetNumberOfComponents(3);
  polydata_color_array->SetNumberOfTuples(num_pts);
  polydata_color_array->SetName("Colors");

  pcl::PointXYZRGB *pcl_point;
  unsigned char polydata_color[3];

  for (long i = 0; i < num_pts; i++) {
    pcl_point = &(input->points[i]);

    polydata_color[0] = pcl_point->r;
    polydata_color[1] = pcl_point->g;
    polydata_color[2] = pcl_point->b;

    // Maybe invert x,y here like for XYZ PCL?
    polydata_points->SetPoint(i, pcl_point->x, pcl_point->y, pcl_point->z);
    polydata_color_array->SetTupleValue(i, polydata_color);
  }

  output->SetPoints(polydata_points);
  output->GetPointData()->SetScalars(polydata_color_array);
}

void Crossbar::VTKPolyDataToPCLxyzrgb(
    vtkSmartPointer<vtkPolyData> input,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output) {
  assert("Input vtkPolyData is NULL!" && input != NULL);

  long num_pts = input->GetNumberOfPoints();
  assert("Input vtkPolyData has zero points!" && num_pts > 0);

  if (output == NULL) {
    output = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>);
  }

  output->clear();
  output->resize(num_pts);

  pcl::PointXYZRGB *pcl_point;
  double polydata_point[3];
  double polydata_color[3];

  vtkDataArray* colors = input->GetPointData()->GetArray("Colors");

  for (long i = 0; i < num_pts; i++) {
    pcl_point = &(output->points[i]);
    input->GetPoint(i, polydata_point);
    colors->GetTuple(i, polydata_color);

    pcl_point->x = polydata_point[0];
    pcl_point->y = polydata_point[1];
    pcl_point->z = polydata_point[2];
    pcl_point->r = polydata_color[0];
    pcl_point->g = polydata_color[1];
    pcl_point->b = polydata_color[2];
  }
}

void Crossbar::intVectorToImageData2D(std::vector<uint32_t> &input,
                                      vtkSmartPointer<vtkImageData> output) {
  assert("Input int vector is empty!" && input.size() > 0);

  if (output == NULL) {
    output = vtkSmartPointer<vtkImageData>::New();
  }

  // Parse the first 8 bytes to determine dimensions
  uint32_t rows, cols;
  memcpy(&rows, &input[0], 4);
  memcpy(&cols, &input[1], 4);

  output->SetDimensions(cols, rows, 1);
  output->SetNumberOfScalarComponents(3);
  output->SetScalarTypeToUnsignedChar();
  output->AllocateScalars();

  for (uint32_t y = 0; y < rows; y++) {
    for (uint32_t x = 0; x < cols; x++) {
      // We need to invert the vertical coordinate since we use different
      // origins
      unsigned char *pixel = static_cast<unsigned char *>(
          output->GetScalarPointer(x, (rows - 1) - y, 0));

      // Skip the two 32-bit values (header)
      memcpy(&pixel[0], &(input[x + y * cols + 2]), 3);
    }
  }

  output->Modified();
}

void Crossbar::imageData2DtoIntVector(vtkSmartPointer<vtkImageData> input,
                                      std::vector<uint32_t> &output) {
  assert("Input vtkImageData is NULL!" && input != NULL);

  long num_pts = input->GetNumberOfPoints();

  assert("Input vtkImageData is empty!" && num_pts > 0);

  output.clear();
  output.resize(num_pts + 2);  // 2 bytes for the header

  int dimensions[3];
  input->GetDimensions(dimensions);

  uint32_t rows = dimensions[1];
  uint32_t cols = dimensions[2];
  memcpy(&output[0], &rows, 4);
  memcpy(&output[1], &cols, 4);

  for (uint32_t y = 0; y < rows; y++) {
    for (uint32_t x = 0; x < cols; x++) {
      // We need to invert the vertical coordinate since we use different
      // origins
      unsigned char *pixel = static_cast<unsigned char *>(
          input->GetScalarPointer(x, (rows - 1) - y, 0));

      // The two first uint32_t are the header
      memcpy(&(output[x + y * cols + 2]), &pixel[0], 3);
    }
  }
}

void Crossbar::floatVectorToCvMat(std::vector<float>& input, cv::Mat& output)
{
    assert("Input float vector is empty!" && input.size() > 0);

    float rows_f, cols_f;
    memcpy(&rows_f, &input[0], 4);
    memcpy(&cols_f, &input[1], 4);

    uint32_t rows = (int)rows_f;
    uint32_t cols = (int)cols_f;

    output.create(rows, cols, CV_32FC3);
    output = cv::Mat::zeros(rows, cols, CV_32FC3);

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            cv::Vec3f& color = output.at<cv::Vec3f>(j,i);

            color[0] = input[3*j + i*cols*3 + 2]; //skip 2 header floats
            color[1] = input[3*j+1 + i*cols*3 + 2];
            color[2] = input[3*j+2 + i*cols*3 + 2];
        }
    }
}

void Crossbar::floatVectorToImageData2D(std::vector<float>& input,
                                        vtkSmartPointer<vtkImageData> output)
{
    assert("Input float vector is empty!" && input.size() > 0);

    if (output == NULL) {
      output = vtkSmartPointer<vtkImageData>::New();
    }

    float rows_f, cols_f;
    memcpy(&rows_f, &input[0], 4);
    memcpy(&cols_f, &input[1], 4);

    uint32_t rows = (int)rows_f;
    uint32_t cols = (int)cols_f;

    output->SetDimensions(cols, rows, 1);
    output->SetNumberOfScalarComponents(3);
    output->SetScalarTypeToFloat();
    output->AllocateScalars();

    for(int y = 0; y < rows; y++)
    {
        for(int x = 0; x < cols; x++)
        {
            // We need to invert the vertical coordinate since we use different
            // origins
            float *pixel = static_cast<float *>(
                output->GetScalarPointer(x, (rows - 1) - y, 0));

            memcpy(&pixel[0], &(input[3*x + y*cols*3 + 2]), 3*sizeof(float));
        }
    }

    output->Modified();
}

void Crossbar::imageData2DToFloatVector(vtkSmartPointer<vtkImageData> input,
                                        std::vector<float>& output)
{
    assert("Input vtkImageData is NULL!" && input != NULL);

    long num_pts = input->GetNumberOfPoints();

    assert("Input vtkImageData is empty!" && num_pts > 0);

    output.clear();
    output.resize(3*num_pts + 2);  // 2 bytes for the header

    int dimensions[3];
    input->GetDimensions(dimensions);

    float rows = (float)dimensions[1];
    float cols = (float)dimensions[2];
    memcpy(&output[0], &rows, 4);
    memcpy(&output[1], &cols, 4);

    for (uint32_t y = 0; y < rows; y++) {
      for (uint32_t x = 0; x < cols; x++) {
        // We need to invert the vertical coordinate since we use different
        // origins
        float *pixel = static_cast<float *>(
            input->GetScalarPointer(x, (rows - 1) - y, 0));

        // The two first uint32_t are the header
        memcpy(&(output[3*x + y * cols*3 + 2]), &pixel[0], 3*sizeof(float));
      }
    }
}
