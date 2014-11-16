#include "filemanager.h"

FileManager::FileManager()
{
}

void FileManager::writeVector(std::vector<uint8_t>& input, const char *filepath,
    bool append)
{
  std::FILE* output_file;
  if(append)
  {
    output_file = std::fopen(filepath, "ab"); //write, binary
  }
  else
  {
    output_file = std::fopen(filepath, "wb"); //append, binary
  }

  if(output_file == NULL)
  {
    std::cerr << "Could not write file at " << filepath << std::endl;
    return;
  }

  uint32_t bytes_written = fwrite(&(input[0]), 1, input.size(), output_file);

  fclose(output_file);

  //Check to see if we wrote everything
  if(bytes_written != input.size())
  {
    std::cerr << "Could not write everything to " << filepath << std::endl;
  }
}




void FileManager::writeVector(std::vector<uint32_t>& input,const char *filepath,
    bool append)
{
  std::FILE* output_file;
  if(append)
  {
    output_file = std::fopen(filepath, "ab"); //write, binary
  }
  else
  {
    output_file = std::fopen(filepath, "wb"); //append, binary
  }

  if(output_file == NULL)
  {
    std::cerr << "Could not write file at " << filepath << std::endl;
    return;
  }

  uint32_t bytes_written = fwrite(&(input[0]), sizeof(uint32_t), input.size(),
      output_file);

  fclose(output_file);

  //Check to see if we wrote everything
  if(bytes_written  != input.size())
  {
    std::cerr << "Could not write everything to " << filepath << std::endl;
  }
}




void FileManager::readVector(const char *filepath, std::vector<uint8_t> &output)
{
  std::FILE* input_file;
  input_file = std::fopen(filepath, "rb");

  if(input_file == NULL)
  {
    std::cerr << "Could not open file at " << filepath << std::endl;
    return;
  }

  //Get the file size
  std::fseek(input_file, 0, SEEK_END);
  int file_size = std::ftell(input_file);
  std::rewind(input_file);

  //Read the file into data
  output.resize(file_size);
  int bytes_read = std::fread(&(output[0]), 1, file_size, input_file);

  std::fclose(input_file);

  //Check to see if we read everything
  if(bytes_read != file_size)
  {
    std::cerr << "Could not read everything from " << filepath << std::endl;
  }

}




void FileManager::readVector(const char *filepath, std::vector<uint32_t> &output)
{
  std::FILE* input_file;
  input_file = std::fopen(filepath, "rb");

  if(input_file == NULL)
  {
    std::cerr << "Could not open file at " << filepath << std::endl;
    return;
  }

  //Get the file size
  std::fseek(input_file, 0, SEEK_END);
  int file_size = std::ftell(input_file);
  std::rewind(input_file);

  //Read the file into data
  output.resize(file_size);
  int bytes_read = std::fread(&(output[0]), sizeof(uint32_t), file_size,
      input_file);

  std::fclose(input_file);

  //Check to see if we read everything
  if((bytes_read * sizeof(uint32_t)) != file_size)
  {
    std::cerr << "Could not read everything from " << filepath << std::endl;
  }
}





void FileManager::writePolyData(vtkSmartPointer<vtkPolyData> input,
    const char *filepath, bool append)
{
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();

  writer->SetFileName(filepath);
  writer->SetDataModeToBinary();

  if(append)
  {
    writer->SetDataModeToAppended();
  }

#if VTK_MAJOR_VERSION <= 5
  writer->SetInput(input);
#else
  writer->SetInputData(input);
#endif

  writer->Write();
}




void FileManager::readPolyData(const char* filepath,
    vtkSmartPointer<vtkPolyData> input)
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader =
      vtkSmartPointer<vtkXMLPolyDataReader>::New();

  reader->SetFileName(filepath);
  reader->Update();

  input->DeepCopy(reader->GetOutput());
}




void FileManager::writePCL(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
    const char *filepath)
{
  pcl::io::savePCDFileBinary(filepath, *input);
}

void FileManager::writePCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    const char *filepath)
{
  pcl::io::savePCDFileBinary(filepath, *input);
}




void FileManager::readPCL(const char *filepath,
    pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filepath, *input) == -1)
  {
    std::cerr << "Could not read " << filepath << std::endl;
  }
}

void FileManager::readPCL(const char *filepath,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (filepath, *input) == -1)
  {
    std::cerr << "Could not read " << filepath << std::endl;
  }
}
