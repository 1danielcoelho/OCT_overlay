#include "form.h"
#include "ui_form.h"

Form::Form(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  m_ui(new Ui::Form),
  m_qnode(argc, argv)
{
  m_ui->setupUi(this);

  //Setup the cloud pointer
  m_raw_cloud.reset(new PointCloudT);
  m_vis_cloud.reset(new PointCloudT);

  //Has qnode's rosShutdown() signal activate MainWindow's close() method
  QObject::connect(&m_qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  //Update status bar
  this->m_ui->status_bar->showMessage("Ready");
  QApplication::processEvents();
}

Form::~Form()
{
  delete m_ui;
}

void Form::on_browse_button_clicked()
{
  //getOpenFileName displays a file dialog and returns the full file path of
  //the selected file, or an empty string if the user canceled the dialog
  //The tr() function makes the dialog language proof (chinese characters, etc)
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setNameFilter(tr("Image Files (*.img);;Text Files (*.txt)"));
  QString file_name = dialog.getOpenFileName(this);

  if(!file_name.isEmpty())
  {
    //Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    //Update status bar
    this->m_ui->status_bar->showMessage("Analysing file... ");
    QApplication::processEvents();

    //Create an input file stream
    std::ifstream input_file(file_name.toStdString().c_str());

    //If the file isn't open, show an error box
    if(!input_file)
    {
      QMessageBox::critical(this, tr("Error"), tr("Could not open file."));
      return;
    }

    //Get the file size
    input_file.seekg(0, std::ios::end);
    int file_size = input_file.tellg();
    input_file.seekg(0, std::ios::beg);

    //Read the file into data
    std::vector<uint8_t> data(file_size);
    input_file.read((char*) &data[0], file_size);

    uint32_t raw_length, raw_width, raw_depth;
    uint64_t checker = 0; //checks for a raw data text file, or a thorlabs img

    //Extract the data dimensions from the header
    std::memcpy(&raw_length, &(data[16]), 4*sizeof(uint8_t));
    std::memcpy(&raw_width, &(data[20]), 4*sizeof(uint8_t));
    std::memcpy(&raw_depth, &(data[24]), 4*sizeof(uint8_t));
    std::memcpy(&checker, &(data[72]), 8*sizeof(uint8_t));

    //Update status bar
    this->m_ui->status_bar->showMessage("Loading data into point cloud... ");
    QApplication::processEvents();

    if(checker == 0)
    {
      //Image has 0 scanWidth and scanLength, so it must be a manually generated
      //array, as opposed to a thorlabs oct *.img file. This means it has 0
      //bytes between each B-scan (frame header)
      this->loadDataIntoCloud(m_raw_cloud, data,
                            raw_length, raw_width, raw_depth, 0, 512);
    } else
    {
      //It's a thorlabs img file: They use 40 bytes of frame header
      this->loadDataIntoCloud(m_raw_cloud, data,
                            raw_length, raw_width, raw_depth, 40, 512);
    }

    //Displays the file name in the ui
    this->m_ui->file_name_lineedit->setText(file_name);

    //Renders the oct data, drop PCL interactors for VTK interactors
    m_viewer.reset(new pcl::visualization::PCLVisualizer ("m_viewer", false));
    m_ui->qvtkWidget->SetRenderWindow(m_viewer->getRenderWindow());
    m_viewer->setupInteractor(m_ui->qvtkWidget->GetInteractor(),
                              m_ui->qvtkWidget->GetRenderWindow());

    //Update status bar
    this->m_ui->status_bar->showMessage("Readying cloud for render... ");
    QApplication::processEvents();
    this->readyCloudForRender(m_raw_cloud);

    m_viewer->addPointCloud(m_vis_cloud, "m_raw_cloud");
    m_viewer->addCoordinateSystem(50.0, 0);
    m_viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "m_raw_cloud");
    //m_viewer->addCoordinateSystem(1.0);
    m_viewer->resetCamera();
    m_ui->qvtkWidget->update();
  }
}

void Form::loadDataIntoCloud(PointCloudT::Ptr cloud, std::vector<uint8_t>& data,
                       uint32_t length, uint32_t width, uint32_t depth,
                       uint32_t frame_header, uint32_t file_header)
{
  uint32_t index = 0;
  uint32_t num_points = length*width*depth;

  cloud->clear();
  cloud->resize(num_points);

  //Create an iterator over the vector we'll be copying into
  std::vector<PointT, Eigen::aligned_allocator<PointT> >::iterator iter =
      cloud->points.begin();

  QElapsedTimer timer;
  timer.start();

  for(uint32_t i = 0; i < length; i++)
  {
    for(uint32_t j = 0; j < width; j++)
    {
      for(uint32_t k = 0; k < depth; k++)
      {
        (*iter).x = i;
        (*iter).y = j;
        (*iter).z = k;
        (*iter).r = data[index +frame_header*i + file_header];

        iter++;
        index++;
      }
    }
  }

  qDebug() << timer.elapsed();
}



void Form::readyCloudForRender(PointCloudT::Ptr cloud)
{
  std::vector<PointT, Eigen::aligned_allocator<PointT> >* vis_points =
                                                        &(m_vis_cloud->points);
  std::vector<PointT, Eigen::aligned_allocator<PointT> >* cloud_points =
                                                        &(cloud->points);

  uint8_t rand_val = 0;
  int num_points = cloud_points->size();

  vis_points->clear();
  vis_points->reserve(num_points);

  for(std::vector<PointT, Eigen::aligned_allocator<PointT> >::iterator i =
    cloud_points->begin(); i != cloud_points->end(); i++)
  {
    rand_val = (*i).r;

    if(rand()%150 + 50 < rand_val)
    {
      m_vis_cloud->points.push_back(*i);
    }
  }


//  for(int i = 0; i < num_points; i++)
//  {
//    rand_val = cloud->points[i].r;

//    if(rand()%150 + 50 < rand_val)
//    {
//      m_vis_cloud->points.push_back(cloud->points[i]);
//    }
//  }
}
