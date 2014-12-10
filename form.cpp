#include "form.h"
#include "ui_form.h"

Form::Form(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  m_ui(new Ui::Form)
{
  m_ui->setupUi(this);

  //Initialize the oct parameters
  m_min_vis_thresh = this->m_ui->raw_min_vis_spinbox->value();
  m_max_vis_thresh = this->m_ui->raw_max_vis_spinbox->value();
  m_current_params.length_steps  = this->m_ui->len_steps_spinbox->value();
  m_current_params.width_steps   = this->m_ui->wid_steps_spinbox->value();
  m_current_params.depth_steps   = this->m_ui->dep_steps_spinbox->value();
  m_current_params.length_range  = this->m_ui->len_range_spinbox->value();
  m_current_params.width_range   = this->m_ui->wid_range_spinbox->value();
  m_current_params.depth_range   = this->m_ui->dep_range_spinbox->value();
  m_current_params.length_offset = this->m_ui->len_off_spinbox->value();
  m_current_params.width_offset  = this->m_ui->wid_off_spinbox->value();

  //Disable some buttons until they can be pressed
  m_connected_to_master = false;
  m_has_ros_raw_oct = false;
  m_has_raw_oct = false;
  m_waiting_response = false;
  m_has_oct_surf = false;
  m_has_oct_mass = false;
  m_has_stereo_data = false;
  m_has_transform = false;
  updateUIStates();

  //Creates qnode and it's thread, connecting signals and slots
  m_qthread = new QThread;
  m_qnode = new QNode(argc, argv);
  m_qnode->moveToThread(m_qthread);

  //Allows us to use OCTinfo in signals/slots
  qRegisterMetaType<OCTinfo>();
  qRegisterMetaType<std::vector<uint8_t> >();

  //Connect signals and slots
  connect(m_qnode,  SIGNAL(rosMasterChanged(bool)),
          this,     SLOT(on_connected_master_checkbox_clicked(bool)));

  connect(this,     SIGNAL(requestScan(OCTinfo)),
          m_qnode,  SLOT(requestScan(OCTinfo)),
          Qt::QueuedConnection); //Runs slot on receiving thread

  connect(this,     SIGNAL(requestSegmentation(OCTinfo)),
          m_qnode,  SLOT(requestSegmentation(OCTinfo)),
          Qt::QueuedConnection);

  connect(this,     SIGNAL(requestRegistration()),
          m_qnode,  SLOT(requestRegistration()),
          Qt::QueuedConnection);

  connect(m_qnode,  SIGNAL(receivedOCTRawData(OCTinfo)),
          this,     SLOT(receivedRawOCTData(OCTinfo)),
          Qt::QueuedConnection);

  connect(m_qnode,  SIGNAL(receivedOCTSurfData(OCTinfo)),
          this,     SLOT(receivedOCTSurfData(OCTinfo)),
          Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedStereoData()),
          this,    SLOT(receivedStereoData()),
          Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedRegistration()),
          this,    SLOT(receivedRegistration()),
          Qt::QueuedConnection);

  //Wire up qnode and it's thread. Don't touch this unless absolutely necessary
  connect(m_qthread, SIGNAL(started()), m_qnode, SLOT(process()));
  connect(m_qnode, SIGNAL(finished()), m_qthread, SLOT(quit()));
  connect(m_qnode, SIGNAL(finished()), m_qthread, SLOT(deleteLater()));
  connect(m_qthread, SIGNAL(finished()), m_qthread, SLOT(deleteLater()));
  m_qthread->start();


  this->m_ui->status_bar->showMessage("Ready");
  QApplication::processEvents();

  //Instantiate vtk objects
  //Data structures
  m_oct_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_oct_stereo_trans = vtkSmartPointer<vtkTransform>::New();
  //Actors
  m_oct_vol_actor = vtkSmartPointer<vtkActor>::New();
  m_oct_surf_actor = vtkSmartPointer<vtkActor>::New();
  m_stereo_2d_actor = vtkSmartPointer<vtkActor2D>::New();
  //Others
  m_renderer = vtkSmartPointer<vtkRenderer>::New();

  m_renderer->SetBackground(0, 0, 0.1);
  m_renderer->SetBackground2(0, 0, 0.2);
  m_renderer->SetGradientBackground(1);

  //Adds our renderer to the QVTK widget
  this->m_ui->qvtkWidget->GetRenderWindow()->AddRenderer(m_renderer);
}

Form::~Form()
{
  delete m_ui;
  delete m_qnode;

  //Waits until the m_qnode's destructor has finished before killing the thread
  m_qthread->wait();
  delete m_qthread;

  //Delete all of our .cache files
  m_file_manager->clearAllFiles();
}

void Form::updateUIStates()
{
  //OCT page
  m_ui->connected_master_checkbox->setChecked(m_connected_to_master);
  m_ui->connected_master_checkbox_2->setChecked(m_connected_to_master);

  m_ui->browse_button->setEnabled(!m_waiting_response && !m_waiting_response);

  m_ui->request_scan_button->setEnabled(!m_waiting_response);
  m_ui->save_button->setEnabled(m_has_ros_raw_oct && !m_waiting_response);
  m_ui->reset_params_button->setEnabled(!m_waiting_response);

  m_ui->len_steps_spinbox->setEnabled(!m_waiting_response);
  m_ui->len_range_spinbox->setEnabled(!m_waiting_response);
  m_ui->len_off_spinbox->setEnabled(!m_waiting_response);
  m_ui->wid_steps_spinbox->setEnabled(!m_waiting_response);
  m_ui->wid_range_spinbox->setEnabled(!m_waiting_response);
  m_ui->wid_off_spinbox->setEnabled(!m_waiting_response);
  m_ui->dep_steps_spinbox->setEnabled(!m_waiting_response);
  m_ui->dep_range_spinbox->setEnabled(!m_waiting_response);

  m_ui->raw_min_vis_spinbox->setEnabled(!m_waiting_response);
  m_ui->raw_max_vis_spinbox->setEnabled(!m_waiting_response);
  m_ui->raw_min_vis_slider->setEnabled(!m_waiting_response);
  m_ui->raw_max_vis_slider->setEnabled(!m_waiting_response);

  m_ui->view_raw_oct_button->setEnabled(m_has_raw_oct && !m_waiting_response);
  m_ui->view_oct_surf_button->setEnabled(m_has_oct_surf && !m_waiting_response);
  m_ui->view_oct_mass_button->setEnabled(m_has_oct_mass && !m_waiting_response);

  m_ui->calc_oct_surf_button->setEnabled(m_has_raw_oct && !m_waiting_response);
  m_ui->calc_oct_mass_button->setEnabled(m_has_oct_surf && !m_waiting_response);

  //Stereocamera page
  m_ui->view_left_image_button->setEnabled(m_has_stereo_data &&
      !m_waiting_response);
  m_ui->view_right_image_button->setEnabled(m_has_stereo_data &&
      !m_waiting_response);
  m_ui->view_disp_image_button->setEnabled(m_has_stereo_data &&
      !m_waiting_response);
  m_ui->view_depth_image_button->setEnabled(m_has_stereo_data &&
      !m_waiting_response);

  //Visualization page
  m_ui->oct_surf_loaded_checkbox->setChecked(m_has_oct_surf);
  m_ui->oct_mass_loaded_checkbox->setChecked(m_has_oct_mass);
  m_ui->stereocamera_surf_loaded_checkbox->setChecked(m_has_stereo_data);

  m_ui->view_oct_vol_oct_surf->setEnabled(m_has_oct_surf && m_has_raw_oct);

  m_ui->over_min_vis_spinbox->setEnabled(!m_waiting_response);
  m_ui->over_max_vis_spinbox->setEnabled(!m_waiting_response);
  m_ui->over_min_vis_slider->setEnabled(!m_waiting_response);
  m_ui->over_max_vis_slider->setEnabled(!m_waiting_response);

  m_ui->calc_transform_button->setEnabled(m_has_oct_surf && m_has_stereo_data &&
      !m_waiting_response);
  m_ui->print_transform_button->setEnabled(m_has_transform &&
      !m_waiting_response);

  m_ui->view_simple_overlay_button->setEnabled(m_has_transform && m_has_oct_surf
      && m_has_stereo_data && !m_waiting_response);

  m_ui->view_complete_overlay_button->setEnabled(m_has_transform &&
      m_has_oct_surf && m_has_stereo_data && m_has_oct_mass &&
      !m_waiting_response);
}

//---------------INPUT----------------------------------------------------------

void Form::processOCTHeader(std::vector<uint8_t>&full_array)
{
    std::memcpy(&m_current_params.length_steps, &(full_array[16]),  4);
    std::memcpy(&m_current_params.width_steps,  &(full_array[20]),  4);
    std::memcpy(&m_current_params.depth_steps,  &(full_array[24]),  4);

    std::memcpy(&m_current_params.length_range, &(full_array[72]),  4);
    std::memcpy(&m_current_params.width_range,  &(full_array[76]),  4);
    std::memcpy(&m_current_params.depth_range,  &(full_array[116]), 4);

    std::memcpy(&m_current_params.length_offset,&(full_array[120]), 4);
    std::memcpy(&m_current_params.width_offset, &(full_array[124]), 4);

    //Checks the datetime string. The Thorlabs software always fills this, but
    //we don't, so we use this to check whether we have one of their img files
    uint32_t checker, frame_header = 0;
    std::memcpy(&checker,                       &(full_array[44]),  4);
    if(checker == 0) //Not a thorlabs .img file, so get frame_header from array
    {
      std::memcpy(&frame_header,                &(full_array[132]), 4);
    }
    else //Thorlabs image, so we use their default 40 bytes frame_header
    {
      frame_header = 40;
    }

    std::vector<uint8_t> result;
    uint32_t b_scan_size = m_current_params.width_steps *
                           m_current_params.depth_steps;
    uint32_t b_scan_count = 0;

    result.resize(m_current_params.length_steps * b_scan_size);

    std::cout << "ls: "   << m_current_params.length_steps
              << "\nws: " << m_current_params.width_steps
              << "\nds: " << m_current_params.depth_steps
              << "\nlr: " << m_current_params.length_range
              << "\nwr: " << m_current_params.width_range
              << "\ndr: " << m_current_params.depth_range
              << "\nlo: " << m_current_params.length_offset
              << "\nwo: " << m_current_params.width_offset << std::endl;

    //Clears the file and frame headers out of full_array
    for(std::vector<uint8_t>::iterator iter=full_array.begin()+OCT_HEADER_BYTES;
        iter != full_array.end(); ++b_scan_count)
    {
      std::copy(iter, iter+b_scan_size, &(result[b_scan_count*b_scan_size]));
      iter += b_scan_size + frame_header;
    }
    result.swap(full_array);
}


void Form::loadVectorToPolyData(std::vector<uint8_t>& oct_data)
{
  //Determines the distance between consecutive points in each direction
  float length_incrm = 1;
  float width_incrm = 1;
  float depth_incrm = 2.762/1024.0; //Fixed axial resolution. If we have no
                                    //depth range, this is our best bet

  if(m_current_params.length_range != 0)
    length_incrm = m_current_params.length_range/m_current_params.length_steps;

  if(m_current_params.width_range != 0)
    width_incrm = m_current_params.width_range/m_current_params.width_steps;

  if(m_current_params.depth_range != 0)
    depth_incrm = m_current_params.depth_range/m_current_params.depth_steps;

  VTK_NEW(vtkPoints, points);
  VTK_NEW(vtkTypeUInt8Array, dataArray);
  points->SetNumberOfPoints(   m_current_params.length_steps *
                               m_current_params.width_steps *
                               m_current_params.depth_steps);
  dataArray->SetNumberOfValues(m_current_params.length_steps *
                               m_current_params.width_steps *
                               m_current_params.depth_steps);

  this->statusBar()->showMessage("Building raw point data... ");
  QApplication::processEvents();

  int id = 0;
  for(int i = 0; i < m_current_params.length_steps; i++)
  {
    for(int j = 0; j < m_current_params.width_steps; j++)
    {
      for(int k = 0; k < m_current_params.depth_steps; k++, id++)
      {        
        points->SetPoint(id, i*length_incrm + m_current_params.length_offset,
                             j*width_incrm  + m_current_params.width_offset,
                             k*depth_incrm);
        dataArray->SetValue(id, oct_data[id]);
//        std::cout << "id: " << id << "\t\tx: " << i*length_incrm << "\t\ty: "
//        << j*width_incrm << "\t\tz: " << k*depth_incrm << "\t\tval: " <<
//        oct_data[id] << std::endl;
      }
    }


    this->statusBar()->showMessage("Building raw point data... " +
        QString::number(i) + " of " +
        QString::number(m_current_params.length_steps));

    QApplication::processEvents();
  }

  m_oct_poly_data->Reset();
  m_oct_poly_data->SetPoints(points);
  m_oct_poly_data->GetPointData()->SetScalars(dataArray);

  m_has_raw_oct = true;
  updateUIStates();

  this->statusBar()->showMessage("Building raw point data... done!");
  QApplication::processEvents();
}


void Form::loadPCLCacheToPolyData(const char* file_path,
    vtkSmartPointer<vtkPolyData> cloud_poly_data)
{
  //Points have no color component
  if(file_path == OCT_SURF_CACHE_PATH)
  {

    this->statusBar()->showMessage("Reading OCT surface PCL cache... ");
    QApplication::processEvents();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(
          new pcl::PointCloud<pcl::PointXYZ>);

    //Reads the file into our pts point cloud
    m_file_manager->readPCL(file_path, pts);

    int num_pts = pts->size();

    VTK_NEW(vtkPoints, points);
    points->Reset();
    points->SetNumberOfPoints(num_pts);

    pcl::PointXYZ* cloud_point;


    this->statusBar()->showMessage("Building OCT surface vtkPolyData... ");
    QApplication::processEvents();

    for(int i = 0; i < num_pts; i++)
    {      
      cloud_point = &(pts->at(i));
      points->SetPoint(i, cloud_point->y, cloud_point->x, cloud_point->z); //INVERTED X AND Y
    }

    cloud_poly_data->Reset();
    cloud_poly_data->SetPoints(points);

    this->statusBar()->showMessage("Building OCT surface vtkPolyData... done!");
    QApplication::processEvents();
  }

  //Points have color component
  else if (file_path == STEREO_DEPTH_CACHE_PATH)
  {

    this->statusBar()->showMessage("Reading depth map PCL cache... ");
    QApplication::processEvents();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts(
          new pcl::PointCloud<pcl::PointXYZRGB>);

    //Reads the file into our pts point cloud
    m_file_manager->readPCL(file_path, pts);
    int num_pts = pts->size();

    VTK_NEW(vtkPoints, points);
    points->Reset();
    points->SetNumberOfPoints(num_pts);

    VTK_NEW(vtkUnsignedCharArray, data_array);
    data_array->Reset();
    data_array->SetNumberOfComponents(3);
    data_array->SetNumberOfTuples(num_pts);
    data_array->SetName("Colors");

    pcl::PointXYZRGB* cloud_point;
    unsigned char color[3] = {0, 0, 0};


    this->statusBar()->showMessage("Building depth map vtkPolyData... ");
    QApplication::processEvents();

    //Iterate over our pts
    for(int i = 0; i < num_pts; i++)
    {
      cloud_point = &(pts->at(i));

      points->SetPoint(i, cloud_point->x, cloud_point->y, cloud_point->z);

      color[0] = cloud_point->r;
      color[1] = cloud_point->g;
      color[2] = cloud_point->b;
      data_array->SetTupleValue(i, color);
    }

    cloud_poly_data->Reset();
    cloud_poly_data->SetPoints(points);
    cloud_poly_data->GetPointData()->SetScalars(data_array);

    this->statusBar()->showMessage("Building depth map vtkPolyData... done!");
    QApplication::processEvents();
  }
  else
  {
    qDebug() << "Invalid file path for loadPCLCacheToPolyData";
    return;
  }
}


void Form::load2DVectorCacheToImageData(const char* file_path,
                             vtkSmartPointer<vtkImageData> image_data)
{
  //Create a vector to hold the data
  std::vector<uint32_t> data;

  //Read the entire file
  m_file_manager->readVector(file_path, data);

  //Parse the first 8 bytes to determine dimensions
  uint32_t rows, cols;
  memcpy(&rows,  &data[0],  4);
  memcpy(&cols,  &data[1],  4);

  image_data->SetDimensions(cols, rows, 1);
  image_data->SetNumberOfScalarComponents(3);
  image_data->SetScalarTypeToUnsignedChar();
  image_data->AllocateScalars();

  uint32_t val;
  for(uint32_t y = 0; y < rows; y++)
  {
    for(uint32_t x = 0; x < cols; x++)
    {
      unsigned char* pixel = static_cast<unsigned char*>
                             (image_data->GetScalarPointer(x, y, 0));

      //The two first uint32_t are the header
      val = data[x + y*cols + 2];
      memcpy(&pixel[0],  &val,  3*sizeof(uint8_t));
    }
  }

  image_data->Modified();
}

//------------PROCESSING--------------------------------------------------------

void Form::medianFilter2D(std::vector<uint8_t>& input)
{
  std::vector<double> rol;
  rol.resize(9);
  std::vector<uint8_t> filtered_data;
  filtered_data.resize(input.size());

  int depth = m_current_params.depth_steps;
  int width = m_current_params.width_steps;
  int length = m_current_params.length_steps;
  int index = 0;

  QString indicator;

  for(int i = 1; i < length-1; i++)
  {
      for(int j = 1; j < width -1; j++)
      {
          for(int k = 1; k < depth -1; k++)
          {
              index = i*(width*depth) + j*depth + k;

              rol[0] = input[index - depth - 1];
              rol[1] = input[index - depth];
              rol[2] = input[index - depth + 1];

              rol[3] = input[index - 1];
              rol[4] = input[index];
              rol[5] = input[index + 1];

              rol[6] = input[index + depth - 1];
              rol[7] = input[index + depth];
              rol[8] = input[index + depth + 1];

              std::sort(rol.begin(), rol.end());

              filtered_data[index] = rol[4];
          }
      }
      this->statusBar()->showMessage(
          QString("Applying 2D median filter... B-scan ") + indicator.number(i)
          + QString(" of ") + indicator.number(length-1));
      QApplication::processEvents();
  }

  input.swap(filtered_data);
}


void Form::medianFilter3D(std::vector<uint8_t>& input)
{
  std::vector<double> rol;
  rol.resize(27);
  std::vector<uint8_t> filtered_data;
  filtered_data.resize(input.size());

  int depth = m_current_params.depth_steps;
  int width = m_current_params.width_steps;
  int length = m_current_params.length_steps;
  uint32_t index = 0;

  QString indicator;

  for(int i = 1; i < length-1; i++)
  {
      for(int j = 1; j < width -1; j++)
      {
          for(int k = 1; k < depth -1; k++)
          {
              index = i*(width*depth) + j*depth +k;

              //Center point
              rol[0]  = input[index];

              //Points in the same B-scan
              rol[1]  = input[index + 1];                       //down
              rol[2]  = input[index - 1];                       //up
              rol[3]  = input[index + depth];                   //forwards
              rol[4]  = input[index + depth + 1];
              rol[5]  = input[index + depth - 1];
              rol[6]  = input[index - depth];                   //back
              rol[7]  = input[index - depth + 1];
              rol[8]  = input[index - depth - 1];

              //Points in the next B-scan
              rol[9]  = input[index + depth*width];//left
              rol[10] = input[index + depth*width + 1];
              rol[11] = input[index + depth*width - 1];
              rol[12] = input[index + depth*width + depth];
              rol[13] = input[index + depth*width + depth + 1];
              rol[14] = input[index + depth*width + depth - 1];
              rol[15] = input[index + depth*width - depth];
              rol[16] = input[index + depth*width - depth + 1];
              rol[17] = input[index + depth*width - depth - 1];

              //Points in the previous B-scan
              rol[18] = input[index - depth*width];//right
              rol[19] = input[index - depth*width + 1];
              rol[20] = input[index - depth*width - 1];
              rol[21] = input[index - depth*width + depth];
              rol[22] = input[index - depth*width + depth + 1];
              rol[23] = input[index - depth*width + depth - 1];
              rol[24] = input[index - depth*width - depth];
              rol[25] = input[index - depth*width - depth + 1];
              rol[26] = input[index - depth*width - depth - 1];

              std::sort(rol.begin(), rol.end());

              filtered_data[index] = rol[13];
          }
      }


      this->statusBar()->showMessage(
          QString("Applying 3D median filter... B-scan ") + indicator.number(i)
          + QString(" of ") + indicator.number(length-1));
      QApplication::processEvents();
  }
  input.swap(filtered_data);
}


void Form::discardTop(std::vector<uint8_t> &input, float fraction_to_discard)
{
  int depth = m_current_params.depth_steps;
  int width = m_current_params.width_steps;
  int length = m_current_params.length_steps;

  int discard_max = depth*fraction_to_discard;

  for(int i = 0; i < length; i++)
  {
    for(int j = 0; j < width; j++)
    {
      for(int k = 0; k < discard_max; k++)
      {
        input[k + j*depth + i*depth*width] = 0;
      }
    }
  }

  std::cout << "Done discarding\n";
}


void Form::normalize(std::vector<uint8_t> &input)
{
  int depth = m_current_params.depth_steps;
  int width = m_current_params.width_steps;
  int length = m_current_params.length_steps;
  int index;
  int max_value = 0;

  std::vector<uint32_t> values;
  values.resize(256);

  //Build a histogram of the greyscale values of the whole image
  for(int i = 0; i < length; i++)
  {
    for(int j = 0; j < width; j++)
    {
      for(int k = 0; k < depth; k++)
      {
        index = input[k + j*depth + i*depth*width];
        values[index] += 1;

        if(index > max_value)max_value = index;
      }
    }
  }

  //Print said histogram to console
  for(std::vector<uint32_t>::iterator iter=values.begin();
      iter != values.end(); ++iter)
  {
    std::cout << "For value \t" << (unsigned int)(iter - values.begin())<<
        ":\t"<< (unsigned int)*iter << std::endl;
  }

  std::cout << "Rescaling by " << (int) max_value << std::endl;

  //Rescales every value to the range [0,255]
  for(int i = 0; i < length; i++)
  {
    for(int j = 0; j < width; j++)
    {
      for(int k = 0; k < depth; k++)
      {
        input[k + j*depth + i*depth*width] *= 255.0/max_value;
      }
    }
  }
}

//------------RENDERING---------------------------------------------------------

void Form::renderAxes()
{
  VTK_NEW(vtkAxesActor, axes_actor);
  axes_actor->SetNormalizedShaftLength(1.0, 1.0, 1.0);
  axes_actor->SetShaftTypeToCylinder();
  axes_actor->SetCylinderRadius(0.02);
  axes_actor->SetXAxisLabelText("length        ");
  axes_actor->SetYAxisLabelText("width (B-Scan)");
  axes_actor->SetZAxisLabelText("depth (A-Scan)");
  axes_actor->GetXAxisCaptionActor2D()->GetTextActor()->
              SetTextScaleModeToNone();
  axes_actor->GetYAxisCaptionActor2D()->GetTextActor()->
              SetTextScaleModeToNone();
  axes_actor->GetZAxisCaptionActor2D()->GetTextActor()->
              SetTextScaleModeToNone();
  axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
              SetFontSize(15);
  axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
              SetFontSize(15);
  axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
              SetFontSize(15);
  axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
  axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
  axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
  axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
  axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
  axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
  axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
              SetColor(1.0, 0.0, 0.0);
  axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
              SetColor(0.0, 1.0, 0.0);
  axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
              SetColor(0.0, 0.0, 1.0);

  m_renderer->AddActor(axes_actor);

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();
}


void Form::renderOCTVolumePolyData()
{
  uint32_t num_pts = m_oct_poly_data->GetPointData()->GetNumberOfTuples();

  if(num_pts == 0)
  {
    qDebug() << "m_oct_poly_data is empty!";
    return;
  }

  m_waiting_response = true;
  updateUIStates();

  vtkPoints* old_points = m_oct_poly_data->GetPoints();
  vtkTypeUInt8Array* old_data_array = vtkTypeUInt8Array::SafeDownCast(
                m_oct_poly_data->GetPointData()->GetScalars());

  VTK_NEW(vtkPoints, new_points);
  VTK_NEW(vtkTypeUInt8Array, new_data_array);
  uint8_t value;

  this->statusBar()->showMessage("Preparing PolyData for display... ");
  QApplication::processEvents();

  std::cout << "min: " << (unsigned int)m_min_vis_thresh << ", max: " << (unsigned int)m_max_vis_thresh << std::endl;

  for(uint32_t i = 0; i < num_pts; i++)
  {
    value = old_data_array->GetValue(i);
//    std::cout << "x: " << (old_points->GetPoint(i))[0] <<
//          "\t\ty: " << (old_points->GetPoint(i))[1] <<
//          "\t\tz: " << (old_points->GetPoint(i))[2] <<
//          "\t\tI: " << (unsigned int)value << std::endl;

    if(value >= m_min_vis_thresh && value <= m_max_vis_thresh)
    {
      new_points->InsertNextPoint(old_points->GetPoint(i));
      new_data_array->InsertNextValue(value);
    }
  }


  this->statusBar()->showMessage("Preparing PolyData for display... done!");
  QApplication::processEvents();

  //Restores to original state, releases memory
  VTK_NEW(vtkPolyData, vis_poly_data);
  vis_poly_data->SetPoints(new_points);
  vis_poly_data->GetPointData()->SetScalars(new_data_array);

  VTK_NEW(vtkVertexGlyphFilter, vert_filter);
  vert_filter->SetInput(vis_poly_data);

  VTK_NEW(vtkPolyDataMapper, mapper);
  mapper->SetInputConnection(vert_filter->GetOutputPort());
  mapper->SetScalarVisibility(1);

  m_oct_vol_actor->SetMapper(mapper);

  m_renderer->AddActor(m_oct_vol_actor);

  this->statusBar()->showMessage("Rendering... ");
  QApplication::processEvents();

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();

  this->statusBar()->showMessage("Rendering... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}


void Form::renderPolyDataSurface(vtkSmartPointer<vtkPolyData> cloud_poly_data)
{
  int num_pts = cloud_poly_data->GetNumberOfPoints();

  if(num_pts == 0)
  {
    qDebug() << "cloud_poly_data is empty!";
    return;
  }

  VTK_NEW(vtkDelaunay2D, delaunay_filter);
  delaunay_filter->SetInput(cloud_poly_data);
  delaunay_filter->SetTolerance(0.001);

  VTK_NEW(vtkPolyDataMapper, mapper);
  mapper->SetInputConnection(delaunay_filter->GetOutputPort());

  m_oct_surf_actor->SetMapper(mapper);

  m_renderer->AddActor(m_oct_surf_actor);

  this->statusBar()->showMessage("Rendering PolyData surface... ");
  QApplication::processEvents();

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();

  this->statusBar()->showMessage("Rendering PolyData surface... done!");
  QApplication::processEvents();
}


void Form::render2DImageData(vtkSmartPointer<vtkImageData> image_data)
{
  int* window_sizes = this->m_ui->qvtkWidget->GetRenderWindow()->GetSize();
  double window_width, window_height;
  window_width = window_sizes[0];
  window_height = window_sizes[1];

  double window_aspect_ratio = window_width/window_height;

  int* image_sizes = image_data->GetExtent();
  double image_width, image_height;
  image_width = image_sizes[1];
  image_height = image_sizes[3];

  double image_aspect_ratio = image_width/image_height;

  //Positions and rescales the image to show the largest possible size with the
  //same aspect ratio
  double scaling = 1;
  if(window_aspect_ratio >= image_aspect_ratio)
  {
    scaling = image_height/window_height;
    m_stereo_2d_actor->SetPosition((window_width - image_width/scaling)/2, 0);
  }
  else
  {
    scaling = image_width/window_width;
    m_stereo_2d_actor->SetPosition(0, (window_height-image_height/scaling)/2);
  }

  VTK_NEW(vtkImageReslice, image_resize_filter);
  image_resize_filter->SetInputConnection(image_data->GetProducerPort());
  image_resize_filter->SetOutputSpacing(scaling, scaling, 1.0);

  VTK_NEW(vtkImageMapper, image_mapper);
  image_mapper->SetInputConnection(image_resize_filter->GetOutputPort());
  image_mapper->SetColorWindow(255.0);
  image_mapper->SetColorLevel(127.5);

  m_stereo_2d_actor->SetMapper(image_mapper);

  m_renderer->RemoveAllViewProps();
  m_renderer->AddActor2D(m_stereo_2d_actor);

  this->statusBar()->showMessage("Rendering 2D Image... ");
  QApplication::processEvents();

  this->m_ui->qvtkWidget->update();

  this->statusBar()->showMessage("Rendering 2D Image... done!");
  QApplication::processEvents();
}


void Form::renderOverlay(vtkSmartPointer<vtkPolyData> oct_surface,
    vtkSmartPointer<vtkPolyData> depth_map)
{
  int num_pts_oct = oct_surface->GetPointData()->GetNumberOfTuples();
  int num_pts_stereo = depth_map->GetPointData()->GetNumberOfTuples();

  if(num_pts_oct == 0 || num_pts_stereo)
  {
    qDebug() << "Either oct_surface or depth_map are empty!";
    return;
  }


  this->statusBar()->showMessage(
        "Preparing vtkPolyData objects for overlay display... ");
  QApplication::processEvents();


  //OCT
  {
    VTK_NEW(vtkTransformFilter, trans_filter);
    trans_filter->SetInput(oct_surface);
    trans_filter->SetTransform(m_oct_stereo_trans);

    VTK_NEW(vtkVertexGlyphFilter, vert_filter);
    vert_filter->RemoveAllInputs();
    vert_filter->SetInputConnection(trans_filter->GetOutputPort());

    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(vert_filter->GetOutputPort());

    m_oct_vol_actor->SetMapper(mapper);
    m_oct_vol_actor->GetProperty()->SetColor(0, 0, 1); //blue
  }

  //Stereocamera depthmap
  {
    VTK_NEW(vtkVertexGlyphFilter, vert_filter);
    vert_filter->RemoveAllInputs();
    vert_filter->SetInput(depth_map);

    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(vert_filter->GetOutputPort());

    m_oct_surf_actor->SetMapper(mapper);
    m_oct_surf_actor->GetProperty()->SetColor(1, 0, 0); //red
  }

  m_renderer->RemoveAllViewProps();
  m_renderer->AddActor(m_oct_vol_actor);
  m_renderer->AddActor(m_oct_surf_actor);
  m_renderer->ResetCamera();

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();
}

//--------------UI CALLBACKS----------------------------------------------------

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
    m_waiting_response = true;
    updateUIStates();

    //Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    std::vector<uint8_t> data;
    m_file_manager->readVector(file_name.toStdString().c_str(), data);    

    this->m_ui->status_bar->showMessage("Loading data into point cloud... ");
    QApplication::processEvents();

    //Data opened with the browse button should always already be processed
    //Just interpret the header, nothing else
    processOCTHeader(data);

    //Immediately write our headerless vector to cache so we can segment
    //Or register using this data
    m_file_manager->writeVector(data, OCT_RAW_CACHE_PATH);

    loadVectorToPolyData(data);

    m_renderer->RemoveAllViewProps();
    renderOCTVolumePolyData();
    m_renderer->ResetCamera();
    renderAxes();

    //Updates our UI param boxes
    on_reset_params_button_clicked();
    this->m_ui->file_name_lineedit->setText(file_name);

    m_has_ros_raw_oct = false;
    m_waiting_response = false;
    updateUIStates();
  }
}


void Form::on_connected_master_checkbox_clicked(bool checked)
{
  m_connected_to_master = checked;
  updateUIStates();
}


void Form::on_dep_steps_spinbox_editingFinished()
{
  //The axial sensor has a fixed resolution, so we change m_dep_range to match
  this->m_ui->dep_range_spinbox->setValue(
      (float)this->m_ui->dep_steps_spinbox->value()/1024.0*2.762);
}


void Form::on_dep_range_spinbox_editingFinished()
{
  //The axial sensor has a fixed resolution, so we change m_dep_steps to match
  this->m_ui->dep_steps_spinbox->setValue((int) (
      this->m_ui->dep_range_spinbox->value()/2.762*1024 + 0.5));
}


void Form::on_request_scan_button_clicked()
{
  //Disables controls until we get a response
  m_waiting_response = true;
  updateUIStates();

  //Gets the updated values
  m_current_params.length_steps = this->m_ui->len_steps_spinbox->value();
  m_current_params.width_steps = this->m_ui->wid_steps_spinbox->value();
  m_current_params.depth_steps = this->m_ui->dep_steps_spinbox->value();
  m_current_params.length_range = this->m_ui->len_range_spinbox->value();
  m_current_params.width_range = this->m_ui->wid_range_spinbox->value();
  m_current_params.depth_range = this->m_ui->dep_range_spinbox->value();
  m_current_params.length_offset = this->m_ui->len_off_spinbox->value();
  m_current_params.width_offset = this->m_ui->wid_off_spinbox->value();

  //Updates our UI param boxes
  on_reset_params_button_clicked();

  //Pack the desired OCT params into a struct
  Q_EMIT requestScan(m_current_params);


  this->m_ui->status_bar->showMessage("Waiting for OCT scanner response... ");
  QApplication::processEvents();
}


void Form::on_save_button_clicked()
{
  QString file_name = QFileDialog::getSaveFileName(this, tr("Save as img file"),
      "", tr("Image file (*.img)"));

  if(!file_name.isEmpty())
  {
    m_waiting_response = true;
    updateUIStates();

    //Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);


    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    std::vector<uint8_t> header;
    header.resize(512);
    memcpy(&header[16],  &m_current_params.length_steps,  4);
    memcpy(&header[20],  &m_current_params.width_steps,   4);
    memcpy(&header[24],  &m_current_params.depth_steps,   4);
    memcpy(&header[72],  &m_current_params.length_range,  4);
    memcpy(&header[76],  &m_current_params.width_range,   4);
    memcpy(&header[116], &m_current_params.depth_range,   4);
    memcpy(&header[120], &m_current_params.length_offset, 4);
    memcpy(&header[124], &m_current_params.width_offset,  4);

    m_file_manager->writeVector(header, file_name.toStdString().c_str());

    //Write the actual data
    std::vector<uint8_t> data;
    m_file_manager->readVector(OCT_RAW_CACHE_PATH, data);
    m_file_manager->writeVector(data, file_name.toStdString().c_str(), true);

    m_waiting_response = false;
    updateUIStates();


    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_raw_min_vis_spinbox_editingFinished()
{
  uint8_t new_value = m_ui->raw_min_vis_spinbox->value();
  m_min_vis_thresh = new_value;
  m_ui->raw_min_vis_slider->setValue(new_value);  

  m_ui->over_min_vis_slider->setValue(new_value);

  //Adjusts the max slider if we need to
  if(new_value > m_max_vis_thresh)
      m_ui->raw_max_vis_slider->setValue(new_value);

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();
  renderAxes();
}

void Form::on_raw_max_vis_spinbox_editingFinished()
{
  uint8_t new_value = m_ui->raw_max_vis_spinbox->value();
  m_max_vis_thresh = new_value;
  m_ui->raw_max_vis_slider->setValue(new_value);

  m_ui->over_max_vis_slider->setValue(new_value);

  //Adjusts the min slider if we need to
  if(new_value < m_min_vis_thresh)
      m_ui->raw_min_vis_slider->setValue(new_value);

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();
  renderAxes();
}

void Form::on_raw_min_vis_slider_valueChanged(int value)
{
  m_min_vis_thresh = value;
  m_ui->raw_min_vis_spinbox->setValue(value);

  m_ui->over_min_vis_slider->setValue(value);

  //Adjusts the max slider if we need to
  if(value > m_max_vis_thresh)
      m_ui->raw_max_vis_slider->setValue(value);
}

void Form::on_raw_max_vis_slider_valueChanged(int value)
{
  m_max_vis_thresh = value;
  m_ui->raw_max_vis_spinbox->setValue(value);  

  m_ui->over_max_vis_slider->setValue(value);

  //Adjusts the min slider if we need to
  if(value < m_min_vis_thresh)
      m_ui->raw_min_vis_slider->setValue(value);

}

void Form::on_raw_min_vis_slider_sliderReleased()
{
    m_renderer->RemoveAllViewProps();
    renderOCTVolumePolyData();
    renderAxes();
}

void Form::on_raw_max_vis_slider_sliderReleased()
{
    m_renderer->RemoveAllViewProps();
    renderOCTVolumePolyData();
    renderAxes();
}

void Form::on_over_min_vis_slider_valueChanged(int value)
{
    m_min_vis_thresh = value;
    m_ui->over_min_vis_spinbox->setValue(value);

    m_ui->raw_min_vis_slider->setValue(value);

    //Adjusts the max slider if we need to
    if(value > m_max_vis_thresh)
        m_ui->over_max_vis_slider->setValue(value);
}

void Form::on_over_max_vis_slider_valueChanged(int value)
{
    m_max_vis_thresh = value;
    m_ui->over_max_vis_spinbox->setValue(value);

    m_ui->raw_max_vis_slider->setValue(value);

    //Adjusts the min slider if we need to
    if(value < m_min_vis_thresh)
        m_ui->over_min_vis_slider->setValue(value);
}

void Form::on_over_min_vis_slider_sliderReleased()
{
    m_renderer->RemoveActor(m_oct_vol_actor);
    renderOCTVolumePolyData();
}

void Form::on_over_max_vis_slider_sliderReleased()
{
    m_renderer->RemoveActor(m_oct_vol_actor);
    renderOCTVolumePolyData();
}

void Form::on_over_min_vis_spinbox_editingFinished()
{
    uint8_t new_value = m_ui->over_min_vis_spinbox->value();
    m_min_vis_thresh = new_value;
    m_ui->over_min_vis_slider->setValue(new_value);

    m_ui->raw_min_vis_slider->setValue(new_value);

    //Adjusts the max slider if we need to
    if(new_value > m_max_vis_thresh)
        m_ui->over_max_vis_slider->setValue(new_value);

    m_renderer->RemoveActor(m_oct_vol_actor);
    renderOCTVolumePolyData();
}

void Form::on_over_max_vis_spinbox_editingFinished()
{
    uint8_t new_value = m_ui->over_max_vis_spinbox->value();
    m_max_vis_thresh = new_value;
    m_ui->over_max_vis_slider->setValue(new_value);

    m_ui->raw_max_vis_slider->setValue(new_value);

    //Adjusts the min slider if we need to
    if(new_value < m_min_vis_thresh)
        m_ui->over_min_vis_slider->setValue(new_value);

    m_renderer->RemoveActor(m_oct_vol_actor);
    renderOCTVolumePolyData();
}

void Form::on_view_raw_oct_button_clicked()
{
  this->m_ui->status_bar->showMessage("Rendering OCT volume data... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();
  renderAxes();

  this->m_ui->status_bar->showMessage("Rendering OCT volume data... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();

}


void Form::on_calc_oct_surf_button_clicked()
{
  //Disables controls until we get a response
  m_waiting_response = true;
  updateUIStates();

  Q_EMIT requestSegmentation(m_current_params);

  this->m_ui->status_bar->showMessage("Waiting for OCT surface response... ");
  QApplication::processEvents();
}


void Form::on_view_left_image_button_clicked()
{  
  this->m_ui->status_bar->showMessage("Rendering left image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkImageData, left_image);

  load2DVectorCacheToImageData(STEREO_LEFT_CACHE_PATH, left_image);

  render2DImageData(left_image);

  this->m_ui->status_bar->showMessage("Rendering left image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}


void Form::on_view_right_image_button_clicked()
{
  this->m_ui->status_bar->showMessage("Rendering right image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkImageData, right_image);

  load2DVectorCacheToImageData(STEREO_RIGHT_CACHE_PATH, right_image);

  render2DImageData(right_image);

  this->m_ui->status_bar->showMessage("Rendering right image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}


void Form::on_view_disp_image_button_clicked()
{
  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkImageData, disp_image);

  load2DVectorCacheToImageData(STEREO_DISP_CACHE_PATH, disp_image);

  render2DImageData(disp_image);

  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}


void Form::on_view_depth_image_button_clicked()
{
  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkPolyData, depth_image);

  loadPCLCacheToPolyData(STEREO_DEPTH_CACHE_PATH, depth_image);

  m_renderer->RemoveAllViewProps();
  renderPolyDataSurface(depth_image);
  renderAxes();

  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}


void Form::on_reset_params_button_clicked()
{
  this->m_ui->len_steps_spinbox->setValue(m_current_params.length_steps);
  this->m_ui->wid_steps_spinbox->setValue(m_current_params.width_steps);
  this->m_ui->dep_steps_spinbox->setValue(m_current_params.depth_steps);
  this->m_ui->len_range_spinbox->setValue(m_current_params.length_range);
  this->m_ui->wid_range_spinbox->setValue(m_current_params.width_range);
  this->m_ui->dep_range_spinbox->setValue(m_current_params.depth_range);
  this->m_ui->len_off_spinbox->setValue(m_current_params.length_offset);
  this->m_ui->wid_off_spinbox->setValue(m_current_params.width_offset);
}


void Form::on_calc_transform_button_clicked()
{
  //Disables controls until we get a response
  m_waiting_response = true;
  updateUIStates();

  Q_EMIT requestRegistration();


  this->m_ui->status_bar->showMessage("Waiting for OCT registration "
      "transform... ");
  QApplication::processEvents();
}


void Form::on_print_transform_button_clicked()
{

  this->m_ui->status_bar->showMessage("Printing transform to console... ");
  QApplication::processEvents();

  for(int row = 0; row < 4; row++)
  {
    std::cout << "\t";
    for(int col = 0; col < 4; col++)
    {
      std::cout << m_oct_stereo_trans->GetMatrix()->GetElement(row, col)<< "\t";
    }
    std::cout << "\n";
  }
}


void Form::on_view_oct_surf_button_clicked()
{

  this->m_ui->status_bar->showMessage("Rendering OCT surface... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkPolyData, surf_oct_poly_data);

  loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);

  m_renderer->RemoveAllViewProps();
  renderPolyDataSurface(surf_oct_poly_data);
  renderAxes();

  this->m_ui->status_bar->showMessage("Rendering OCT surface... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_oct_vol_oct_surf_clicked()
{
  this->m_ui->status_bar->showMessage("Rendering OCT raw data + OCT surface"
                                      " overlay... ");
  QApplication::processEvents();

  m_renderer->RemoveAllViewProps();

  VTK_NEW(vtkPolyData, surf_oct_poly_data);
  loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);
  renderPolyDataSurface(surf_oct_poly_data);

  renderOCTVolumePolyData();

  renderAxes();

  this->m_ui->status_bar->showMessage("Rendering OCT raw data + OCT surface"
                                      " overlay... done!");
  QApplication::processEvents();
}


void Form::on_view_simple_overlay_button_clicked()
{

  this->m_ui->status_bar->showMessage("Rendering simple surface overlay... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  //Read the oct surface and depth maps
  VTK_NEW(vtkPolyData, oct_surface);
  VTK_NEW(vtkPolyData, depth_map);
  loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, oct_surface);
  loadPCLCacheToPolyData(STEREO_DEPTH_CACHE_PATH, depth_map);

  //Render both simultaneously
  renderOverlay(oct_surface, depth_map);


  this->m_ui->status_bar->showMessage("Rendering simple surface overlay... "
      "done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

//------------QNODE CALLBACKS---------------------------------------------------

void Form::receivedRawOCTData(OCTinfo params)
{  
  this->m_ui->status_bar->showMessage("Waiting for OCT scanner response... "
      "done!");
  QApplication::processEvents();

  //Loads m_qnode's data vector into m_oct_poly_data
  std::vector<uint8_t> raw_data;
  m_file_manager->readVector(OCT_RAW_CACHE_PATH, raw_data);

  //Load updated values. These get sent all the way from the actual OCT scan
  //performed, so it's a way of debugging
  m_current_params.length_steps = params.length_steps;
  m_current_params.width_steps = params.width_steps;
  m_current_params.depth_steps = params.depth_steps;
  m_current_params.length_range = params.length_range;
  m_current_params.width_range = params.width_range;
  m_current_params.depth_range = params.depth_range;
  m_current_params.length_offset = params.length_offset;
  m_current_params.width_offset = params.width_offset;

  discardTop(raw_data, 0.1);
  medianFilter3D(raw_data);
  normalize(raw_data);

  //Write our new, filtered vector to our cache file
  m_file_manager->writeVector(raw_data, OCT_RAW_CACHE_PATH);

  loadVectorToPolyData(raw_data);

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();
  m_renderer->ResetCamera();
  renderAxes();

  //Re-enables controls for a potential new scan
  m_has_raw_oct = true;
  m_has_ros_raw_oct = true;
  m_waiting_response = false;
  updateUIStates();
}


void Form::receivedOCTSurfData(OCTinfo params)
{  
  this->m_ui->status_bar->showMessage("Waiting for OCT surface response... "
                                      "done!");
  QApplication::processEvents();

  //qnode has written an OCT surface to cache, so we read it
  VTK_NEW(vtkPolyData, surf_oct_poly_data);
  loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);

  m_renderer->RemoveAllViewProps();
  renderPolyDataSurface(surf_oct_poly_data);
  m_renderer->ResetCamera();
  renderAxes();

  //Re-enables controls for a potential new scan
  m_has_oct_surf = true;
  m_waiting_response = false;
  updateUIStates();
}


void Form::receivedStereoData()
{
  m_has_stereo_data = true;
  updateUIStates();
}


void Form::receivedRegistration()
{
  //Read and parse the YAML 4x4 transform
  cv::Mat cv_matrix;
  m_oct_stereo_trans->Identity();
  double elements[16];

  cv::FileStorage file_stream(VIS_TRANS_CACHE_PATH, cv::FileStorage::READ );
  if(file_stream.isOpened())
  {
    file_stream["octRegistrationMatrix"] >> cv_matrix;
    for( unsigned int row = 0; row < 4; row++ )
    {
      for( unsigned int col = 0; col< 4; col++ )
      {
        elements[row*4 + col] = cv_matrix.at<float>(row,col);
      }
    }
  }
  m_oct_stereo_trans->SetMatrix(elements);

  m_has_transform = true;
  updateUIStates();
}

















