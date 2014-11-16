#include "form.h"
#include "ui_form.h"



Form::Form(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  m_ui(new Ui::Form)
{
  m_ui->setupUi(this);

  //Initialize the oct parameters
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
  m_has_stereo_pcl = false;
  updateUIStates();

  //Creates qnode and it's thread, connecting signals and slots
  m_qthread = new QThread;
  m_qnode = new QNode(argc, argv);
  m_qnode->moveToThread(m_qthread);

  //Allows us to use OCTinfo in signals/slots
  qRegisterMetaType<OCTinfo>();
  qRegisterMetaType<std::vector<uint8_t> >();

  //Connect signals and slots
  connect(m_qnode, SIGNAL(rosMasterChanged(bool)), this,
          SLOT(on_connected_master_checkbox_clicked(bool)));

  connect(this,SIGNAL(requestScan(OCTinfo)),m_qnode, SLOT(requestScan(OCTinfo)),
          Qt::QueuedConnection); //Runs slot on receiving thread

  connect(this,SIGNAL(requestSegmentation(OCTinfo,std::vector<uint8_t>)),
          m_qnode, SLOT(requestSegmentation(OCTinfo,std::vector<uint8_t>)),
          Qt::QueuedConnection); //Sadly you need to pass-by-value to run in a
                                 //different thread

  connect(m_qnode, SIGNAL(receivedOCTRawData(OCTinfo)), this,
          SLOT(receivedRawOCTData(OCTinfo)), Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedOCTSurfData(OCTinfo)), this,
          SLOT(receivedOCTSurfData(OCTinfo)), Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedStereoData()), this,
          SLOT(receivedStereoData()), Qt::QueuedConnection);

  connect(m_qthread, SIGNAL(started()), m_qnode, SLOT(process()));
  connect(m_qnode, SIGNAL(finished()), m_qthread, SLOT(quit()));
  connect(m_qnode, SIGNAL(finished()), m_qthread, SLOT(deleteLater()));
  connect(m_qthread, SIGNAL(finished()), m_qthread, SLOT(deleteLater()));
  m_qthread->start();

  //Update status bar
  this->m_ui->status_bar->showMessage("Ready");
  QApplication::processEvents();

  //Instantiate vtk objects
  //Data structures
  m_raw_oct_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_vis_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_stereo_depth_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_stereo_left_image = vtkSmartPointer<vtkImageData>::New();
  m_stereo_right_image = vtkSmartPointer<vtkImageData>::New();
  m_stereo_disp_image = vtkSmartPointer<vtkImageData>::New();
  //Filters
  m_vert_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  m_image_resize_filter = vtkSmartPointer<vtkImageReslice>::New();
  //Mappers
  m_poly_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  m_image_mapper = vtkSmartPointer<vtkImageMapper>::New();
  //Actors
  m_actor = vtkSmartPointer<vtkActor>::New();
  m_axes_actor = vtkSmartPointer<vtkAxesActor>::New();
  m_image_actor = vtkSmartPointer<vtkActor2D>::New();
  //Others
  m_renderer = vtkSmartPointer<vtkRenderer>::New();

  m_renderer->SetBackground(0, 0, 0);
  //m_renderer->SetBackground2(0.1, 0.1, 0.1);
  //m_renderer->SetGradientBackground(1);

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
}





void Form::loadRawOCTData(std::vector<uint8_t>& oct_data, OCTinfo params/*=0*/,
      int file_header /*= 512*/, int frame_header /*= 40*/)
{
  //If we get in here it means we're opening a Thorlabs img file
  if(file_header == 512 && params == default_oct_info)
  {
    //Extract the data dimensions from the header
    std::memcpy(&params.length_steps,  &(oct_data[16]),  4*sizeof(uint8_t));
    std::memcpy(&params.width_steps,   &(oct_data[20]),  4*sizeof(uint8_t));
    std::memcpy(&params.depth_steps,   &(oct_data[24]),  4*sizeof(uint8_t));
    std::memcpy(&params.length_range,  &(oct_data[72]),  4*sizeof(uint8_t));
    std::memcpy(&params.width_range,   &(oct_data[76]),  4*sizeof(uint8_t));
    std::memcpy(&params.depth_range,   &(oct_data[116]), 4*sizeof(uint8_t));
    std::memcpy(&params.length_offset, &(oct_data[120]), 4*sizeof(uint8_t));
    std::memcpy(&params.width_offset,  &(oct_data[124]), 4*sizeof(uint8_t));
  }
  //If not, then we take the params out of the OCTinfo struct that was passed
  else if(file_header == 0 && !(params == default_oct_info))
  {
    params.length_steps = params.length_steps;
    params.width_steps  = params.width_steps;
    params.depth_steps  = params.depth_steps;
    params.length_range = params.length_range;
    params.width_range  = params.width_range;
    params.depth_range  = params.depth_range;
    params.length_offset= params.length_offset;
    params.width_offset = params.width_offset;
  }
  else
  {
    std::cerr << "Invalid input to loadRawOCTData!" << std::endl;
    return;
  }

  //Checks for the datetime string: If it exists, this is a thorlabs img
  //file, and the frameheader needs to be 40. It should be 0 otherwise
  uint32_t checker = 0;
  std::memcpy(&checker, &(oct_data[44]), 4*sizeof(uint8_t));
  if(checker == 0) //Still 0
  {
    frame_header = 0;
  }

  //Determines the distance between consecutive points in each direction
  float length_incrm = 1;
  float width_incrm = 1;
  float depth_incrm = 2.762/1024.0; //Fixed axial resolution. If we have no
                                    //depth range, this is our best bet

  if(params.length_range != 0)
    length_incrm = params.length_range/params.length_steps;

  if(params.width_range != 0)
    width_incrm = params.width_range/params.width_steps;

  if(params.depth_range != 0)
    depth_incrm = params.depth_range/params.depth_steps;

  //Creates an array for point coordinates, and one for the scalars
  VTK_NEW(vtkPoints, points);
  points->Reset();
  VTK_NEW(vtkTypeUInt8Array, dataArray);
  dataArray->Reset();

  points->SetNumberOfPoints(   params.length_steps *
                               params.width_steps *
                               params.depth_steps);

  dataArray->SetNumberOfValues(params.length_steps *
                               params.width_steps *
                               params.depth_steps);

  //Update status bar
  this->statusBar()->showMessage("Building raw point data... ");
  QApplication::processEvents();

  int id = 0;
  for(int i = 0; i < params.length_steps; i++)
  {
    for(int j = 0; j < params.width_steps; j++)
    {
      for(int k = 0; k < params.depth_steps; k++, id++)
      {
        //Thorlabs img files use 40 bytes of NULL between each B-scan
        int val = oct_data[id + frame_header*i + file_header];

        points->SetPoint(id, i*length_incrm + params.length_offset,
                             j*width_incrm + params.width_offset,
                             k*depth_incrm);
        dataArray->SetValue(id,val);
        //std::cout << "id: " << id << "\t\tx: " << i*length_incrm << "\t\ty: "
        //<< j*width_incrm << "\t\tz: " << k*depth_incrm << "\t\tval: " <<
        //val << std::endl;
      }
    }

    //Update status bar
    this->statusBar()->showMessage("Building raw point data... " +
        QString::number(i) + " of " +
        QString::number(params.length_steps));

    QApplication::processEvents();
  }

  //Abandon all previous data
  m_raw_oct_poly_data->Reset();

  //Add scalar data and point data
  m_raw_oct_poly_data->SetPoints(points);
  m_raw_oct_poly_data->GetPointData()->SetScalars(dataArray);

  //Set our state
  m_has_raw_oct = true;
  updateUIStates();

  //Update status bar
  this->statusBar()->showMessage("Building raw point data... done!");
  QApplication::processEvents();
}




void Form::renderRawOCTData()
{
  //Get the number of points
  uint32_t num_pts = m_raw_oct_poly_data->GetPointData()->GetNumberOfTuples();

  if(num_pts == 0)
  {
    qDebug() << "m_raw_oct_poly_data is empty!";
    return;
  }

  vtkPoints* old_points = m_raw_oct_poly_data->GetPoints();
  vtkTypeUInt8Array* old_data_array = vtkTypeUInt8Array::SafeDownCast(
                m_raw_oct_poly_data->GetPointData()->GetScalars());

  VTK_NEW(vtkPoints, new_points);
  VTK_NEW(vtkTypeUInt8Array, new_data_array);
  uint8_t value;

  //Update status bar
  this->statusBar()->showMessage("Preparing PolyData for display... ");
  QApplication::processEvents();

  for(uint32_t i = 0; i < num_pts; i++)
  {
    value = old_data_array->GetValue(i);
    if(value > VIS_THRESHOLD)
    {
      new_points->InsertNextPoint(old_points->GetPoint(i));
      new_data_array->InsertNextValue(value);

//      std::cout << "x: " << (old_points->GetPoint(i))[0] <<
//          "\t\ty: " << (old_points->GetPoint(i))[1] <<
//          "\t\tz: " << (old_points->GetPoint(i))[2] <<
//          "\t\tI: " << (int)value << std::endl;
    }
  }

  //Update status bar
  this->statusBar()->showMessage("Preparing PolyData for display... done!");
  QApplication::processEvents();

  //Restores to original state, releases memory
  m_vis_poly_data->Reset();

  m_vis_poly_data->SetPoints(new_points);
  m_vis_poly_data->GetPointData()->SetScalars(new_data_array);

  m_vert_filter->SetInput(m_vis_poly_data);

  m_poly_mapper->SetInputConnection(m_vert_filter->GetOutputPort());
  m_poly_mapper->SetScalarVisibility(1);

  m_actor->SetMapper(m_poly_mapper);

  //Reset the renderer
  m_renderer->RemoveAllViewProps();

  m_renderer->AddActor(m_actor);
  m_renderer->ResetCamera();

  //Fancies up our axes_actor
  m_axes_actor->SetNormalizedShaftLength(1.0, 1.0, 1.0);
  m_axes_actor->SetShaftTypeToCylinder();
  m_axes_actor->SetCylinderRadius(0.02);
  m_axes_actor->SetXAxisLabelText("length        ");
  m_axes_actor->SetYAxisLabelText("width (B-Scan)");
  m_axes_actor->SetZAxisLabelText("depth (A-Scan)");
  m_axes_actor->GetXAxisCaptionActor2D()->GetTextActor()->
      SetTextScaleModeToNone();
  m_axes_actor->GetYAxisCaptionActor2D()->GetTextActor()->
      SetTextScaleModeToNone();
  m_axes_actor->GetZAxisCaptionActor2D()->GetTextActor()->
      SetTextScaleModeToNone();
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(15);
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(15);
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(15);
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetItalic(0);
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetItalic(0);
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetItalic(0);
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetShadow(0);
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetShadow(0);
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetShadow(0);
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetColor(1.0, 0.0, 0.0);
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetColor(0.0, 1.0, 0.0);
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetColor(0.0, 0.0, 1.0);

  m_renderer->AddActor(m_axes_actor);

  //Update status bar
  this->statusBar()->showMessage("Rendering... ");
  QApplication::processEvents();

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();

  //Update status bar
  this->statusBar()->showMessage("Rendering... done!");
  QApplication::processEvents();
}




void Form::loadPCLStereoDepthMap()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts(
        new pcl::PointCloud<pcl::PointXYZRGB>);

  //Update status bar
  this->statusBar()->showMessage("Reading stereocam PCL... ");
  QApplication::processEvents();

  //Reads the file into our pts point cloud
  m_file_manager->readPCL(STEREO_DEPTH_CACHE_PATH, pts);

  VTK_NEW(vtkPoints, points);
  points->Reset();
  VTK_NEW(vtkUnsignedCharArray, data_array);
  data_array->Reset();

  int num_pts = pts->size();

  //Pre-allocate the space for our data
  points->SetNumberOfPoints(num_pts);
  data_array->SetNumberOfComponents(3);
  data_array->SetNumberOfTuples(num_pts);
  data_array->SetName("Colors");

  pcl::PointXYZRGB* cloud_point;
  unsigned char color[3] = {0, 0, 0};

  //Update status bar
  this->statusBar()->showMessage("Building depth map poly data... ");
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

  m_stereo_depth_poly_data->Reset();
  m_stereo_depth_poly_data->SetPoints(points);
  m_stereo_depth_poly_data->GetPointData()->SetScalars(data_array);

  //Set our state
  m_has_stereo_pcl = true;
  updateUIStates();

  //Update status bar
  this->statusBar()->showMessage("Building depth map poly data... done!");
  QApplication::processEvents();
}




void Form::renderPCLStereoDepthMap()
{
  int num_pts = m_stereo_depth_poly_data->GetPointData()->GetNumberOfTuples();

  if(num_pts == 0)
  {
    qDebug() << "m_stereo_depth_poly_data is empty!";
    return;
  }

  //Update status bar
  this->statusBar()->showMessage("Preparing PolyData for display... ");
  QApplication::processEvents();

  m_vis_poly_data->Reset();

  m_vis_poly_data->SetPoints(m_stereo_depth_poly_data->GetPoints());
  m_vis_poly_data->GetPointData()->SetScalars(
      m_stereo_depth_poly_data->GetPointData()->GetScalars());

  m_vert_filter->SetInput(m_vis_poly_data);

  m_poly_mapper->SetInputConnection(m_vert_filter->GetOutputPort());
  //m_poly_mapper->SetScalarVisibility(1);

  m_actor->SetMapper(m_poly_mapper);

  //Reset the renderer
  m_renderer->RemoveAllViewProps();

  m_renderer->AddActor(m_actor);
  m_renderer->ResetCamera();

  //Fancies up our axes_actor
  m_axes_actor->SetNormalizedShaftLength(1.0, 1.0, 1.0);
  m_axes_actor->SetShaftTypeToCylinder();
  m_axes_actor->SetCylinderRadius(0.02);
  m_axes_actor->SetXAxisLabelText("length        ");
  m_axes_actor->SetYAxisLabelText("width (B-Scan)");
  m_axes_actor->SetZAxisLabelText("depth (A-Scan)");
  m_axes_actor->GetXAxisCaptionActor2D()->GetTextActor()->
      SetTextScaleModeToNone();
  m_axes_actor->GetYAxisCaptionActor2D()->GetTextActor()->
      SetTextScaleModeToNone();
  m_axes_actor->GetZAxisCaptionActor2D()->GetTextActor()->
      SetTextScaleModeToNone();
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(15);
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(15);
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(15);
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetItalic(0);
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetItalic(0);
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetItalic(0);
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetShadow(0);
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetShadow(0);
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetShadow(0);
  m_axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetColor(1.0, 0.0, 0.0);
  m_axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetColor(0.0, 1.0, 0.0);
  m_axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetColor(0.0, 0.0, 1.0);

  m_renderer->AddActor(m_axes_actor);

  //Update status bar
  this->statusBar()->showMessage("Rendering... ");
  QApplication::processEvents();

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();

  //Update status bar
  this->statusBar()->showMessage("Rendering... done!");
  QApplication::processEvents();
}




void Form::load2DStereoImage(const char* file_path)
{
  //Create a vector to hold the data
  std::vector<uint32_t> data;

  //Read the entire file
  m_file_manager->readVector(file_path, data);

  //Parse the first 8 bytes to determine dimensions
  uint32_t rows, cols;
  memcpy(&rows,  &data[0],  4*sizeof(uint8_t));
  memcpy(&cols,  &data[1],  4*sizeof(uint8_t));

  vtkImageData* work_pointer;

  //Create and setup a vtkImageData
  if(file_path == STEREO_LEFT_CACHE_PATH)
  {
    work_pointer = m_stereo_left_image;
  }
  else if(file_path == STEREO_RIGHT_CACHE_PATH)
  {
    work_pointer = m_stereo_right_image;
  }
  else if(file_path == STEREO_DISP_CACHE_PATH)
  {
    work_pointer = m_stereo_disp_image;
  }
  else
  {
    qDebug() << "Invalid 2D stereo image path";
    return;
  }

  work_pointer->SetDimensions(rows, cols, 1);
  work_pointer->SetNumberOfScalarComponents(3);
  work_pointer->SetScalarTypeToUnsignedChar();
  work_pointer->AllocateScalars();

  uint32_t val;
  for(int x = 0; x < rows; x++)
  {
    for(int y = 0; y < cols; y++)
    {
      unsigned char* pixel = static_cast<unsigned char*>
                             (work_pointer->GetScalarPointer(x, y, 0));

      //The two first uint32_t are the header
      val = data[y + x*cols + 2];
      memcpy(&pixel[0],  &val,  3*sizeof(uint8_t));
    }
  }

  work_pointer->Modified();
}






void Form::render2DStereoImage(vtkSmartPointer<vtkImageData> image_data)
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

  double scaling = 1;
  if(window_aspect_ratio >= image_aspect_ratio)
  {
    scaling = image_height/window_height;
    m_image_actor->SetPosition((window_width - image_width/scaling)/2, 0);
  }
  else
  {
    scaling = image_width/window_width;
    m_image_actor->SetPosition(0, (window_height-image_height/scaling)/2);
  }

  m_image_resize_filter->SetInputConnection(image_data->GetProducerPort());
  m_image_resize_filter->SetOutputSpacing(scaling, scaling, 1.0);

  m_image_mapper->SetInputConnection(m_image_resize_filter->GetOutputPort());
  m_image_mapper->SetColorWindow(255.0);
  m_image_mapper->SetColorLevel(127.5);

  m_image_actor->SetMapper(m_image_mapper);

  m_renderer->RemoveAllViewProps();
  m_renderer->AddActor2D(m_image_actor);
  m_renderer->ResetCamera();

  this->m_ui->qvtkWidget->update();
}





void Form::updateUIStates()
{
  //OCT page
  m_ui->connected_master_checkbox->setChecked(m_connected_to_master);
  m_ui->connected_master_checkbox_2->setChecked(m_connected_to_master);

  m_ui->browse_button->setEnabled(!m_waiting_response & !m_waiting_response);

  m_ui->request_scan_button->setEnabled(!m_waiting_response);
  m_ui->save_button->setEnabled(m_has_ros_raw_oct & !m_waiting_response);

  m_ui->len_steps_spinbox->setEnabled(!m_waiting_response);
  m_ui->len_range_spinbox->setEnabled(!m_waiting_response);
  m_ui->len_off_spinbox->setEnabled(!m_waiting_response);
  m_ui->wid_steps_spinbox->setEnabled(!m_waiting_response);
  m_ui->wid_range_spinbox->setEnabled(!m_waiting_response);
  m_ui->wid_off_spinbox->setEnabled(!m_waiting_response);
  m_ui->dep_steps_spinbox->setEnabled(!m_waiting_response);
  m_ui->dep_range_spinbox->setEnabled(!m_waiting_response);

  m_ui->view_raw_oct_button->setEnabled(m_has_raw_oct & !m_waiting_response);
  m_ui->view_oct_surf_button->setEnabled(m_has_oct_surf & !m_waiting_response);
  m_ui->view_oct_mass_button->setEnabled(m_has_oct_surf & !m_waiting_response);

  m_ui->calc_oct_surf_button->setEnabled(m_has_raw_oct & !m_waiting_response);
  m_ui->calc_oct_mass_button->setEnabled(m_has_raw_oct & !m_waiting_response);

  //Stereocamera page
  m_ui->view_depth_image_button->setEnabled(m_has_stereo_pcl &
      !m_waiting_response);

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
    m_waiting_response = true;
    updateUIStates();

    //Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    //Update status bar
    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    //Read the file into data
    std::vector<uint8_t> data;
    m_file_manager->readVector(file_name.toStdString().c_str(), data);

    //Update status bar
    this->m_ui->status_bar->showMessage("Loading data into point cloud... ");
    QApplication::processEvents();

    //Loads the raw oct data vector into m_raw_oct_poly_data
    loadRawOCTData(data);

    //Renders the m_raw_oct_poly_data vtkPolyData
    renderRawOCTData();

    //Displays the file name in the ui
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

void Form::on_len_steps_spinbox_editingFinished()
{
  m_current_params.length_steps = this->m_ui->len_steps_spinbox->value();
}

void Form::on_wid_steps_spinbox_editingFinished()
{
  m_current_params.width_steps = this->m_ui->wid_steps_spinbox->value();
}

void Form::on_dep_steps_spinbox_editingFinished()
{
  m_current_params.depth_steps = this->m_ui->dep_steps_spinbox->value();

  //The axial sensor has a fixed resolution, so we change m_dep_range to match
  this->m_ui->dep_range_spinbox->setValue(
      (float)m_current_params.depth_steps/1024.0*2.762);
}

void Form::on_len_range_spinbox_editingFinished()
{
  m_current_params.length_range = this->m_ui->len_range_spinbox->value();
}

void Form::on_wid_range_spinbox_editingFinished()
{
  m_current_params.width_range = this->m_ui->wid_range_spinbox->value();
}

void Form::on_dep_range_spinbox_editingFinished()
{
  m_current_params.depth_range = this->m_ui->dep_range_spinbox->value();

  //The axial sensor has a fixed resolution, so we change m_dep_steps to match
  this->m_ui->dep_steps_spinbox->setValue((int) (
      m_current_params.depth_range/2.762*1024 + 0.5));
}

void Form::on_len_off_spinbox_editingFinished()
{
  m_current_params.length_offset = this->m_ui->len_off_spinbox->value();
}

void Form::on_wid_off_spinbox_editingFinished()
{
  m_current_params.width_offset = this->m_ui->wid_off_spinbox->value();
}

void Form::on_request_scan_button_clicked()
{
  //Disables controls until we get a response
  m_waiting_response = true;
  updateUIStates();

  //Pack the desired OCT params into a struct
  Q_EMIT requestScan(m_current_params);

  //Update status bar
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

    //Update status bar
    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    std::vector<uint8_t> header;
    header.resize(512);
    memcpy(&header[16],  &m_current_params.length_steps,  4*sizeof(uint8_t));
    memcpy(&header[20],  &m_current_params.width_steps,   4*sizeof(uint8_t));
    memcpy(&header[24],  &m_current_params.depth_steps,   4*sizeof(uint8_t));
    memcpy(&header[72],  &m_current_params.length_range,  4*sizeof(uint8_t));
    memcpy(&header[76],  &m_current_params.width_range,   4*sizeof(uint8_t));
    memcpy(&header[116], &m_current_params.depth_range,   4*sizeof(uint8_t));
    memcpy(&header[120], &m_current_params.length_offset, 4*sizeof(uint8_t));
    memcpy(&header[124], &m_current_params.width_offset,  4*sizeof(uint8_t));

    m_file_manager->writeVector(header, file_name.toStdString().c_str());

    //Write the actual data
    std::vector<uint8_t> data;
    m_file_manager->readVector(OCT_RAW_CACHE_PATH, data);
    m_file_manager->writeVector(data, file_name.toStdString().c_str(), true);

    m_waiting_response = false;
    updateUIStates();

    //Update status bar
    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_view_raw_oct_button_clicked()
{
    renderRawOCTData();
}

void Form::on_calc_oct_surf_button_clicked()
{

}

void Form::receivedRawOCTData(OCTinfo params)
{
  //Update status bar
  this->m_ui->status_bar->showMessage("Waiting for OCT scanner response... \
      done!");
  QApplication::processEvents();

  //Loads m_qnode's data vector into m_raw_oct_poly_data
  std::vector<uint8_t> raw_data;
  m_file_manager->readVector(OCT_RAW_CACHE_PATH, raw_data);

  loadRawOCTData(raw_data, params, 0, 0);

  m_has_ros_raw_oct = true;
  updateUIStates();

  //Immediately render it
  renderRawOCTData();

  //Re-enables controls for a potential new scan
  m_waiting_response = false;
  updateUIStates();
}

void Form::receivedOCTSurfData(OCTinfo params)
{
  //Fetch m_oct_pcl_surface from qnode

  //Render it somehow
}

void Form::receivedStereoData()
{
  m_has_stereo_pcl = true;
  updateUIStates();
}

void Form::on_view_depth_image_button_clicked()
{
  m_waiting_response = true;
  updateUIStates();

  loadPCLStereoDepthMap();

  renderPCLStereoDepthMap();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_left_image_button_clicked()
{
  load2DStereoImage(STEREO_LEFT_CACHE_PATH);

  render2DStereoImage(m_stereo_left_image);
}
