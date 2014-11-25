#include "form.h"
#include "ui_form.h"

//============================================================================//
//                                                                            //
//  PUBLIC METHODS                                                            //
//                                                                            //
//============================================================================//

Form::Form(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  m_ui(new Ui::Form)
{
  m_ui->setupUi(this);

  //Initialize the oct parameters
  m_vis_threshold = this->m_ui->viewing_threshold_spinbox->value();
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

  //Update status bar
  this->m_ui->status_bar->showMessage("Ready");
  QApplication::processEvents();

  //Instantiate vtk objects
  //Data structures
  m_raw_oct_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_oct_stereo_trans = vtkSmartPointer<vtkTransform>::New();
  //Filters
  m_vert_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  m_image_resize_filter = vtkSmartPointer<vtkImageReslice>::New();
  m_trans_filter = vtkSmartPointer<vtkTransformFilter>::New();
  //Mappers
  m_poly_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  m_second_poly_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  m_image_mapper = vtkSmartPointer<vtkImageMapper>::New();
  //Actors
  m_actor = vtkSmartPointer<vtkActor>::New();
  m_second_actor = vtkSmartPointer<vtkActor>::New();
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

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

void Form::loadRawOCTData(std::vector<uint8_t>& oct_data, OCTinfo params/*=0*/,
      int file_header /*= 512*/, int frame_header /*= 40*/)
{
  //If we get in here it means we're opening a Thorlabs img file
  if(file_header == 512 && params == default_oct_info)
  {
    //Extract the data dimensions from the header to our m_current_params
    std::memcpy(&m_current_params.length_steps,  &(oct_data[16]),
                                                 4*sizeof(uint8_t));
    std::memcpy(&m_current_params.width_steps,   &(oct_data[20]),
                                                 4*sizeof(uint8_t));
    std::memcpy(&m_current_params.depth_steps,   &(oct_data[24]),
                                                 4*sizeof(uint8_t));
    std::memcpy(&m_current_params.length_range,  &(oct_data[72]),
                                                 4*sizeof(uint8_t));
    std::memcpy(&m_current_params.width_range,   &(oct_data[76]),
                                                 4*sizeof(uint8_t));
    std::memcpy(&m_current_params.depth_range,   &(oct_data[116]),
                                                 4*sizeof(uint8_t));
    std::memcpy(&m_current_params.length_offset, &(oct_data[120]),
                                                 4*sizeof(uint8_t));
    std::memcpy(&m_current_params.width_offset,  &(oct_data[124]),
                                                 4*sizeof(uint8_t));
    //Reset our controls to show the actual params of the received data
    on_reset_params_button_clicked();
  }
  //If not, then we take the params out of the OCTinfo struct that was passed
  else if(file_header == 0 && !(params == default_oct_info))
  {
    m_current_params.length_steps = params.length_steps;
    m_current_params.width_steps  = params.width_steps;
    m_current_params.depth_steps  = params.depth_steps;
    m_current_params.length_range = params.length_range;
    m_current_params.width_range  = params.width_range;
    m_current_params.depth_range  = params.depth_range;
    m_current_params.length_offset= params.length_offset;
    m_current_params.width_offset = params.width_offset;
    //Reset our controls to show the actual params of the received data
    on_reset_params_button_clicked();
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

  if(m_current_params.length_range != 0)
    length_incrm = m_current_params.length_range/m_current_params.length_steps;

  if(m_current_params.width_range != 0)
    width_incrm = m_current_params.width_range/m_current_params.width_steps;

  if(m_current_params.depth_range != 0)
    depth_incrm = m_current_params.depth_range/m_current_params.depth_steps;

  //Creates an array for point coordinates, and one for the scalars
  VTK_NEW(vtkPoints, points);
  points->Reset();
  VTK_NEW(vtkTypeUInt8Array, dataArray);
  dataArray->Reset();

  points->SetNumberOfPoints(   m_current_params.length_steps *
                               m_current_params.width_steps *
                               m_current_params.depth_steps);

  dataArray->SetNumberOfValues(m_current_params.length_steps *
                               m_current_params.width_steps *
                               m_current_params.depth_steps);

  //Update status bar
  this->statusBar()->showMessage("Building raw point data... ");
  QApplication::processEvents();

  int id = 0;
  for(int i = 0; i < m_current_params.length_steps; i++)
  {
    for(int j = 0; j < m_current_params.width_steps; j++)
    {
      for(int k = 0; k < m_current_params.depth_steps; k++, id++)
      {
        //Thorlabs img files use 40 bytes of NULL between each B-scan
        int val = oct_data[id + frame_header*i + file_header];

        points->SetPoint(id, i*length_incrm + m_current_params.length_offset,
                             j*width_incrm + m_current_params.width_offset,
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
        QString::number(m_current_params.length_steps));

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

//------------------------------------------------------------------------------

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
  VTK_NEW(vtkPolyData, vis_poly_data);
  uint8_t value;

  //Update status bar
  this->statusBar()->showMessage("Preparing PolyData for display... ");
  QApplication::processEvents();

  for(uint32_t i = 0; i < num_pts; i++)
  {
    value = old_data_array->GetValue(i);
    if(value > m_vis_threshold)
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
  vis_poly_data->Reset();

  vis_poly_data->SetPoints(new_points);
  vis_poly_data->GetPointData()->SetScalars(new_data_array);

  m_vert_filter->SetInput(vis_poly_data);

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

//------------------------------------------------------------------------------

void Form::loadPCLtoPolyData(const char* file_path,
    vtkSmartPointer<vtkPolyData> cloud_poly_data)
{
  //Points have no color component
  if(file_path == OCT_SURF_CACHE_PATH)
  {
    //Update status bar
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

    //Update status bar
    this->statusBar()->showMessage("Building OCT surface vtkPolyData... ");
    QApplication::processEvents();

    for(int i = 0; i < num_pts; i++)
    {
      cloud_point = &(pts->at(i));
      points->SetPoint(i, cloud_point->x, cloud_point->y, cloud_point->z);
    }

    cloud_poly_data->Reset();
    cloud_poly_data->SetPoints(points);
  }

  //Points have color component
  else if (file_path == STEREO_DEPTH_CACHE_PATH)
  {
    //Update status bar
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

    //Update status bar
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
  }
  else
  {
    qDebug() << "Invalid file path for loadPCLtoPolyData";
    return;
  }
}

//------------------------------------------------------------------------------

void Form::renderPolyDataSurface(vtkSmartPointer<vtkPolyData> cloud_poly_data)
{
  int num_pts = cloud_poly_data->GetPointData()->GetNumberOfTuples();

  if(num_pts == 0)
  {
    qDebug() << "cloud_poly_data is empty!";
    return;
  }

  //Update status bar
  this->statusBar()->showMessage(
      "Preparing depth map vtkPolyData for display... ");
  QApplication::processEvents();

  m_vert_filter->SetInput(cloud_poly_data);

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

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();
}

//------------------------------------------------------------------------------

void Form::load2DStereoImage(const char* file_path,
      vtkSmartPointer<vtkImageData> image_data)
{
  //Create a vector to hold the data
  std::vector<uint32_t> data;

  //Read the entire file
  m_file_manager->readVector(file_path, data);

  //Parse the first 8 bytes to determine dimensions
  uint32_t rows, cols;
  memcpy(&rows,  &data[0],  4*sizeof(uint8_t));
  memcpy(&cols,  &data[1],  4*sizeof(uint8_t));

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

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

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

  //Update status bar
  this->statusBar()->showMessage(
        "Preparing vtkPolyData objects for overlay display... ");
  QApplication::processEvents();

  //OCT
  {
    m_trans_filter->SetInput(oct_surface);
    m_trans_filter->SetTransform(m_oct_stereo_trans);

    m_vert_filter->RemoveAllInputs();
    m_vert_filter->SetInputConnection(m_trans_filter->GetOutputPort());

    m_poly_mapper->SetInputConnection(m_vert_filter->GetOutputPort());

    m_actor->SetMapper(m_poly_mapper);
    m_actor->GetProperty()->SetColor(0, 0, 1); //blue

    //Produce the oct output
    m_poly_mapper->Update();
  }

  //Stereocamera depthmap
  {
    m_vert_filter->RemoveAllInputs();
    m_vert_filter->SetInput(depth_map);

    m_second_poly_mapper->SetInputConnection(m_vert_filter->GetOutputPort());

    m_second_actor->SetMapper(m_second_poly_mapper);
    m_second_actor->GetProperty()->SetColor(1, 0, 0); //red

    m_second_poly_mapper->Update();
  }

  m_renderer->RemoveAllViewProps();
  m_renderer->AddActor(m_actor);
  m_renderer->AddActor(m_second_actor);
  m_renderer->ResetCamera();

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();
}

//------------------------------------------------------------------------------

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

  m_ui->viewing_threshold_spinbox->setEnabled(!m_waiting_response);

  m_ui->view_raw_oct_button->setEnabled(m_has_raw_oct && !m_waiting_response);
  m_ui->view_oct_surf_button->setEnabled(m_has_oct_surf && !m_waiting_response);
  m_ui->view_oct_mass_button->setEnabled(m_has_oct_surf && !m_waiting_response);

  m_ui->calc_oct_surf_button->setEnabled(m_has_raw_oct && !m_waiting_response);
  m_ui->calc_oct_mass_button->setEnabled(m_has_raw_oct && !m_waiting_response);

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

//============================================================================//
//                                                                            //
//  PRIVATE Q_SLOTS                                                           //
//                                                                            //
//============================================================================//

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

//------------------------------------------------------------------------------

void Form::on_connected_master_checkbox_clicked(bool checked)
{
  m_connected_to_master = checked;
  updateUIStates();
}

//------------------------------------------------------------------------------

void Form::on_dep_steps_spinbox_editingFinished()
{
  //The axial sensor has a fixed resolution, so we change m_dep_range to match
  this->m_ui->dep_range_spinbox->setValue(
      (float)this->m_ui->dep_steps_spinbox->value()/1024.0*2.762);
}

//------------------------------------------------------------------------------

void Form::on_dep_range_spinbox_editingFinished()
{
  //The axial sensor has a fixed resolution, so we change m_dep_steps to match
  this->m_ui->dep_steps_spinbox->setValue((int) (
      this->m_ui->dep_range_spinbox->value()/2.762*1024 + 0.5));
}

//------------------------------------------------------------------------------

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

  //Pack the desired OCT params into a struct
  Q_EMIT requestScan(m_current_params);

  //Update status bar
  this->m_ui->status_bar->showMessage("Waiting for OCT scanner response... ");
  QApplication::processEvents();
}

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

void Form::on_viewing_threshold_spinbox_editingFinished()
{
  m_vis_threshold = m_ui->viewing_threshold_spinbox->value();
}

//------------------------------------------------------------------------------

void Form::on_view_raw_oct_button_clicked()
{
    renderRawOCTData();
}

//------------------------------------------------------------------------------

void Form::on_calc_oct_surf_button_clicked()
{
  //Disables controls until we get a response
  m_waiting_response = true;
  updateUIStates();

  Q_EMIT requestSegmentation(m_current_params);

  //Update status bar
  this->m_ui->status_bar->showMessage("Waiting for OCT surface response... ");
  QApplication::processEvents();
}

//------------------------------------------------------------------------------

void Form::on_view_left_image_button_clicked()
{  
  this->m_ui->status_bar->showMessage("Rendering left image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkImageData, left_image);

  load2DStereoImage(STEREO_LEFT_CACHE_PATH, left_image);

  render2DStereoImage(left_image);

  this->m_ui->status_bar->showMessage("Rendering left image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

//------------------------------------------------------------------------------

void Form::on_view_right_image_button_clicked()
{
  this->m_ui->status_bar->showMessage("Rendering right image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkImageData, right_image);

  load2DStereoImage(STEREO_RIGHT_CACHE_PATH, right_image);

  render2DStereoImage(right_image);

  this->m_ui->status_bar->showMessage("Rendering right image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

//------------------------------------------------------------------------------

void Form::on_view_disp_image_button_clicked()
{
  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkImageData, disp_image);

  load2DStereoImage(STEREO_DISP_CACHE_PATH, disp_image);

  render2DStereoImage(disp_image);

  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

//------------------------------------------------------------------------------

void Form::on_view_depth_image_button_clicked()
{
  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkPolyData, depth_image);

  loadPCLtoPolyData(STEREO_DEPTH_CACHE_PATH, depth_image);

  renderPolyDataSurface(depth_image);

  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

void Form::on_calc_transform_button_clicked()
{
  //Disables controls until we get a response
  m_waiting_response = true;
  updateUIStates();

  Q_EMIT requestRegistration();

  //Update status bar
  this->m_ui->status_bar->showMessage("Waiting for OCT registration "
      "transform... ");
  QApplication::processEvents();
}

//------------------------------------------------------------------------------

void Form::on_print_transform_button_clicked()
{
  //Update status bar
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

//------------------------------------------------------------------------------

void Form::on_view_oct_surf_button_clicked()
{
  //Update status bar
  this->m_ui->status_bar->showMessage("Rendering OCT surface... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkPolyData, surf_oct_poly_data);

  loadPCLtoPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);
  renderPolyDataSurface(surf_oct_poly_data);

  //Update status bar
  this->m_ui->status_bar->showMessage("Rendering OCT surface... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

//------------------------------------------------------------------------------

void Form::on_view_simple_overlay_button_clicked()
{
  //Update status bar
  this->m_ui->status_bar->showMessage("Rendering simple surface overlay... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  //Read the oct surface and depth maps
  VTK_NEW(vtkPolyData, oct_surface);
  VTK_NEW(vtkPolyData, depth_map);
  loadPCLtoPolyData(OCT_SURF_CACHE_PATH, oct_surface);
  loadPCLtoPolyData(STEREO_DEPTH_CACHE_PATH, depth_map);

  //Render both simultaneously
  renderOverlay(oct_surface, depth_map);

  //Update status bar
  this->m_ui->status_bar->showMessage("Rendering simple surface overlay... "
      "done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

//------------------------------------------------------------------------------

void Form::receivedRawOCTData(OCTinfo params)
{
  //Update status bar
  this->m_ui->status_bar->showMessage("Waiting for OCT scanner response... "
      "done!");
  QApplication::processEvents();

  //Loads m_qnode's data vector into m_raw_oct_poly_data
  std::vector<uint8_t> raw_data;
  m_file_manager->readVector(OCT_RAW_CACHE_PATH, raw_data);

  loadRawOCTData(raw_data, params, 0, 0);

  //Immediately render it
  renderRawOCTData();

  //Re-enables controls for a potential new scan
  m_has_ros_raw_oct = true;
  m_waiting_response = false;
  updateUIStates();
}

//------------------------------------------------------------------------------

void Form::receivedOCTSurfData(OCTinfo params)
{
  //Update status bar
  this->m_ui->status_bar->showMessage("Waiting for OCT surface response... "
      "done!");
  QApplication::processEvents();

  VTK_NEW(vtkPolyData, surf_oct_poly_data);

  loadPCLtoPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);
  renderPolyDataSurface(surf_oct_poly_data);

  //Re-enables controls for a potential new scan
  m_has_oct_surf = true;
  m_waiting_response = false;
  updateUIStates();
}

//------------------------------------------------------------------------------

void Form::receivedStereoData()
{
  m_has_stereo_data = true;
  updateUIStates();
}

//------------------------------------------------------------------------------

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

