#include "form.h"
#include "ui_form.h"

Form::Form(int argc, char** argv, QWidget* parent)
    : QMainWindow(parent), m_ui(new Ui::Form) {
  m_ui->setupUi(this);

  // Initialize the oct parameters with the UI values (syncing them)
  m_min_vis_thresh = this->m_ui->raw_min_vis_spinbox->value();
  m_max_vis_thresh = this->m_ui->raw_max_vis_spinbox->value();
  m_current_params.length_steps = this->m_ui->len_steps_spinbox->value();
  m_current_params.width_steps = this->m_ui->wid_steps_spinbox->value();
  m_current_params.depth_steps = this->m_ui->dep_steps_spinbox->value();
  m_current_params.length_range = this->m_ui->len_range_spinbox->value();
  m_current_params.width_range = this->m_ui->wid_range_spinbox->value();
  m_current_params.depth_range = this->m_ui->dep_range_spinbox->value();
  m_current_params.length_offset = this->m_ui->len_off_spinbox->value();
  m_current_params.width_offset = this->m_ui->wid_off_spinbox->value();

  // Disable some buttons until they can be pressed
  m_connected_to_master = false;
  m_has_ros_raw_oct = false;
  m_has_raw_oct = false;
  m_waiting_response = false;
  m_has_oct_surf = false;
  m_has_oct_mass = false;
  m_has_left_img = false;
  m_has_right_img = false;
  m_has_disp_img = false;
  m_has_depth_img = false;
  m_has_transform = false;
  m_viewing_overlay = false;
  updateUIStates();

  // Creates qnode and it's thread, connecting signals and slots
  m_qthread = new QThread;
  m_qnode = new QNode(argc, argv);
  m_qnode->moveToThread(m_qthread);

  // Allows us to use OCTinfo structs in signals/slots
  qRegisterMetaType<OCTinfo>();
  qRegisterMetaType<std::vector<uint8_t> >();

  // Connect signals and slots
  connect(m_qnode, SIGNAL(rosMasterChanged(bool)), this,
          SLOT(on_connected_master_checkbox_clicked(bool)));

  connect(this, SIGNAL(requestScan(OCTinfo)), m_qnode,
          SLOT(requestScan(OCTinfo)),
          Qt::QueuedConnection);  // Runs slot on receiving thread

  connect(this, SIGNAL(requestSegmentation(OCTinfo)), m_qnode,
          SLOT(requestSegmentation(OCTinfo)), Qt::QueuedConnection);

  connect(this, SIGNAL(requestRegistration()), m_qnode,
          SLOT(requestRegistration()), Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedOCTRawData(OCTinfo)), this,
          SLOT(receivedRawOCTData(OCTinfo)), Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedOCTSurfData(OCTinfo)), this,
          SLOT(receivedOCTSurfData(OCTinfo)), Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedLeftImage()), this, SLOT(receivedLeftImage()),
          Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedRightImage()), this,
          SLOT(receivedRightImage()), Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedDispImage()), this, SLOT(receivedDispImage()),
          Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedDepthImage()), this,
          SLOT(receivedDepthImage()), Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedRegistration()), this,
          SLOT(receivedRegistration()), Qt::QueuedConnection);

  connect(this, SIGNAL(setLeftAccumulatorSize(uint)), m_qnode,
          SLOT(setLeftAccumulatorSize(uint)), Qt::QueuedConnection);

  connect(this, SIGNAL(setDepthAccumulatorSize(uint)), m_qnode,
          SLOT(setDepthAccumulatorSize(uint)), Qt::QueuedConnection);

  // Wire up qnode and it's thread. Don't touch this unless absolutely necessary
  connect(m_qthread, SIGNAL(started()), m_qnode, SLOT(process()));
  connect(m_qnode, SIGNAL(finished()), m_qthread, SLOT(quit()));
  connect(m_qnode, SIGNAL(finished()), m_qthread, SLOT(deleteLater()));
  connect(m_qthread, SIGNAL(finished()), m_qthread, SLOT(deleteLater()));
  m_qthread->start();

  this->m_ui->status_bar->showMessage("Ready");
  QApplication::processEvents();

  // Instantiate vtk objects
  // Data structures
  m_oct_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_oct_stereo_trans = vtkSmartPointer<vtkTransform>::New();
  m_oct_stereo_trans->Identity();
  // Actors
  m_oct_vol_actor = vtkSmartPointer<vtkActor>::New();
  m_oct_surf_actor = vtkSmartPointer<vtkActor>::New();
  m_oct_mass_actor = vtkSmartPointer<vtkActor>::New();
  m_stereo_2d_actor = vtkSmartPointer<vtkActor2D>::New();
  m_stereo_depth_actor = vtkSmartPointer<vtkActor>::New();
  m_oct_axes_actor = vtkSmartPointer<vtkAxesActor>::New();
  m_trans_axes_actor = vtkSmartPointer<vtkAxesActor>::New();
  // Others
  m_renderer = vtkSmartPointer<vtkRenderer>::New();

  // A non-black background allows us to see datapoints with scalar value 0
  m_renderer->SetBackground(0, 0, 0.1);
  m_renderer->SetBackground2(0, 0, 0.05);
  m_renderer->SetGradientBackground(1);

  // Adds our renderer to the QVTK widget
  this->m_ui->qvtkWidget->GetRenderWindow()->AddRenderer(m_renderer);
}

Form::~Form() {
  delete m_ui;
  delete m_qnode;

  // Waits until the m_qnode's destructor has finished before killing the thread
  m_qthread->wait();
  delete m_qthread;

  // Delete all of our .cache files
  m_file_manager->clearAllFiles();
}

void Form::updateUIStates() {
  // OCT page
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
  // m_ui->calc_oct_mass_button->setEnabled(m_has_oct_surf &&
  // !m_waiting_response);
  m_ui->calc_oct_mass_button->setEnabled(!m_waiting_response);

  // Stereocamera page
  m_ui->view_left_image_button->setEnabled(m_has_left_img &&
                                           !m_waiting_response);
  m_ui->view_right_image_button->setEnabled(m_has_right_img &&
                                            !m_waiting_response);
  m_ui->view_disp_image_button->setEnabled(m_has_disp_img &&
                                           !m_waiting_response);
  m_ui->view_depth_image_button->setEnabled(m_has_depth_img &&
                                            !m_waiting_response);
  m_ui->left_accu_spinbox->setEnabled(!m_waiting_response);
  m_ui->depth_accu_spinbox->setEnabled(!m_waiting_response);
  m_ui->left_accu_reset_button->setEnabled(!m_waiting_response);
  m_ui->depth_accu_reset_button->setEnabled(!m_waiting_response);

  // Visualization page
  m_ui->over_min_vis_spinbox->setEnabled(!m_waiting_response);
  m_ui->over_max_vis_spinbox->setEnabled(!m_waiting_response);
  m_ui->over_min_vis_slider->setEnabled(!m_waiting_response);
  m_ui->over_max_vis_slider->setEnabled(!m_waiting_response);

  m_ui->over_raw_checkbox->setEnabled(m_has_raw_oct && !m_waiting_response);
  m_ui->over_oct_surf_checkbox->setEnabled(m_has_oct_surf &&
                                           !m_waiting_response);
  m_ui->over_oct_mass_checkbox->setEnabled(m_has_oct_mass &&
                                           !m_waiting_response);
  m_ui->over_oct_axes_checkbox->setEnabled(m_has_raw_oct &&
                                           !m_waiting_response);
  m_ui->over_depth_checkbox->setEnabled(m_has_depth_img && !m_waiting_response);
  m_ui->over_trans_axes_checkbox->setEnabled(m_has_transform &&
                                             !m_waiting_response);
  m_ui->calc_transform_button->setEnabled(m_has_oct_surf && m_has_depth_img &&
                                          !m_waiting_response);
  m_ui->print_transform_button->setEnabled(m_has_transform &&
                                           !m_waiting_response);

  // Clear the overlay selections if we go back to viewing something else
  if (!m_viewing_overlay) {
    m_ui->over_raw_checkbox->setChecked(false);
    m_ui->over_oct_surf_checkbox->setChecked(false);
    m_ui->over_oct_mass_checkbox->setChecked(false);
    m_ui->over_depth_checkbox->setChecked(false);
    m_ui->over_trans_axes_checkbox->setChecked(false);
    m_ui->over_oct_axes_checkbox->setChecked(false);
  }
}

//---------------INPUT----------------------------------------------------------

void Form::processOCTHeader(std::vector<uint8_t>& full_array) {
  // Copies the OCT params from the header locations into our struct
  std::memcpy(&m_current_params.length_steps, &(full_array[16]), 4);
  std::memcpy(&m_current_params.width_steps, &(full_array[20]), 4);
  std::memcpy(&m_current_params.depth_steps, &(full_array[24]), 4);

  std::memcpy(&m_current_params.length_range, &(full_array[72]), 4);
  std::memcpy(&m_current_params.width_range, &(full_array[76]), 4);
  std::memcpy(&m_current_params.depth_range, &(full_array[116]), 4);

  std::memcpy(&m_current_params.length_offset, &(full_array[120]), 4);
  std::memcpy(&m_current_params.width_offset, &(full_array[124]), 4);

  // Checks the datetime string. The Thorlabs software always fills this, but
  // we don't, so we use this to check whether we have one of their img files
  uint32_t checker, frame_header = 0;
  std::memcpy(&checker, &(full_array[44]), 4);
  if (checker == 0)  // Not a thorlabs .img file, so get frame_header from array
  {
    std::memcpy(&frame_header, &(full_array[132]), 4);
  } else  // Thorlabs image, so we use their default 40 bytes frame_header
  {
    frame_header = 40;
  }

  uint32_t b_scan_size =
      m_current_params.width_steps * m_current_params.depth_steps;
  uint32_t b_scan_count = 0;

  std::vector<uint8_t> result;
  result.resize(m_current_params.length_steps * b_scan_size);

  // Goes over the full_array, skipping the file and frame headers
  for (std::vector<uint8_t>::iterator iter =
           full_array.begin() + OCT_HEADER_BYTES;
       iter != full_array.end(); ++b_scan_count) {
    std::copy(iter, iter + b_scan_size, &(result[b_scan_count * b_scan_size]));
    iter += b_scan_size + frame_header;
  }

  result.swap(full_array);
}

void Form::loadVectorToPolyData(std::vector<uint8_t>& oct_data) {
  // Determines the distance between consecutive points in each direction
  float length_incrm = 1;
  float width_incrm = 1;
  float depth_incrm = 2.762 / 1024.0;  // Fixed axial resolution. If we have no
  // depth range, this is our best bet

  // In case we have some sample data or weird input, prevents null increments
  if (m_current_params.length_range != 0)
    length_incrm =
        m_current_params.length_range / m_current_params.length_steps;

  if (m_current_params.width_range != 0)
    width_incrm = m_current_params.width_range / m_current_params.width_steps;

  if (m_current_params.depth_range != 0)
    depth_incrm = m_current_params.depth_range / m_current_params.depth_steps;

  // Clear the arays holding the current point data and scalars, to prevent
  // memory spikes

  // Setup arrays to hold point coordinates and the scalars
  VTK_NEW(vtkPoints, points);
  VTK_NEW(vtkTypeUInt8Array, dataArray);
  points->SetNumberOfPoints(m_current_params.length_steps *
                            m_current_params.width_steps *
                            m_current_params.depth_steps);
  dataArray->SetNumberOfValues(m_current_params.length_steps *
                               m_current_params.width_steps *
                               m_current_params.depth_steps);

  // Set them into our polydata early, to potentially drop (and gracefully
  // delete) the arrays that previously were in those positions
  m_oct_poly_data->Reset();
  m_oct_poly_data->SetPoints(points);
  m_oct_poly_data->GetPointData()->SetScalars(dataArray);

  this->statusBar()->showMessage("Building raw point data... ");
  QApplication::processEvents();

  int id = 0;
  for (int i = 0; i < m_current_params.length_steps; i++) {
    for (int j = 0; j < m_current_params.width_steps; j++) {
      for (int k = 0; k < m_current_params.depth_steps; k++, id++) {
        points->SetPoint(id, i * length_incrm + m_current_params.length_offset,
                         j * width_incrm + m_current_params.width_offset,
                         k * depth_incrm);

        dataArray->SetValue(id, oct_data[id]);
        //        std::cout << "id: " << id << "\t\tx: " << i*length_incrm <<
        // "\t\ty: "
        //        << j*width_incrm << "\t\tz: " << k*depth_incrm << "\t\tval: "
        // <<
        //        oct_data[id] << std::endl;
      }
    }

    this->statusBar()->showMessage(
        "Building raw point data... " + QString::number(i) + " of " +
        QString::number(m_current_params.length_steps));

    QApplication::processEvents();
  }

  m_has_raw_oct = true;
  updateUIStates();

  this->statusBar()->showMessage("Building raw point data... done!");
  QApplication::processEvents();
}

void Form::loadPCLCacheToPolyData(
    const char* file_path, vtkSmartPointer<vtkPolyData> cloud_poly_data) {
  // Points have no color components
  if (std::strcmp(file_path, OCT_SURF_CACHE_PATH) == 0) {
    this->statusBar()->showMessage("Reading OCT surface PCL cache... ");
    QApplication::processEvents();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);

    // Reads the file into our pts point cloud
    m_file_manager->readPCL(file_path, pts);

    int num_pts = pts->size();

    VTK_NEW(vtkPoints, points);
    points->SetNumberOfPoints(num_pts);

    pcl::PointXYZ* cloud_point;

    this->statusBar()->showMessage("Building OCT surface vtkPolyData... ");
    QApplication::processEvents();

    for (int i = 0; i < num_pts; i++) {
      cloud_point = &(pts->at(i));

      // Intentionally inverts X and Y at this point. This is to compensate for
      // OCT_segmentation, at some point of the algorithm, inverting what it
      // considers "x" and "y". This is combined with another type of inversion
      // at QNode::requestSegmentation
      points->SetPoint(i, cloud_point->y, cloud_point->x, cloud_point->z);
    }

    cloud_poly_data->SetPoints(points);

    this->statusBar()->showMessage("Building OCT surface vtkPolyData... done!");
    QApplication::processEvents();
  }

  // Points have color component
  else if (std::strcmp(file_path, STEREO_DEPTH_CACHE_PATH) == 0) {
    this->statusBar()->showMessage("Reading depth map PCL cache... ");
    QApplication::processEvents();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    // Reads the file into our pts point cloud
    m_file_manager->readPCL(file_path, pts);
    int num_pts = pts->size();

    VTK_NEW(vtkPoints, points);
    points->SetNumberOfPoints(num_pts);

    VTK_NEW(vtkUnsignedCharArray, data_array);
    data_array->SetNumberOfComponents(3);
    data_array->SetNumberOfTuples(num_pts);
    data_array->SetName("Colors");

    pcl::PointXYZRGB* cloud_point;
    unsigned char color[3] = {0, 0, 0};

    this->statusBar()->showMessage("Building depth map vtkPolyData... ");
    QApplication::processEvents();

    for (int i = 0; i < num_pts; i++) {
      cloud_point = &(pts->at(i));

      points->SetPoint(i, cloud_point->x, cloud_point->y, cloud_point->z);

      color[0] = cloud_point->r;
      color[1] = cloud_point->g;
      color[2] = cloud_point->b;
      data_array->SetTupleValue(i, color);
    }

    cloud_poly_data->SetPoints(points);
    cloud_poly_data->GetPointData()->SetScalars(data_array);

    std::cout
        << "\n\n=========================================\ncloud_poly_data:\n";
    cloud_poly_data->Print(std::cout);

    this->statusBar()->showMessage("Building depth map vtkPolyData... done!");
    QApplication::processEvents();
  } else {
    qDebug() << "Invalid file path for loadPCLCacheToPolyData";
    return;
  }
}

void Form::load2DVectorCacheToImageData(
    const char* file_path, vtkSmartPointer<vtkImageData> image_data) {
  if (std::strcmp(file_path, STEREO_DISP_CACHE_PATH) == 0) {
    std::vector<uint8_t> data;

    // Read the entire file
    m_file_manager->readVector(file_path, data);

    // Parse the first 8 bytes to determine dimensions
    uint32_t rows, cols;
    memcpy(&rows, &data[0], 4);
    memcpy(&cols, &data[1], 4);

    image_data->SetDimensions(cols, rows, 1);
    image_data->SetNumberOfScalarComponents(1);
    image_data->SetScalarTypeToUnsignedChar();
    image_data->AllocateScalars();

    uint8_t val;
    for (uint32_t y = 0; y < rows; y++) {
      for (uint32_t x = 0; x < cols; x++) {
        // We need to invert the vertical coordinate since we use different
        // origins
        unsigned char* pixel = static_cast<unsigned char*>(
            image_data->GetScalarPointer(x, (rows - 1) - y, 0));

        // The two first uint32_t are the header
        val = data[x + y * cols + 8];  // Skip the two 32-bit values (header)
        memcpy(&pixel[0], &val, 1);
      }
    }

    image_data->Modified();
  } else {
    std::vector<uint32_t> data;

    // Read the entire file
    m_file_manager->readVector(file_path, data);

    // Parse the first 8 bytes to determine dimensions
    uint32_t rows, cols;
    memcpy(&rows, &data[0], 4);
    memcpy(&cols, &data[1], 4);

    image_data->SetDimensions(cols, rows, 1);
    image_data->SetNumberOfScalarComponents(3);
    image_data->SetScalarTypeToUnsignedChar();
    image_data->AllocateScalars();

    uint32_t val;
    for (uint32_t y = 0; y < rows; y++) {
      for (uint32_t x = 0; x < cols; x++) {
        // We need to invert the vertical coordinate since we use different
        // origins
        unsigned char* pixel = static_cast<unsigned char*>(
            image_data->GetScalarPointer(x, (rows - 1) - y, 0));

        // The two first uint32_t are the header
        val = data[x + y * cols + 2];  // Skip the two 32-bit values (header)
        memcpy(&pixel[0], &val, 3);
      }
    }

    image_data->Modified();
  }
}

//------------PROCESSING--------------------------------------------------------

void Form::medianFilter2D(std::vector<uint8_t>& input) {
  std::vector<double> rol;
  rol.resize(9);
  std::vector<uint8_t> filtered_data;
  filtered_data.resize(input.size());

  int depth = m_current_params.depth_steps;
  int width = m_current_params.width_steps;
  int length = m_current_params.length_steps;
  int index = 0;

  QString indicator;

  for (int i = 1; i < length - 1; i++) {
    for (int j = 1; j < width - 1; j++) {
      for (int k = 1; k < depth - 1; k++) {
        index = i * (width * depth) + j * depth + k;

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
        QString("Applying 2D median filter... B-scan ") + indicator.number(i) +
        QString(" of ") + indicator.number(length - 1));
    QApplication::processEvents();
  }

  input.swap(filtered_data);
}

void Form::medianFilter3D(std::vector<uint8_t>& input) {
  std::vector<double> rol;
  rol.resize(27);
  std::vector<uint8_t> filtered_data;
  filtered_data.resize(input.size());

  int depth = m_current_params.depth_steps;
  int width = m_current_params.width_steps;
  int length = m_current_params.length_steps;
  uint32_t index = 0;

  QString indicator;

  for (int i = 1; i < length - 1; i++) {
    for (int j = 1; j < width - 1; j++) {
      for (int k = 1; k < depth - 1; k++) {
        index = i * (width * depth) + j * depth + k;

        // Center point
        rol[0] = input[index];

        // Points in the same B-scan
        rol[1] = input[index + 1];      // down
        rol[2] = input[index - 1];      // up
        rol[3] = input[index + depth];  // forwards
        rol[4] = input[index + depth + 1];
        rol[5] = input[index + depth - 1];
        rol[6] = input[index - depth];  // back
        rol[7] = input[index - depth + 1];
        rol[8] = input[index - depth - 1];

        // Points in the next B-scan
        rol[9] = input[index + depth * width];  // left
        rol[10] = input[index + depth * width + 1];
        rol[11] = input[index + depth * width - 1];
        rol[12] = input[index + depth * width + depth];
        rol[13] = input[index + depth * width + depth + 1];
        rol[14] = input[index + depth * width + depth - 1];
        rol[15] = input[index + depth * width - depth];
        rol[16] = input[index + depth * width - depth + 1];
        rol[17] = input[index + depth * width - depth - 1];

        // Points in the previous B-scan
        rol[18] = input[index - depth * width];  // right
        rol[19] = input[index - depth * width + 1];
        rol[20] = input[index - depth * width - 1];
        rol[21] = input[index - depth * width + depth];
        rol[22] = input[index - depth * width + depth + 1];
        rol[23] = input[index - depth * width + depth - 1];
        rol[24] = input[index - depth * width - depth];
        rol[25] = input[index - depth * width - depth + 1];
        rol[26] = input[index - depth * width - depth - 1];

        std::sort(rol.begin(), rol.end());

        filtered_data[index] = rol[13];
      }
    }

    this->statusBar()->showMessage(
        QString("Applying 3D median filter... B-scan ") + indicator.number(i) +
        QString(" of ") + indicator.number(length - 1));

    QApplication::processEvents();
  }
  input.swap(filtered_data);
}

void Form::discardTop(std::vector<uint8_t>& input, float fraction_to_discard) {
  int depth = m_current_params.depth_steps;
  int width = m_current_params.width_steps;
  int length = m_current_params.length_steps;

  // The OCT scanner produces an artifact where the very top of every A scan
  // consists of false, intensity = 255 samples. Here we discard those (set them
  // to 0) so it doesn't trouble the other algorithms
  int discard_max = depth * fraction_to_discard;

  for (int i = 0; i < length; i++) {
    for (int j = 0; j < width; j++) {
      for (int k = 0; k < discard_max; k++) {
        input[k + j * depth + i * depth * width] = 0;
      }
    }
  }
}

void Form::normalize(std::vector<uint8_t>& input) {
  int depth = m_current_params.depth_steps;
  int width = m_current_params.width_steps;
  int length = m_current_params.length_steps;
  int index;
  int max_value = 0;

  std::vector<uint32_t> values;
  values.resize(256);

  // Build a histogram of the greyscale values of the whole image
  for (int i = 0; i < length; i++) {
    for (int j = 0; j < width; j++) {
      for (int k = 0; k < depth; k++) {
        index = input[k + j * depth + i * depth * width];
        values[index] += 1;

        if (index > max_value) max_value = index;
      }
    }
  }

  // Print said histogram to console
  for (std::vector<uint32_t>::iterator iter = values.begin();
       iter != values.end(); ++iter) {
    std::cout << "For value \t" << (unsigned int)(iter - values.begin())
              << ":\t" << (unsigned int)*iter << std::endl;
  }

  std::cout << "Rescaling by " << (int)max_value << std::endl;

  // Rescales every value to the range [0,255]
  for (int i = 0; i < length; i++) {
    for (int j = 0; j < width; j++) {
      for (int k = 0; k < depth; k++) {
        input[k + j * depth + i * depth * width] *= 255.0 / max_value;
      }
    }
  }
}

void Form::discardImageSides(vtkSmartPointer<vtkImageData> input, float frac_x,
                             float frac_y) {

  // Makes sure the type is 'unsigned char'. This is lazy, but making this
  // function generic would involve templating it and passing a type, which is
  // not a necessary function at this time
  int type = input->GetScalarType();
  assert(type == 3);

  int extents[6];
  input->GetExtent(extents);

  // For a 128x128x128 points image: Sizes = 128
  int size_x = extents[1] - extents[0] + 1;
  int size_y = extents[3] - extents[2] + 1;
  int size_z = extents[5] - extents[4] + 1;

  // Find the minimum and maximum indices that are valid
  // For a 128x128x128 points image with fracs = 0: min_x = 0; max_x = 127
  int min_x = extents[0] + frac_x * size_x;
  int max_x = extents[1] - frac_x * size_x;
  int min_y = extents[2] + frac_y * size_y;
  int max_y = extents[3] - frac_y * size_y;

  // Go over input, sets any sample that has an 'i' value lower than min_i or
  // higher than max_i to zero
  for (int i = 0; i < size_x; i++) {
    for (int j = 0; j < size_y; j++) {
      for (int k = 0; k < size_z; k++) {
        if (i < min_x || i > max_x || j < min_y || j > max_y) {

          unsigned char* pixel =
              static_cast<unsigned char*>(input->GetScalarPointer(i, j, k));

          pixel[0] = 0;
        }
      }
    }
  }
}

//------------RENDERING---------------------------------------------------------

void Form::renderAxes(vtkSmartPointer<vtkAxesActor> actor,
                      vtkSmartPointer<vtkTransform> trans) {
  actor->SetNormalizedShaftLength(1.0, 1.0, 1.0);
  actor->SetShaftTypeToCylinder();
  actor->SetCylinderRadius(0.02);
  actor->SetXAxisLabelText("length        ");
  actor->SetYAxisLabelText("width (B-Scan)");
  actor->SetZAxisLabelText("depth (A-Scan)");
  actor->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
  actor->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
  actor->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
  actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(15);
  actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(15);
  actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(15);
  actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
  actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
  actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
  actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
  actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
  actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
  actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
  actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1.0, 0.0,
                                                                      0.0);
  actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 1.0,
                                                                      0.0);
  actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0,
                                                                      1.0);
  actor->SetUserTransform(trans);

  m_renderer->AddActor(actor);

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();
}

void Form::renderOCTVolumePolyData() {
  uint32_t num_pts = m_oct_poly_data->GetPointData()->GetNumberOfTuples();

  if (num_pts == 0) {
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

  for (uint32_t i = 0; i < num_pts; i++) {
    value = old_data_array->GetValue(i);
    //    std::cout << "x: " << (old_points->GetPoint(i))[0] <<
    //          "\t\ty: " << (old_points->GetPoint(i))[1] <<
    //          "\t\tz: " << (old_points->GetPoint(i))[2] <<
    //          "\t\tI: " << (unsigned int)value << std::endl;

    if (value >= m_min_vis_thresh && value <= m_max_vis_thresh) {
      new_points->InsertNextPoint(old_points->GetPoint(i));
      new_data_array->InsertNextValue(value);
    }
  }

  this->statusBar()->showMessage("Preparing PolyData for display... done!");
  QApplication::processEvents();

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

void Form::renderPolyDataSurface(vtkSmartPointer<vtkPolyData> cloud_poly_data,
                                 vtkSmartPointer<vtkActor> actor) {
  int num_pts = cloud_poly_data->GetNumberOfPoints();

  if (num_pts == 0) {
    qDebug() << "cloud_poly_data is empty!";
    return;
  }

  if (actor == m_oct_surf_actor) {
    VTK_NEW(vtkDelaunay2D, delaunay_filter);
    delaunay_filter->SetInput(cloud_poly_data);
    delaunay_filter->SetTolerance(0.001);

    VTK_NEW(vtkTransformFilter, trans_filter)
    trans_filter->SetInputConnection(delaunay_filter->GetOutputPort());
    trans_filter->SetTransform(m_oct_stereo_trans);

    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(trans_filter->GetOutputPort());

    actor->SetMapper(mapper);

    m_renderer->AddActor(actor);

    this->statusBar()->showMessage("Rendering OCT surface... ");
    QApplication::processEvents();

    this->m_ui->qvtkWidget->update();
    QApplication::processEvents();

    this->statusBar()->showMessage("Rendering OCT surface... done!");
    QApplication::processEvents();

    return;
  } else if (actor == m_stereo_depth_actor) {
    VTK_NEW(vtkVertexGlyphFilter, vert_filter);
    vert_filter->SetInput(cloud_poly_data);

    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(vert_filter->GetOutputPort());

    actor->SetMapper(mapper);

    m_renderer->AddActor(actor);

    this->statusBar()->showMessage("Rendering depth map... ");
    QApplication::processEvents();

    this->m_ui->qvtkWidget->update();
    QApplication::processEvents();

    this->statusBar()->showMessage("Rendering depth map... done!");
    QApplication::processEvents();
  }
}

void Form::render2DImageData(vtkSmartPointer<vtkImageData> image_data) {
  // Calculate the image and window's aspect ratios
  int* window_sizes = this->m_ui->qvtkWidget->GetRenderWindow()->GetSize();
  double window_width, window_height;
  window_width = window_sizes[0];
  window_height = window_sizes[1];

  double window_aspect_ratio = window_width / window_height;

  int* image_sizes = image_data->GetExtent();
  double image_width, image_height;
  image_width = image_sizes[1];
  image_height = image_sizes[3];

  double image_aspect_ratio = image_width / image_height;

  // Positions and rescales the image to show the largest possible size while
  // keeping the same aspect ratio
  double scaling = 1;
  if (window_aspect_ratio >= image_aspect_ratio) {
    scaling = image_height / window_height;
    m_stereo_2d_actor->SetPosition((window_width - image_width / scaling) / 2,
                                   0);
  } else {
    scaling = image_width / window_width;
    m_stereo_2d_actor->SetPosition(
        0, (window_height - image_height / scaling) / 2);
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

void Form::renderOCTMass(vtkSmartPointer<vtkActor> actor,
                         vtkSmartPointer<vtkPolyData> surf) {
  uint32_t num_pts = m_oct_poly_data->GetPointData()->GetNumberOfTuples();
  uint32_t num_surf_pts = surf->GetNumberOfPoints();

  if (num_pts == 0) {
    qDebug() << "m_oct_poly_data is empty!";
    return;
  }

  m_waiting_response = true;
  updateUIStates();

  // Determine increments
  float length_incrm = 1;
  float width_incrm = 1;
  float depth_incrm = 2.762 / 1024.0;
  if (m_current_params.length_range != 0)
    length_incrm =
        m_current_params.length_range / m_current_params.length_steps;
  if (m_current_params.width_range != 0)
    width_incrm = m_current_params.width_range / m_current_params.width_steps;
  if (m_current_params.depth_range != 0)
    depth_incrm = m_current_params.depth_range / m_current_params.depth_steps;

  this->statusBar()->showMessage("Building height plane... ");
  QApplication::processEvents();

  double height_plane[m_current_params.length_steps]
                     [m_current_params.width_steps];

  for (int i = 0; i < num_surf_pts; ++i) {
    double coords[3];
    surf->GetPoint(i, coords);

    height_plane[int(coords[0] / length_incrm)][int(coords[1] / width_incrm)] =
        coords[2];
  }

  // Lets release the surface since we don't need it anymore
  surf = NULL;

  // No smart pointers here since we force-delete these later to conserve RAM
  vtkPoints* under_pts = vtkPoints::New();
  vtkTypeUInt8Array* under_vals = vtkTypeUInt8Array::New();

  this->statusBar()->showMessage("Extracting volume under OCT surface... ");
  QApplication::processEvents();

  for (uint32_t i = 0; i < num_pts; ++i) {
    // Grabs a point in the volume
    double coords[3];
    double val;
    m_oct_poly_data->GetPoint(i, coords);
    m_oct_poly_data->GetPointData()->GetScalars()->GetTuple(i, &val);

    // The point is under the surface
    if (height_plane[int(coords[0] / length_incrm)]
                    [int(coords[1] / width_incrm)] < coords[2]) {
      under_pts->InsertNextPoint(coords);
      under_vals->InsertNextValue((uint8_t)val);
    }
  }

  // Free up some resources
  m_oct_poly_data = NULL;

  // No smart pointers here since we force-delete this later to conserve RAM
  vtkImageData* image = vtkImageData::New();
  image->SetExtent(0, m_current_params.length_steps - 1, 0,  // here
                   m_current_params.width_steps - 1, 0,
                   m_current_params.depth_steps - 1);
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToUnsignedChar();
  image->SetSpacing(length_incrm, width_incrm, depth_incrm);

  this->statusBar()->showMessage("Constructing imagedata... ");
  QApplication::processEvents();

  // Initializes the imagedata to null (filled with garbage otherwise)
  for (int i = 0; i < m_current_params.length_steps; i++) {  // here
    for (int j = 0; j < m_current_params.width_steps; j++) {
      for (int k = 0; k < m_current_params.depth_steps; k++) {

        unsigned char* pixel =
            static_cast<unsigned char*>(image->GetScalarPointer(i, j, k));

        pixel[0] = 0;
      }
    }
  }

  // Adds the scalar values from the sub-surface oct volume to the imagedata
  uint32_t num_under_pts = under_pts->GetNumberOfPoints();
  uint8_t value;
  for (uint32_t i = 0; i < num_under_pts; i++) {
    value = under_vals->GetValue(i);

    double point_coords[3];
    under_pts->GetPoint(i, point_coords);

    //    if (int(point_coords[0] / (1.0 * length_incrm)) <  // here
    //            m_current_params.length_steps / 2 &&
    //        int(point_coords[1] / (1.0 * width_incrm)) <
    //            m_current_params.width_steps / 2) {

    unsigned char* pixel = static_cast<unsigned char*>(
        image->GetScalarPointer(int(point_coords[0] / (1.0 * length_incrm)),
                                int(point_coords[1] / (1.0 * width_incrm)),
                                int(point_coords[2] / (1.0 * depth_incrm))));
    pixel[0] = value;
    //    }
  }

  // Now that we used our sub-volume points and scalars, we clear them up
  under_pts->Delete();
  under_vals->Delete();

  //  for (int x = 0; x < m_current_params.length_steps; ++x) {
  //    for (int y = 0; y < m_current_params.width_steps; ++y) {
  //      unsigned char* pixel =
  //          static_cast<unsigned char*>(image->GetScalarPointer(x, y, 100));
  //      std::cout << (unsigned int)pixel[0] << " ";
  //    }
  //    std::cout << std::endl;
  //  }

  this->statusBar()->showMessage("Applying convolution... ");
  QApplication::processEvents();

  //  VTK_NEW(vtkImageConvolve, conv_filt);
  //  conv_filt->SetInput(image);
  //  double kern[7 * 7 * 7];
  //  for (int i = 0; i < 7 * 7 * 7; i++) {
  //    kern[i] = 1.0 / (7.0*7.0*7.0);
  //  }
  //  conv_filt->SetKernel7x7x7(kern);
  //  conv_filt->Update();

  VTK_NEW(vtkImageFFT, fft_filt);
  fft_filt->SetInput(image);
  fft_filt->Update();
  fft_filt->GetOutput()->ReleaseDataFlagOn();

  // Clear our imagedata since we will be using its FFT from now on
  image->Delete();

  this->statusBar()->showMessage("Applying low-pass filter... ");
  QApplication::processEvents();

  VTK_NEW(vtkImageIdealLowPass, lowpass_filt);
  lowpass_filt->SetInputConnection(fft_filt->GetOutputPort());
  lowpass_filt->SetCutOff(3, 3, 3);
  lowpass_filt->Update();
  lowpass_filt->GetOutput()->ReleaseDataFlagOn();

  VTK_NEW(vtkImageRFFT, rfft_filt);
  rfft_filt->SetInputConnection(lowpass_filt->GetOutputPort());
  rfft_filt->Update();
  rfft_filt->GetOutput()->ReleaseDataFlagOn();

  // Get rid of imaginary components generated by RFFT
  VTK_NEW(vtkImageExtractComponents, extract_filt);
  extract_filt->SetInputConnection(rfft_filt->GetOutputPort());
  extract_filt->SetComponents(0);
  extract_filt->Update();
  extract_filt->GetOutput()->ReleaseDataFlagOn();

  this->statusBar()->showMessage("Calculating divergent... ");
  QApplication::processEvents();

  VTK_NEW(vtkImageShiftScale, type_filt);
  type_filt->SetInputConnection(extract_filt->GetOutputPort());
  type_filt->SetOutputScalarTypeToUnsignedChar();
  type_filt->Update();
  type_filt->GetOutput()->ReleaseDataFlagOn();
  type_filt->GetOutput()->Print(std::cout << "Type filter:");

  //  VTK_NEW(vtkImageDivergence, div_filt);
  //  div_filt->SetInputConnection(rfft_filt->GetOutputPort());
  //  div_filt->Update();

  //  VTK_NEW(vtkImageGradient, grad_filt);
  //  grad_filt->SetInputConnection(extract_filt->GetOutputPort());
  //  grad_filt->SetDimensionality(3);
  //  grad_filt->Update();
  //  grad_filt->GetOutput()->Print(std::cout);

  this->statusBar()->showMessage("Extracting contours... ");
  QApplication::processEvents();

  uint8_t num_contours = 1;
  //  VTK_NEW(vtkContourFilter, cont_filt);
  //  cont_filt->SetInputConnection(rfft_filt->GetOutputPort());
  //  // cont_filt->GenerateValues(num_contours, 0, 50);
  //  cont_filt->SetValue(0, 40);
  //  cont_filt->Update();
  //  double contours[num_contours];
  //  cont_filt->GetValues(contours);

  // Grab X component of the gradient
  //  VTK_NEW(vtkImageExtractComponents, extract_x_filt);
  //  extract_x_filt->SetInputConnection(grad_filt->GetOutputPort());
  //  extract_x_filt->SetComponents(0);
  //  extract_x_filt->Update();

  //  //Grab Y component of the gradient
  //  VTK_NEW(vtkImageExtractComponents, extract_y_filt);
  //  extract_y_filt->SetInputConnection(grad_filt->GetOutputPort());
  //  extract_y_filt->SetComponents(0);
  //  extract_y_filt->Update();

  //  // Gradient might be negative
  //  VTK_NEW(vtkImageMathematics, abs_x_filt);
  //  abs_x_filt->SetOperationToAbsoluteValue();
  //  abs_x_filt->SetInputConnection(extract_x_filt->GetOutputPort());
  //  abs_x_filt->Update();

  //  // Gradient might be negative
  //  VTK_NEW(vtkImageMathematics, abs_y_filt);
  //  abs_y_filt->SetOperationToAbsoluteValue();
  //  abs_y_filt->SetInputConnection(extract_y_filt->GetOutputPort());
  //  abs_y_filt->Update();

  //  // Add the two gradients
  //  VTK_NEW(vtkImageMathematics, sum_filt);
  //  sum_filt->SetOperationToAdd();
  //  sum_filt->SetInput1(abs_x_filt->GetOutput());
  //  sum_filt->SetInput2(abs_y_filt->GetOutput());
  //  sum_filt->Update();

  VTK_NEW(vtkImageGradientMagnitude, gradmag_filt);
  gradmag_filt->SetInputConnection(type_filt->GetOutputPort());
  gradmag_filt->Update();
  gradmag_filt->GetOutput()->ReleaseDataFlagOn();
  gradmag_filt->GetOutput()->Print(std::cout << "Grad filter:");

  vtkSmartPointer<vtkImageData> grad_output = gradmag_filt->GetOutput();
  discardImageSides(grad_output, 0.02, 0.02);

  VTK_NEW(vtkImageContinuousErode3D, erode_filt);
  erode_filt->SetInput(grad_output);
  // erode_filt->SetInputConnection(gradmag_filt->GetOutputPort());
  erode_filt->SetKernelSize(5, 5, 1);
  erode_filt->Update();
  //erode_filt->GetOutput()->ReleaseDataFlagOn();

  VTK_NEW(vtkImageContinuousDilate3D, dilate_filt);
  dilate_filt->SetInputConnection(erode_filt->GetOutputPort());
  dilate_filt->SetKernelSize(10, 10, 2);
  dilate_filt->Update();
  //dilate_filt->GetOutput()->ReleaseDataFlagOn();

  VTK_NEW(vtkImageContinuousErode3D, erode_filt2);
  erode_filt2->SetInputConnection(dilate_filt->GetOutputPort());
  erode_filt2->SetKernelSize(15, 15, 2);
  erode_filt2->Update();
  //erode_filt2->GetOutput()->ReleaseDataFlagOn();

  VTK_NEW(vtkImageMarchingCubes, cubes_filter);
  cubes_filter->SetInputConnection(dilate_filt->GetOutputPort());
  cubes_filter->SetValue(0, 60);
  // cubes_filter->ComputeGradientsOff();
  // cubes_filter->ComputeNormalsOff();
  cubes_filter->Update();
  // cubes_filter->GetOutput()->ReleaseDataFlagOn();

  this->statusBar()->showMessage("Decimating mesh... ");
  QApplication::processEvents();

  //  VTK_NEW(vtkDecimatePro, dec_filt);
  //  dec_filt->SetInputConnection(cubes_filter->GetOutputPort());
  //  dec_filt->SetTargetReduction(0.9);
  //  dec_filt->PreserveTopologyOn();
  //  dec_filt->Update();

  this->statusBar()->showMessage("Building LUT... ");
  QApplication::processEvents();

  VTK_NEW(vtkLookupTable, lut);
  lut->SetTableRange(1, num_contours);
  lut->SetNumberOfColors(num_contours);
  for (int i = 0; i < num_contours; ++i) {
    lut->SetTableValue(i, (rand() % 256) / 255.0, (rand() % 256) / 255.0,
                       (rand() % 256) / 255.0);
  }
  lut->Build();

  this->statusBar()->showMessage("Mapping... ");
  QApplication::processEvents();

  VTK_NEW(vtkPolyDataMapper, mapper);
  mapper->SetInputConnection(cubes_filter->GetOutputPort());
  mapper->SetLookupTable(lut);
  mapper->SetScalarModeToUseCellData();
  mapper->SetScalarRange(lut->GetTableRange());

  actor->SetMapper(mapper);

  m_renderer->AddActor(actor);

  this->statusBar()->showMessage("Rendering... ");
  QApplication::processEvents();

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();

  this->statusBar()->showMessage("Rendering... done!");
  QApplication::processEvents();

  m_has_oct_mass = true;
  m_waiting_response = false;
  updateUIStates();
}

//--------------UI CALLBACKS----------------------------------------------------

void Form::on_browse_button_clicked() {
  // getOpenFileName displays a file dialog and returns the full file path of
  // the selected file, or an empty string if the user canceled the dialog
  // The tr() function makes the dialog language proof (chinese characters, etc)
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setNameFilter(tr("Image Files (*.img);;Text Files (*.txt)"));
  QString file_name = dialog.getOpenFileName(this);

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    m_viewing_overlay = false;
    updateUIStates();

    // Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    std::vector<uint8_t> data;
    m_file_manager->readVector(file_name.toStdString().c_str(), data);

    this->m_ui->status_bar->showMessage("Loading data into point cloud... ");
    QApplication::processEvents();

    // Data opened with the browse button should always already be processed
    // Just interpret the header, nothing else, since it should be filtered
    processOCTHeader(data);

    // Immediately write our headerless vector to cache so we can segment
    // Or register using this data
    m_file_manager->writeVector(data, OCT_RAW_CACHE_PATH);

    loadVectorToPolyData(data);

    m_renderer->RemoveAllViewProps();
    renderOCTVolumePolyData();
    m_renderer->ResetCamera();

    VTK_NEW(vtkTransform, trans);
    trans->Identity();
    renderAxes(m_oct_axes_actor, trans);

    // Updates our UI param boxes
    on_reset_params_button_clicked();
    this->m_ui->file_name_lineedit->setText(file_name);

    m_has_ros_raw_oct = false;
    m_waiting_response = false;
    updateUIStates();
  }
}

void Form::on_connected_master_checkbox_clicked(bool checked) {
  m_connected_to_master = checked;
  updateUIStates();
}

void Form::on_dep_steps_spinbox_editingFinished() {
  // The axial sensor has a fixed resolution, so we change m_dep_range to match
  this->m_ui->dep_range_spinbox->setValue(
      (float)this->m_ui->dep_steps_spinbox->value() / 1024.0 * 2.762);
}

void Form::on_dep_range_spinbox_editingFinished() {
  // The axial sensor has a fixed resolution, so we change m_dep_steps to match
  this->m_ui->dep_steps_spinbox->setValue(
      (int)(this->m_ui->dep_range_spinbox->value() / 2.762 * 1024 + 0.5));
}

void Form::on_request_scan_button_clicked() {
  // Disables controls until we get a response, locking the oct params
  m_waiting_response = true;
  updateUIStates();

  // Gets the updated oct params
  m_current_params.length_steps = this->m_ui->len_steps_spinbox->value();
  m_current_params.width_steps = this->m_ui->wid_steps_spinbox->value();
  m_current_params.depth_steps = this->m_ui->dep_steps_spinbox->value();
  m_current_params.length_range = this->m_ui->len_range_spinbox->value();
  m_current_params.width_range = this->m_ui->wid_range_spinbox->value();
  m_current_params.depth_range = this->m_ui->dep_range_spinbox->value();
  m_current_params.length_offset = this->m_ui->len_off_spinbox->value();
  m_current_params.width_offset = this->m_ui->wid_off_spinbox->value();

  // Send our params to qnode so it can send a ROS service request to oct_client
  Q_EMIT requestScan(m_current_params);

  this->m_ui->status_bar->showMessage("Waiting for OCT scanner response... ");
  QApplication::processEvents();
}

void Form::on_save_button_clicked() {
  QString file_name = QFileDialog::getSaveFileName(
      this, tr("Save as img file"), "", tr("Image file (*.img)"));

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    updateUIStates();

    // Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    std::vector<uint8_t> header;
    header.resize(512);
    memcpy(&header[16], &m_current_params.length_steps, 4);
    memcpy(&header[20], &m_current_params.width_steps, 4);
    memcpy(&header[24], &m_current_params.depth_steps, 4);
    memcpy(&header[72], &m_current_params.length_range, 4);
    memcpy(&header[76], &m_current_params.width_range, 4);
    memcpy(&header[116], &m_current_params.depth_range, 4);
    memcpy(&header[120], &m_current_params.length_offset, 4);
    memcpy(&header[124], &m_current_params.width_offset, 4);

    // Write the vector
    m_file_manager->writeVector(header, file_name.toStdString().c_str());

    // Write the actual data, the true at the end of writeVector means we want
    // to append this data to the previously written header
    std::vector<uint8_t> data;
    m_file_manager->readVector(OCT_RAW_CACHE_PATH, data);
    m_file_manager->writeVector(data, file_name.toStdString().c_str(), true);

    m_waiting_response = false;
    updateUIStates();

    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_reset_params_button_clicked() {
  this->m_ui->len_steps_spinbox->setValue(m_current_params.length_steps);
  this->m_ui->wid_steps_spinbox->setValue(m_current_params.width_steps);
  this->m_ui->dep_steps_spinbox->setValue(m_current_params.depth_steps);
  this->m_ui->len_range_spinbox->setValue(m_current_params.length_range);
  this->m_ui->wid_range_spinbox->setValue(m_current_params.width_range);
  this->m_ui->dep_range_spinbox->setValue(m_current_params.depth_range);
  this->m_ui->len_off_spinbox->setValue(m_current_params.length_offset);
  this->m_ui->wid_off_spinbox->setValue(m_current_params.width_offset);
}

void Form::on_view_raw_oct_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering OCT volume data... ");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);

  this->m_ui->status_bar->showMessage("Rendering OCT volume data... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_calc_oct_surf_button_clicked() {
  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  Q_EMIT requestSegmentation(m_current_params);

  this->m_ui->status_bar->showMessage("Waiting for OCT surface response... ");
  QApplication::processEvents();
}

void Form::on_view_oct_surf_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering OCT surface... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkPolyData, surf_oct_poly_data);
  loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);
  m_renderer->RemoveAllViewProps();
  renderPolyDataSurface(surf_oct_poly_data, m_oct_surf_actor);

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);

  this->m_ui->status_bar->showMessage("Rendering OCT surface... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_calc_oct_mass_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering OCT mass... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  VTK_NEW(vtkPolyData, surf_oct_poly_data);
  loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);

  m_renderer->RemoveAllViewProps();
  renderOCTMass(m_oct_mass_actor, surf_oct_poly_data);

  this->m_ui->status_bar->showMessage("Rendering OCT mass... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_left_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering left image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  VTK_NEW(vtkImageData, left_image);
  load2DVectorCacheToImageData(STEREO_LEFT_CACHE_PATH, left_image);
  render2DImageData(left_image);

  this->m_ui->status_bar->showMessage("Rendering left image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_right_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering right image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  VTK_NEW(vtkImageData, right_image);
  load2DVectorCacheToImageData(STEREO_RIGHT_CACHE_PATH, right_image);
  render2DImageData(right_image);

  this->m_ui->status_bar->showMessage("Rendering right image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_disp_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  VTK_NEW(vtkImageData, disp_image);
  load2DVectorCacheToImageData(STEREO_DISP_CACHE_PATH, disp_image);
  render2DImageData(disp_image);

  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_depth_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  VTK_NEW(vtkPolyData, depth_image);
  loadPCLCacheToPolyData(STEREO_DEPTH_CACHE_PATH, depth_image);
  m_renderer->RemoveAllViewProps();
  renderPolyDataSurface(depth_image, m_stereo_depth_actor);

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_trans_axes_actor, m_oct_stereo_trans);

  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_calc_transform_button_clicked() {
  m_waiting_response = true;
  updateUIStates();

  Q_EMIT requestRegistration();

  this->m_ui->status_bar->showMessage(
      "Waiting for OCT registration "
      "transform... ");
  QApplication::processEvents();
}

void Form::on_print_transform_button_clicked() {

  this->m_ui->status_bar->showMessage("Printing transform to console... ");
  QApplication::processEvents();

  for (int row = 0; row < 4; row++) {
    std::cout << "\t";
    for (int col = 0; col < 4; col++) {
      std::cout << m_oct_stereo_trans->GetMatrix()->GetElement(row, col)
                << "\t";
    }
    std::cout << "\n";
  }
}

void Form::on_raw_min_vis_spinbox_editingFinished() {
  uint8_t new_value = m_ui->raw_min_vis_spinbox->value();

  // Update the slider to the side. This will not call a redraw. The actual
  // m_min_vis_thresh gets updated on the slider callback
  m_ui->raw_min_vis_slider->setValue(new_value);

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);
}

void Form::on_raw_max_vis_spinbox_editingFinished() {
  uint8_t new_value = m_ui->raw_max_vis_spinbox->value();

  // Update the slider to the side. This will not call a redraw. The actual
  // m_min_vis_thresh gets updated on the slider callback
  m_ui->raw_max_vis_slider->setValue(new_value);

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);
}

void Form::on_raw_min_vis_slider_valueChanged(int value) {
  m_min_vis_thresh = value;
  m_ui->raw_min_vis_spinbox->setValue(value);

  // Updates the equivalent slider on the Overlay tab
  m_ui->over_min_vis_slider->setValue(value);

  // Updates the max slider with a sensible value
  if (value > m_max_vis_thresh) m_ui->raw_max_vis_slider->setValue(value);
}

void Form::on_raw_max_vis_slider_valueChanged(int value) {
  m_max_vis_thresh = value;
  m_ui->raw_max_vis_spinbox->setValue(value);

  // Updates the equivalent slider on the Overlay tab
  m_ui->over_max_vis_slider->setValue(value);

  // Updates the min slider with a sensible value
  if (value < m_min_vis_thresh) m_ui->raw_min_vis_slider->setValue(value);
}

void Form::on_raw_min_vis_slider_sliderReleased() {
  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);
}

void Form::on_raw_max_vis_slider_sliderReleased() {
  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);
}

void Form::on_over_min_vis_spinbox_editingFinished() {
  uint8_t new_value = m_ui->over_min_vis_spinbox->value();

  // Update the slider to the side. This will not call a redraw. The actual
  // m_min_vis_thresh gets updated on the slider callback
  m_ui->over_min_vis_slider->setValue(new_value);

  m_renderer->RemoveActor(m_oct_vol_actor);
  renderOCTVolumePolyData();
}

void Form::on_over_max_vis_spinbox_editingFinished() {
  uint8_t new_value = m_ui->over_max_vis_spinbox->value();

  // Update the slider to the side. This will not call a redraw. The actual
  // m_min_vis_thresh gets updated on the slider callback
  m_ui->over_max_vis_slider->setValue(new_value);

  m_renderer->RemoveActor(m_oct_vol_actor);
  renderOCTVolumePolyData();
}

void Form::on_over_min_vis_slider_valueChanged(int value) {
  m_min_vis_thresh = value;
  m_ui->over_min_vis_spinbox->setValue(value);

  // Updates the equivalent slider on the OCT tab
  m_ui->raw_min_vis_slider->setValue(value);

  // Updates the max slider with a sensible value
  if (value > m_max_vis_thresh) m_ui->over_max_vis_slider->setValue(value);
}

void Form::on_over_max_vis_slider_valueChanged(int value) {
  m_max_vis_thresh = value;
  m_ui->over_max_vis_spinbox->setValue(value);

  // Updates the equivalent slider on the OCT tab
  m_ui->raw_max_vis_slider->setValue(value);

  // Updates the min slider with a sensible value
  if (value < m_min_vis_thresh) m_ui->over_min_vis_slider->setValue(value);
}

void Form::on_over_min_vis_slider_sliderReleased() {
  m_renderer->RemoveActor(m_oct_vol_actor);
  renderOCTVolumePolyData();
}

void Form::on_over_max_vis_slider_sliderReleased() {
  m_renderer->RemoveActor(m_oct_vol_actor);
  renderOCTVolumePolyData();
}

void Form::on_over_raw_checkbox_clicked() {
  // If we started viewing overlay from another tab, then clear actors
  if (!m_viewing_overlay) {
    m_renderer->RemoveAllViewProps();
    m_viewing_overlay = true;
  }
  m_waiting_response = true;
  updateUIStates();

  if (m_ui->over_raw_checkbox->isChecked()) {
    this->m_ui->status_bar->showMessage(
        "Adding OCT raw data to overlay view...");
    QApplication::processEvents();

    renderOCTVolumePolyData();
  } else {
    this->m_ui->status_bar->showMessage(
        "Removing OCT raw data from overlay view...", 3000);
    QApplication::processEvents();

    m_renderer->RemoveActor(m_oct_vol_actor);
    this->m_ui->qvtkWidget->update();
  }

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_over_oct_surf_checkbox_clicked() {
  // If we started viewing overlay from another tab, then clear actors
  if (!m_viewing_overlay) {
    m_renderer->RemoveAllViewProps();
    m_viewing_overlay = true;
  }
  m_waiting_response = true;
  updateUIStates();

  if (m_ui->over_oct_surf_checkbox->isChecked()) {
    this->m_ui->status_bar->showMessage(
        "Adding OCT surface to overlay view...");
    QApplication::processEvents();

    VTK_NEW(vtkPolyData, surf_oct_poly_data);
    loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);
    renderPolyDataSurface(surf_oct_poly_data, m_oct_surf_actor);
  } else {
    this->m_ui->status_bar->showMessage(
        "Removing OCT surface from overlay view...", 3000);
    QApplication::processEvents();

    m_renderer->RemoveActor(m_oct_surf_actor);
    this->m_ui->qvtkWidget->update();
  }

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_over_oct_mass_checkbox_clicked() {
  // If we started viewing overlay from another tab, then clear actors
  if (!m_viewing_overlay) {
    m_renderer->RemoveAllViewProps();
    m_viewing_overlay = true;
  }
  m_waiting_response = true;
  updateUIStates();

  if (m_ui->over_oct_mass_checkbox->isChecked()) {
    this->m_ui->status_bar->showMessage("Adding OCT mass to overlay view...");
    QApplication::processEvents();

    VTK_NEW(vtkPolyData, surf_oct_poly_data);
    loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);

    m_renderer->RemoveAllViewProps();
    renderOCTMass(m_oct_mass_actor, surf_oct_poly_data);
  } else {
    this->m_ui->status_bar->showMessage(
        "Removing OCT mass from overlay view...", 3000);
    QApplication::processEvents();

    m_renderer->RemoveActor(m_oct_mass_actor);
    this->m_ui->qvtkWidget->update();
  }

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_over_oct_axes_checkbox_clicked() {
  // If we started viewing overlay from another tab, then clear actors
  if (!m_viewing_overlay) {
    m_renderer->RemoveAllViewProps();
    m_viewing_overlay = true;
  }
  m_waiting_response = true;
  updateUIStates();

  if (m_ui->over_oct_axes_checkbox->isChecked()) {
    this->m_ui->status_bar->showMessage(
        "Adding OCT axes actor to overlay "
        "view...");
    QApplication::processEvents();

    VTK_NEW(vtkTransform, trans);
    trans->Identity();
    renderAxes(m_oct_axes_actor, trans);

  } else {
    this->m_ui->status_bar->showMessage(
        "Removing OCT axes actor from overlay view...", 3000);
    QApplication::processEvents();

    m_renderer->RemoveActor(m_oct_axes_actor);
    this->m_ui->qvtkWidget->update();
  }

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_over_depth_checkbox_clicked() {
  // If we started viewing overlay from another tab, then clear actors
  if (!m_viewing_overlay) {
    m_renderer->RemoveAllViewProps();
    m_viewing_overlay = true;
  }
  m_waiting_response = true;
  updateUIStates();

  if (m_ui->over_depth_checkbox->isChecked()) {
    this->m_ui->status_bar->showMessage(
        "Adding stereocamera depth map to"
        " overlay view...");
    QApplication::processEvents();

    VTK_NEW(vtkPolyData, depth_image);
    loadPCLCacheToPolyData(STEREO_DEPTH_CACHE_PATH, depth_image);
    renderPolyDataSurface(depth_image, m_stereo_depth_actor);
  } else {
    this->m_ui->status_bar->showMessage(
        "Remove stereocamera depth map from"
        " overlay view...",
        3000);
    QApplication::processEvents();
    m_renderer->RemoveActor(m_stereo_depth_actor);
    this->m_ui->qvtkWidget->update();
  }

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_over_trans_axes_checkbox_clicked() {
  // If we started viewing overlay from another tab, then clear actors
  if (!m_viewing_overlay) {
    m_renderer->RemoveAllViewProps();
    m_viewing_overlay = true;
  }
  m_waiting_response = true;
  updateUIStates();

  if (m_ui->over_trans_axes_checkbox->isChecked()) {
    this->m_ui->status_bar->showMessage(
        "Adding transformed axes actor to overlay "
        "view...");
    QApplication::processEvents();

    renderAxes(m_trans_axes_actor, m_oct_stereo_trans);

  } else {
    this->m_ui->status_bar->showMessage(
        "Removing transformed axes actor from overlay view...", 3000);
    QApplication::processEvents();

    m_renderer->RemoveActor(m_trans_axes_actor);
    this->m_ui->qvtkWidget->update();
  }

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_left_accu_reset_button_clicked() {
  m_has_left_img = false;
  updateUIStates();

  std::cout << "left accu reset called" << std::endl;
  Q_EMIT setLeftAccumulatorSize(m_ui->left_accu_spinbox->value());
  Q_EMIT resetAccumulators();
}

void Form::on_depth_accu_reset_button_clicked() {
  m_has_depth_img = false;
  updateUIStates();

  std::cout << "depth accu reset called" << std::endl;
  Q_EMIT setDepthAccumulatorSize(m_ui->depth_accu_spinbox->value());
}

//------------QNODE CALLBACKS---------------------------------------------------

void Form::receivedRawOCTData(OCTinfo params) {
  this->m_ui->status_bar->showMessage(
      "Waiting for OCT scanner response... "
      "done!");
  QApplication::processEvents();

  std::vector<uint8_t> raw_data;
  m_file_manager->readVector(OCT_RAW_CACHE_PATH, raw_data);

  m_current_params.length_steps = params.length_steps;
  m_current_params.width_steps = params.width_steps;
  m_current_params.depth_steps = params.depth_steps;
  m_current_params.length_range = params.length_range;
  m_current_params.width_range = params.width_range;
  m_current_params.depth_range = params.depth_range;
  m_current_params.length_offset = params.length_offset;
  m_current_params.width_offset = params.width_offset;

  // Process received raw data
  discardTop(raw_data, 0.1);
  medianFilter3D(raw_data);
  normalize(raw_data);

  // Write our new, filtered vector to our cache file
  m_file_manager->writeVector(raw_data, OCT_RAW_CACHE_PATH);

  loadVectorToPolyData(raw_data);

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();
  m_renderer->ResetCamera();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);

  // In case we opened a file before, clear that filename from the box
  m_ui->file_name_lineedit->clear();

  m_has_raw_oct = true;
  m_has_ros_raw_oct = true;  // Allows saving as .img
  m_waiting_response = false;
  updateUIStates();
}

void Form::receivedOCTSurfData(OCTinfo params) {
  this->m_ui->status_bar->showMessage(
      "Waiting for OCT surface response... "
      "done!");
  QApplication::processEvents();

  VTK_NEW(vtkPolyData, surf_oct_poly_data);
  loadPCLCacheToPolyData(OCT_SURF_CACHE_PATH, surf_oct_poly_data);

  m_renderer->RemoveAllViewProps();
  renderPolyDataSurface(surf_oct_poly_data, m_oct_surf_actor);
  m_renderer->ResetCamera();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);

  m_has_oct_surf = true;
  m_waiting_response = false;
  updateUIStates();
}

void Form::receivedLeftImage() {
  m_has_left_img = true;
  updateUIStates();
}

void Form::receivedRightImage() {
  m_has_right_img = true;
  updateUIStates();
}

void Form::receivedDispImage() {
  m_has_disp_img = true;
  updateUIStates();
}

void Form::receivedDepthImage() {
  m_has_depth_img = true;
  updateUIStates();
}

void Form::receivedRegistration() {
  // Read and parse the YAML 4x4 transform
  cv::Mat cv_matrix;
  m_oct_stereo_trans->Identity();
  double elements[16];

  cv::FileStorage file_stream(VIS_TRANS_CACHE_PATH, cv::FileStorage::READ);
  if (file_stream.isOpened()) {
    file_stream["octRegistrationMatrix"] >> cv_matrix;
    for (unsigned int row = 0; row < 4; row++) {
      for (unsigned int col = 0; col < 4; col++) {
        elements[row * 4 + col] = cv_matrix.at<float>(row, col);
      }
    }
  }
  m_oct_stereo_trans->SetMatrix(elements);

  m_has_transform = true;
  updateUIStates();
}
