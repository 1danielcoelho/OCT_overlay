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
  m_has_raw_oct = false;
  m_waiting_response = false;
  m_has_oct_surf = false;
  m_has_oct_mass = false;
  m_has_stereo_cache = false;
  m_has_left_image = false;
  m_has_right_image = false;
  m_has_disp_image = false;
  m_has_depth_image = false;
  m_has_transform = false;
  m_viewing_overlay = false;
  m_viewing_realtime_overlay = false;
  m_viewing_background = false;
  updateUIStates();

  // Creates qnode and it's thread, connecting signals and slots
  m_qthread = new QThread;
  m_qnode = new QNode(argc, argv);
  m_qnode->moveToThread(m_qthread);

  // Allows us to use OCTinfo structs in signals/slots
  qRegisterMetaType<OCTinfo>();
  qRegisterMetaType<std::vector<uint8_t> >();
  qRegisterMetaType<std::vector<float> >();
  qRegisterMetaType<vtkSmartPointer<vtkPolyData> >();
  qRegisterMetaType<vtkSmartPointer<vtkImageData> >();

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

  connect(m_qnode, SIGNAL(receivedStereoImages()), this,
          SLOT(receivedStereoImages()), Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(accumulated(float)), this, SLOT(accumulated(float)),
          Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(newSurface(vtkSmartPointer<vtkPolyData>)), this,
          SLOT(newSurface(vtkSmartPointer<vtkPolyData>)),
          Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(newBackground(vtkSmartPointer<vtkImageData>)), this,
          SLOT(newBackground(vtkSmartPointer<vtkImageData>)),
          Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(newEdges(std::vector<float>)), this,
          SLOT(newEdges(std::vector<float>)),
          Qt::QueuedConnection);

  connect(m_qnode, SIGNAL(receivedRegistration()), this,
          SLOT(receivedRegistration()), Qt::QueuedConnection);

  connect(this, SIGNAL(setAccumulatorSize(uint)), m_qnode,
          SLOT(setAccumulatorSize(uint)), Qt::QueuedConnection);

  connect(this, SIGNAL(resetAccumulators()), m_qnode,SLOT(resetAccumulators()),
          Qt::QueuedConnection);

  connect(this, SIGNAL(startOverlay()), m_qnode, SLOT(startOverlay()),
          Qt::QueuedConnection);

  connect(this, SIGNAL(stopOverlay()), m_qnode, SLOT(stopOverlay()),
          Qt::QueuedConnection);

  // Wire up qnode and it's thread. Don't touch this unless absolutely
  // necessary. It allows both to quit gracefully
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
  m_oct_surf_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_oct_mass_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_stereo_left_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_stereo_reconstr_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_tex_quad_poly_data = vtkSmartPointer<vtkPolyData>::New();
  m_stereo_left_image = vtkSmartPointer<vtkImageData>::New();
  m_stereo_right_image = vtkSmartPointer<vtkImageData>::New();
  m_stereo_disp_image = vtkSmartPointer<vtkImageData>::New();
  m_stereo_depth_image = vtkSmartPointer<vtkImageData>::New();
  m_oct_stereo_trans = vtkSmartPointer<vtkTransform>::New();
  m_oct_stereo_trans->Identity();
  // Actors
  m_oct_vol_actor = vtkSmartPointer<vtkActor>::New();
  m_oct_surf_actor = vtkSmartPointer<vtkActor>::New();
  m_oct_mass_actor = vtkSmartPointer<vtkActor>::New();
  m_background_actor = vtkSmartPointer<vtkActor>::New();
  m_stereo_2d_actor = vtkSmartPointer<vtkActor2D>::New();
  m_stereo_reconstr_actor = vtkSmartPointer<vtkActor>::New();
  m_stereo_left_actor = vtkSmartPointer<vtkActor>::New();
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
  m_crossbar->clearAllFiles();
}

void Form::updateUIStates() {
  // OCT page
  m_ui->connected_master_checkbox->setChecked(m_connected_to_master);
  m_ui->connected_master_checkbox_2->setChecked(m_connected_to_master);

  m_ui->browse_button->setEnabled(!m_waiting_response && !m_waiting_response);

  m_ui->request_scan_button->setEnabled(!m_waiting_response);
  m_ui->save_button->setEnabled(m_has_raw_oct && !m_waiting_response);
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
  m_ui->calc_oct_mass_button->setEnabled(m_has_raw_oct && m_has_oct_surf &&
                                         !m_waiting_response);

  m_ui->save_oct_surf_button->setEnabled(m_has_oct_surf && !m_waiting_response);
  m_ui->save_oct_mass_button->setEnabled(m_has_oct_mass && !m_waiting_response);

  m_ui->browse_oct_surf_button->setEnabled(!m_waiting_response);
  m_ui->browse_oct_mass_button->setEnabled(!m_waiting_response);

  // Stereocamera page
  m_ui->request_left_image_button->setEnabled(m_has_stereo_cache &&
                                              !m_waiting_response);
  m_ui->request_right_image_button->setEnabled(m_has_stereo_cache &&
                                               !m_waiting_response);
  m_ui->request_disp_image_button->setEnabled(m_has_stereo_cache &&
                                              !m_waiting_response);
  m_ui->request_depth_image_button->setEnabled(m_has_stereo_cache &&
                                               !m_waiting_response);

  m_ui->browse_left_image_button->setEnabled(!m_waiting_response);
  m_ui->browse_right_image_button->setEnabled(!m_waiting_response);
  m_ui->browse_disp_image_button->setEnabled(!m_waiting_response);
  m_ui->browse_depth_image_button->setEnabled(!m_waiting_response);

  m_ui->save_left_image_button->setEnabled(m_has_left_image &&
                                           !m_waiting_response);
  m_ui->save_right_image_button->setEnabled(m_has_right_image &&
                                            !m_waiting_response);
  m_ui->save_disp_image_button->setEnabled(m_has_disp_image &&
                                           !m_waiting_response);
  m_ui->save_depth_image_button->setEnabled(m_has_depth_image &&
                                            !m_waiting_response);

  m_ui->view_left_image_button->setEnabled(m_has_left_image &&
                                           !m_waiting_response);
  m_ui->view_right_image_button->setEnabled(m_has_right_image &&
                                            !m_waiting_response);
  m_ui->view_disp_image_button->setEnabled(m_has_disp_image &&
                                           !m_waiting_response);
  m_ui->view_depth_image_button->setEnabled(m_has_depth_image &&
                                            !m_waiting_response);

  m_ui->accu_spinbox->setEnabled(!m_waiting_response);
  m_ui->accu_reset_button->setEnabled(!m_waiting_response);

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
  m_ui->over_oct_axes_checkbox->setEnabled(!m_waiting_response);
  m_ui->over_depth_checkbox->setEnabled(m_has_depth_image &&
                                        !m_waiting_response);
  m_ui->over_trans_axes_checkbox->setEnabled(m_has_transform &&
                                             !m_waiting_response);

  m_ui->calc_transform_button->setEnabled(m_has_oct_surf && m_has_depth_image
                                          && !m_waiting_response);
  m_ui->print_transform_button->setEnabled(m_has_transform &&
                                           !m_waiting_response);
  m_ui->browse_transform_button->setEnabled(!m_waiting_response);
  m_ui->save_transform_button->setEnabled(!m_waiting_response &&
                                          m_has_transform);

  m_ui->over_start_button->setEnabled(m_has_stereo_cache &&
                                      !m_waiting_response &&
                                      !m_viewing_realtime_overlay);
  m_ui->over_stop_button->setEnabled(m_viewing_realtime_overlay);

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

void Form::getCenterOfMass(vtkSmartPointer<vtkPolyData> mesh,
                           std::vector<double>& com) {
  int num_pts = mesh->GetNumberOfPoints();

  assert("Mesh has zero points!" && num_pts > 0);

  // Initializes center of mass to zero
  com.clear();
  com.resize(3);

  vtkPoints* pts = mesh->GetPoints();

  // Append the sum of all points into com
  for (int i = 0; i < num_pts; i++) {
    double point[3];
    pts->GetPoint(i, point);

    com[0] += point[0];
    com[1] += point[1];
    com[2] += point[2];
  }

  com[0] = com[0] * (1.0 / num_pts);
  com[1] = com[1] * (1.0 / num_pts);
  com[2] = com[2] * (1.0 / num_pts);
}

void Form::segmentTumour(vtkSmartPointer<vtkActor> actor,
                         vtkSmartPointer<vtkPolyData> surf) {
  uint32_t num_pts = m_oct_poly_data->GetPointData()->GetNumberOfTuples();
  uint32_t num_surf_pts = surf->GetNumberOfPoints();

  assert("Can't segment tumour if m_oct_poly_data is empty!" && num_pts > 0);

  m_waiting_response = true;
  updateUIStates();

  // Determine increments, used for spacing of the data
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

#ifdef AT_HOME
  // Free up some resources
  m_oct_poly_data = NULL;
#endif  // AT_HOME

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

  // Converts our polydata into our new imagedata
  for (uint32_t i = 0; i < num_pts; i++) {
    double coords[3];
    double val;
    m_oct_poly_data->GetPoint(i, coords);
    m_oct_poly_data->GetPointData()->GetScalars()->GetTuple(i, &val);

    unsigned char* pixel = static_cast<unsigned char*>(
        image->GetScalarPointer(int(coords[0] / (1.0 * length_incrm)),
                                int(coords[1] / (1.0 * width_incrm)),
                                int(coords[2] / (1.0 * depth_incrm))));
    pixel[0] = val;
  }

  this->statusBar()->showMessage("Applying FFT... ");
  QApplication::processEvents();

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
  lowpass_filt->SetCutOff(FFT_LOWPASS_CUTOFF_X, FFT_LOWPASS_CUTOFF_Y,
                          FFT_LOWPASS_CUTOFF_Z);
  lowpass_filt->Update();
  lowpass_filt->GetOutput()->ReleaseDataFlagOn();

  this->statusBar()->showMessage("Applying RFFT... ");
  QApplication::processEvents();

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

  this->statusBar()->showMessage("Converting to unsigned char... ");
  QApplication::processEvents();

  // Cast voxel depth from double to unsigned char
  VTK_NEW(vtkImageShiftScale, type_filt);
  type_filt->SetInputConnection(extract_filt->GetOutputPort());
  type_filt->SetOutputScalarTypeToUnsignedChar();
  type_filt->Update();
  type_filt->GetOutput()->ReleaseDataFlagOn();

  this->statusBar()->showMessage("Calculating gradient... ");
  QApplication::processEvents();

  // Get the magnitude of the gradient of the imagedata
  VTK_NEW(vtkImageGradientMagnitude, gradmag_filt);
  gradmag_filt->SetInputConnection(type_filt->GetOutputPort());
  gradmag_filt->Update();
  gradmag_filt->GetOutput()->ReleaseDataFlagOn();

  this->statusBar()->showMessage("Discarding gradient edges... ");
  QApplication::processEvents();

  // Discard sides of the imagedata (harsh gradient)
  vtkSmartPointer<vtkImageData> grad_output = gradmag_filt->GetOutput();

  //SliceViewer::view3dImageData(grad_output);
  discardImageSides(grad_output, DISCARD_SIDES_PERCENT_X,
                    DISCARD_SIDES_PERCENT_Y);

  this->statusBar()->showMessage("Discarding area above surface... ");
  QApplication::processEvents();

  std::cout << "Grad output scalar type: " << grad_output->GetScalarType() << std::endl;

  // Since the surface generated by OCT_segmentation is always a single point
  // thick, we use this 2D array to store the height of the surface plane at
  // each x,y point of the volume. This is then used to determine the part of
  // the volume that sits under the surface
  double height_plane[m_current_params.length_steps]
                     [m_current_params.width_steps];

  for(int x = 0; x<m_current_params.length_steps; x++)
  {
      for(int y = 0; y<m_current_params.width_steps; y++)
      {
          height_plane[x][y] = 0;
      }
  }

  for (uint32_t i = 0; i < num_surf_pts; ++i) {
    double coords[3];
    surf->GetPoint(i, coords);

    height_plane[int(coords[0] / length_incrm)][int(coords[1] / width_incrm)] =
        coords[2] + SURFACE_THICKNESS;
  }

  for (int i = 0; i < m_current_params.length_steps; i++) {  // here
    for (int j = 0; j < m_current_params.width_steps; j++) {

        double height_plane_depth = height_plane[i][j];

        int depth_index = int(height_plane_depth/depth_incrm + 0.5);

        for (int k = 0; k < depth_index; k++) {

        unsigned char* pixel =
            static_cast<unsigned char*>(grad_output->GetScalarPointer(i, j, k));

        pixel[0] = 0;
      }
    }
  }

  this->statusBar()->showMessage("Eroding... ");
  QApplication::processEvents();

  // Eroding the sample gets rid of most of the noise, but fragments our contour
  VTK_NEW(vtkImageContinuousErode3D, erode_filt);
  erode_filt->SetInput(grad_output);
  erode_filt->SetKernelSize(ERODE_KERNEL_X, ERODE_KERNEL_Y, ERODE_KERNEL_Z);
  erode_filt->Update();
  erode_filt->GetOutput()->ReleaseDataFlagOn();

  this->statusBar()->showMessage("Dilating... ");
  QApplication::processEvents();

  // Enlarges our contour again, which helps to generate less meshes when using
  // marching cubes
  VTK_NEW(vtkImageContinuousDilate3D, dilate_filt);
  dilate_filt->SetInputConnection(erode_filt->GetOutputPort());
  dilate_filt->SetKernelSize(DILATE_KERNEL_X, DILATE_KERNEL_Y, DILATE_KERNEL_Z);
  dilate_filt->Update();  

  // After dilation, our contour is connected into a single blob, so we erode
  // again to shrink small anomalies that we merged. This does not cleave the
  // larger segments of our contour
  VTK_NEW(vtkImageContinuousErode3D, erode_filt2);
  erode_filt2->SetInputConnection(dilate_filt->GetOutputPort());
  erode_filt2->SetKernelSize(SECOND_ERODE_KERNEL_X, SECOND_ERODE_KERNEL_Y,
                             SECOND_ERODE_KERNEL_Z);
  erode_filt2->Update();

  this->statusBar()->showMessage("Normalizing to 0-255 range... ");
  QApplication::processEvents();

  double ranges[2];
  dilate_filt->GetOutput()->GetScalarRange(ranges);

  VTK_NEW(vtkImageShiftScale, cast_filter);
  cast_filter->SetInput(erode_filt2->GetOutput());
  cast_filter->SetShift(-ranges[0]);
  cast_filter->SetScale(255.0 / (ranges[1] - ranges[0]));
  cast_filter->SetOutputScalarTypeToUnsignedChar();
  cast_filter->Update();

  double thresh = SliceViewer::view3dImageData(cast_filter->GetOutput());

  this->statusBar()->showMessage("Applying marching cubes... ");
    QApplication::processEvents();

  VTK_NEW(vtkImageMarchingCubes, cubes_filter);
  cubes_filter->SetInput(cast_filter->GetOutput());
  cubes_filter->SetValue(0, thresh);
  cubes_filter->ComputeGradientsOff();
  cubes_filter->ComputeNormalsOff();
  cubes_filter->ComputeScalarsOff();
  cubes_filter->Update();

  //SliceViewer::viewPolyData(cubes_filter->GetOutput());

  this->statusBar()->showMessage("Determining number of meshes... ");
  QApplication::processEvents();

  // Initially we use this to get the total number of regions
  VTK_NEW(vtkPolyDataConnectivityFilter, con_filter);
  con_filter->SetInputConnection(cubes_filter->GetOutputPort());
  con_filter->ColorRegionsOn();
  con_filter->SetScalarConnectivity(0);
  con_filter->SetExtractionModeToAllRegions();
  con_filter->Update();
  int num_regions = con_filter->GetNumberOfExtractedRegions();

  // Each cluster holds a set of regions that should be melded together
  std::vector<Cluster> clusters;

  // Now we'll use this to extract every single region, one at a time
  con_filter->SetExtractionModeToSpecifiedRegions();

  // Used to get the volume approximation of a mesh
  VTK_NEW(vtkMassProperties, mass_filter);

  // Check inside the loop to understand the point of this
  VTK_NEW(vtkPolyDataConnectivityFilter, clean_pts_filter);
  clean_pts_filter->SetExtractionModeToLargestRegion();
  clean_pts_filter->SetScalarConnectivity(0);

  for (int i = 0; i < num_regions; ++i) {
    this->statusBar()->showMessage("Analysing mesh " + QString::number(i + 1) +
                                   " of " + QString::number(num_regions) +
                                   "... ");
    QApplication::processEvents();

    con_filter->InitializeSpecifiedRegionList();
    con_filter->AddSpecifiedRegion(i);
    con_filter->Update();

    // At this point, the mesh resulting from con_filter still has the points
    // from ALL meshes, even though it only has the cells (triangles) of the
    // desired mesh. To fix this, we deep-copy this mesh, and run through
    // another connectivity filter: This removes the extra points, and doesn't
    // take too long. This is sort of a glitch, but works perfectly and
    // efficiently
    VTK_NEW(vtkPolyData, mesh);
    mesh->DeepCopy(con_filter->GetOutput());

    clean_pts_filter->SetInput(mesh);
    clean_pts_filter->Update();

    mass_filter->SetInputConnection(clean_pts_filter->GetOutputPort());
    mass_filter->Update();

    double volume = mass_filter->GetVolume();

    // 0.02 is a reasonable size. Smaller than this would probably be noise
    if (volume > MIN_VOLUME) {
      // Compute the region's center of mass
      std::vector<double> com;
      getCenterOfMass(clean_pts_filter->GetOutput(), com);

      Cluster cluster;
      cluster.cbrt = std::pow(volume, 1.0 / 3.0);
      cluster.com[0] = com[0];
      cluster.com[1] = com[1];
      cluster.com[2] = com[2];
      cluster.indices.push_back(i);

      clusters.push_back(cluster);
    }
  }

  this->statusBar()->showMessage(
      "Performing hierarchical clustering on mesh segments... ");
  QApplication::processEvents();

  // Perform hierarchical clustering on the regions until the maximum similarity
  // between two different regions is below a threshold
  Clustering::hierarchicalClustering(clusters);
  int num_clusters = clusters.size();

  //  We need to hold the polydatas that will be created in the for loop below
  std::vector<vtkSmartPointer<vtkPolyData> > blobs;
  blobs.resize(num_clusters);

  for (int i = 0; i < num_clusters; i++) {
    // Select all the meshes belonging to this region
    con_filter->InitializeSpecifiedRegionList();
    for (unsigned int j = 0; j < clusters[i].indices.size(); j++) {
      con_filter->AddSpecifiedRegion(clusters[i].indices[j]);
    }
    con_filter->Update();

    this->statusBar()->showMessage("Grouping and cleaning mesh cluster " +
                                   QString::number(i + 1) + " of " +
                                   QString::number(num_clusters) + "... ");
    QApplication::processEvents();

    // Append the selected regions into a single polydata
    VTK_NEW(vtkAppendPolyData, append_filter);
    append_filter->SetInputConnection(con_filter->GetOutputPort());
    append_filter->Update();

    VTK_NEW(vtkCleanPolyData, clean_filter);
    clean_filter->SetInputConnection(append_filter->GetOutputPort());
    clean_filter->SetTolerance(0.01);
    clean_filter->Update();

    // Disconnect the clean filter's output from the pipeline
    vtkSmartPointer<vtkPolyData> clean_output = clean_filter->GetOutput();
    // clean_output->SetSource(NULL);
    clean_output->DeleteCells();  // Deletes cells and links

    this->statusBar()->showMessage(
        "Performing Delaunay3D triangulation on mesh cluster " +
        QString::number(i + 1) + " of " + QString::number(num_clusters) +
        "... ");
    QApplication::processEvents();

    // Generates a single enveloping mesh that envelops all individual meshes in
    // the polydata. Equivalent to wrapping the meshes in plastic
    VTK_NEW(vtkDelaunay3D, del_filter);
    del_filter->SetInput(clean_output);
    del_filter->SetTolerance(0.001);
    del_filter->Update();

    // The generated mesh is tetrahedral. Here we extract the outer triangular
    // mesh
    VTK_NEW(vtkDataSetSurfaceFilter, surf_filter);
    surf_filter->SetInputConnection(del_filter->GetOutputPort());
    surf_filter->Update();

    // Center of mass is based on the number of points. Since Delaunay3D
    // might have changed that, we get new centers
    std::vector<double> center;
    getCenterOfMass(surf_filter->GetOutput(), center);

    this->statusBar()->showMessage("Adjusting mesh cluster " +
                                   QString::number(i + 1) + " of " +
                                   QString::number(num_clusters) + "... ");
    QApplication::processEvents();

    VTK_NEW(vtkTransform, trans);
    trans->PostMultiply();  // Makes it so the transforms are concatenated on
                            // the left side
    trans->Translate(-center[0], -center[1], -center[2]);
    trans->Scale(FIT_SCALING_X, FIT_SCALING_Y, FIT_SCALING_Z);
    trans->Translate(center[0], center[1], center[2]);

    VTK_NEW(vtkTransformPolyDataFilter, transform_filter);
    transform_filter->SetInputConnection(surf_filter->GetOutputPort());
    transform_filter->SetTransform(trans);
    transform_filter->Update();

    VTK_NEW(vtkPPolyDataNormals, normals_filter);
    normals_filter->SetInputConnection(transform_filter->GetOutputPort());
    normals_filter->ComputePointNormalsOn();
    normals_filter->ComputeCellNormalsOff();
    normals_filter->ConsistencyOn();
    normals_filter->SplittingOff();
    normals_filter->FlipNormalsOff();
    normals_filter->AutoOrientNormalsOn();
    normals_filter->PieceInvariantOn();
    normals_filter->Update();

    // We'll be modifying the output of the normals filter, so its important to
    // disconnect it or it will update the filter itself whenever modified
    //    vtkPolyData* normals_output = normals_filter->GetOutput();
    //    normals_output->Register(NULL);
    //    normals_output->SetSource(NULL);
    VTK_NEW(vtkPolyData, normals_output);
    normals_output->ShallowCopy(normals_filter->GetOutput());

    int number_points = normals_output->GetNumberOfPoints();
    vtkDataArray* normals = normals_output->GetPointData()->GetArray("Normals");

    VTK_NEW(vtkPoints, new_pts);
    new_pts->SetDataTypeToFloat();
    new_pts->SetNumberOfPoints(number_points);

    for (int j = 0; j < number_points; j++) {
      double norms[3];
      double pos[3];

      normals_output->GetPoint(j, pos);
      normals->GetTuple(j, norms);

      double new_pt[3];
      new_pt[0] = pos[0] + norms[0] * (WARP_FACTOR_X);
      new_pt[1] = pos[1] + norms[1] * (WARP_FACTOR_Y);
      new_pt[2] = pos[2] + norms[2] * (WARP_FACTOR_Z);

      new_pts->InsertPoint(j, new_pt);
    }

    // Update the mesh with our shifted points
    normals_output->SetPoints(new_pts);

    //  Add the result to our vector
    blobs[i] = normals_output;
  }

  this->statusBar()->showMessage("Grouping and saving all mesh clusters... ");
  QApplication::processEvents();

  // Now that all clusters have run through delaunay3d individually, we can
  // unite all meshes in a single polydata
  VTK_NEW(vtkAppendPolyData, append_filter2);
  for (unsigned int i = 0; i < blobs.size(); i++) {
    append_filter2->AddInput(blobs[i]);
  }
  append_filter2->Update();

  m_oct_mass_poly_data = append_filter2->GetOutput();

// We cleared our oct point cloud to conserve RAM. Let's load it again
#ifdef AT_HOME
  std::vector<uint8_t> data;
  m_crossbar->readVector(OCT_RAW_CACHE_PATH, data);
  m_crossbar->ucharVectorToPolyData(data, m_current_params, m_oct_poly_data);
#endif  // AT_HOME

  renderOCTMass();

  m_has_oct_mass = true;
  m_waiting_response = false;
  updateUIStates();
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

void Form::renderOCTSurface() {

  m_waiting_response = true;
  updateUIStates();

  assert("Input vtkPolyData is NULL!" && m_oct_surf_poly_data != NULL);

  int num_pts = m_oct_surf_poly_data->GetNumberOfPoints();

  assert("Input vtkPolyData is NULL!" && num_pts > 0);

  VTK_NEW(vtkDelaunay2D, delaunay_filter);
  delaunay_filter->SetInput(m_oct_surf_poly_data);
  delaunay_filter->SetTolerance(0.001);

  //Applies the transform received from registration. Should be the
  //identity transform in case we haven't performed it yet
  VTK_NEW(vtkTransformFilter, trans_filter)
  trans_filter->SetInputConnection(delaunay_filter->GetOutputPort());
  trans_filter->SetTransform(m_oct_stereo_trans);

  VTK_NEW(vtkPolyDataMapper, mapper);
  mapper->SetInputConnection(trans_filter->GetOutputPort());

  m_oct_surf_actor->SetMapper(mapper);

  m_renderer->AddActor(m_oct_surf_actor);

  this->statusBar()->showMessage("Rendering OCT surface... ");
  QApplication::processEvents();

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();

  this->statusBar()->showMessage("Rendering OCT surface... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::renderStereocameraReconstruction() {

  m_waiting_response = true;
  updateUIStates();

  this->statusBar()->showMessage("Rendering stereocamera reconstruction... ");
  QApplication::processEvents();

  int dimensions[3];
  m_stereo_left_image->GetDimensions(dimensions);

  uint32_t rows = dimensions[1];
  uint32_t cols = dimensions[0];

  VTK_NEW(vtkTypeUInt8Array, color_array);
  color_array->SetNumberOfComponents(3);
  color_array->SetNumberOfTuples(rows * cols);
  color_array->SetName("Colors");

  VTK_NEW(vtkPoints, points);
  points->SetNumberOfPoints(rows * cols);

  vtkIdType point_id = 0;
  for(uint32_t i = 0; i < rows; i++)
  {
      for(uint32_t j = 0; j < cols; j++)
      {
          float* coords = static_cast<float*>(
                      m_stereo_depth_image->GetScalarPointer(j, i, 0));
          uint8_t* color = static_cast<uint8_t*>(
                      m_stereo_left_image->GetScalarPointer(j, i, 0));

          float color_float[3];
          color_float[0] = (float)color[0];
          color_float[1] = (float)color[1];
          color_float[2] = (float)color[2];

          points->SetPoint(point_id, coords[0], coords[1], coords[2]);
          color_array->SetTuple(point_id, color_float);

          point_id++;
      }
  }

  m_stereo_reconstr_poly_data->SetPoints(points);
  m_stereo_reconstr_poly_data->GetPointData()->SetScalars(color_array);

  VTK_NEW(vtkVertexGlyphFilter, vert_filter);
  vert_filter->SetInput(m_stereo_reconstr_poly_data);

  VTK_NEW(vtkPolyDataMapper, mapper);
  mapper->SetInputConnection(vert_filter->GetOutputPort());

  m_stereo_reconstr_actor->SetMapper(mapper);

  m_renderer->AddActor(m_stereo_reconstr_actor);

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();

  this->statusBar()->showMessage("Rendering stereocamera reconstruction... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::render2DImageData(vtkSmartPointer<vtkImageData> image_data) {

  m_waiting_response = true;
  updateUIStates();

  this->statusBar()->showMessage("Rendering 2D Image... ");
  QApplication::processEvents();

  if(image_data->GetScalarType() == VTK_FLOAT)
  {
      std::cout << "Input 2D Image data is not of float"
                   " type. Casting to uchar..." << std::endl;

      double ranges[2];
      image_data->GetScalarRange(ranges);

      std::cout << "Scalar ranges: " << ranges[0] << " to " << ranges[1] << std::endl;

      VTK_NEW(vtkImageShiftScale, cast_filter);
      cast_filter->SetInput(image_data);
      cast_filter->SetShift(-ranges[0]);
      cast_filter->SetScale(1);
      cast_filter->SetOutputScalarTypeToUnsignedChar();
      cast_filter->Update();

      image_data = cast_filter->GetOutput();


      image_data->GetScalarRange(ranges);
      std::cout << "Scalar ranges after scaling: " << ranges[0] << " to " << ranges[1] << std::endl;
  }

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

  this->m_ui->qvtkWidget->update();

  this->statusBar()->showMessage("Rendering 2D Image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

}

void Form::renderOCTMass() {
  m_waiting_response = true;
  updateUIStates();

  assert("m_oct_mass_poly_data is empty!" &&
         m_oct_mass_poly_data->GetNumberOfCells() > 0);

  this->statusBar()->showMessage("Rendering OCT mass... ");
  QApplication::processEvents();

  //Applies the transform received from registration. Should be the
  //identity transform in case we haven't performed it yet
  VTK_NEW(vtkTransformFilter, trans_filter)
  trans_filter->SetInput(m_oct_mass_poly_data);
  trans_filter->SetTransform(m_oct_stereo_trans);

  VTK_NEW(vtkPolyDataMapper, mapper);
  mapper->SetInputConnection(trans_filter->GetOutputPort());
  mapper->SetScalarModeToDefault();
  mapper->SetScalarVisibility(1);

  m_oct_mass_actor->SetMapper(mapper);
  //m_oct_mass_actor->GetProperty()->SetColor(0.8d, 0.8d, 1.0d);

  m_renderer->AddActor(m_oct_mass_actor);

  this->m_ui->qvtkWidget->update();
  QApplication::processEvents();

  this->statusBar()->showMessage("Rendering OCT mass...  done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::renderReadyStereoSurface()
{
    m_waiting_response = true;
    updateUIStates();

    VTK_NEW(vtkVertexGlyphFilter, vert);
    vert->SetInput(m_stereo_reconstr_poly_data);

    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(vert->GetOutputPort());

    m_stereo_reconstr_actor->SetMapper(mapper);

    m_renderer->AddActor(m_stereo_reconstr_actor);

    double position[3];
    double* orientation;
    m_renderer->GetActiveCamera()->GetPosition(position);
    orientation = m_renderer->GetActiveCamera()->GetOrientation();

//    std::cout << "Camera pos: " << position[0] << ", " << position[1] << ", " << position[2]
//              << ", orientation: " << orientation[0] << ", " << orientation[1] << ", " <<
//                 ", " << orientation[2] << std::endl;

    this->m_ui->qvtkWidget->update();
    QApplication::processEvents();

    m_waiting_response = false;
    updateUIStates();
}

//--------------UI CALLBACKS----------------------------------------------------


void Form::on_connected_master_checkbox_clicked(bool checked) {
  m_connected_to_master = checked;
  updateUIStates();
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

void Form::on_browse_button_clicked() {
  // getOpenFileName displays a file dialog and returns the full file path of
  // the selected file, or an empty string if the user canceled the dialog
  // The tr() function makes the dialog language proof (chinese characters, etc)
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setFilter(tr("Image Files (*.img);;Text Files (*.txt)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    m_viewing_overlay = false;
    updateUIStates();

    // Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    std::vector<uint8_t> data;
    m_crossbar->readVector(file_name.toStdString().c_str(), data);

    this->m_ui->status_bar->showMessage("Loading file into a vtkPolyData... ");
    QApplication::processEvents();

    // Data opened with the browse button should always already be processed
    // Just interpret the header, nothing else, since it should be filtered
    processOCTHeader(data);

    // Immediately write our headerless vector to cache so we can segment
    // Or register using this data
    m_crossbar->writeVector(data, OCT_RAW_CACHE_PATH);

    m_crossbar->ucharVectorToPolyData(data, m_current_params, m_oct_poly_data);

    m_renderer->RemoveAllViewProps();
    renderOCTVolumePolyData();
    m_renderer->ResetCamera();

    VTK_NEW(vtkTransform, trans);
    trans->Identity();
    renderAxes(m_oct_axes_actor, trans);

    // Updates our UI param boxes
    on_reset_params_button_clicked();

    m_has_raw_oct = true;
    m_waiting_response = false;
    updateUIStates();
  }
}

void Form::on_save_button_clicked() {
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setDefaultSuffix("img");
  dialog.setFilter(tr("OCT volume image file (*.img)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

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
    m_crossbar->writeVector(header, file_name.toStdString().c_str());

    // Write the actual data, the true at the end of writeVector means we want
    // to append this data to the previously written header
    std::vector<uint8_t> data;
    m_crossbar->readVector(OCT_RAW_CACHE_PATH, data);
    m_crossbar->writeVector(data, file_name.toStdString().c_str(), true);

    m_waiting_response = false;
    updateUIStates();

    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
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

void Form::on_calc_oct_mass_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering OCT mass... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  m_renderer->RemoveAllViewProps();
  segmentTumour(m_oct_mass_actor, m_oct_surf_poly_data);

  this->m_ui->status_bar->showMessage("Rendering OCT mass... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_browse_oct_surf_button_clicked() {
  // getOpenFileName displays a file dialog and returns the full file path of
  // the selected file, or an empty string if the user canceled the dialog
  // The tr() function makes the dialog language proof (chinese characters, etc)
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setFilter(tr("OCT surface file (*.surf)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    m_viewing_overlay = false;
    updateUIStates();

    // Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    m_crossbar->readPolyData(file_name.toStdString().c_str(),
                             m_oct_surf_poly_data);

    m_renderer->RemoveAllViewProps();
    renderOCTSurface();
    m_renderer->ResetCamera();

    VTK_NEW(vtkTransform, trans);
    trans->Identity();
    renderAxes(m_oct_axes_actor, trans);

    m_has_oct_surf = true;
    m_waiting_response = false;
    updateUIStates();
  }
}

void Form::on_browse_oct_mass_button_clicked() {
  // getOpenFileName displays a file dialog and returns the full file path of
  // the selected file, or an empty string if the user canceled the dialog
  // The tr() function makes the dialog language proof (chinese characters, etc)
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setFilter(tr("OCT mass file (*.mass)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    m_viewing_overlay = false;
    updateUIStates();

    // Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    m_crossbar->readPolyData(file_name.toStdString().c_str(),
                             m_oct_mass_poly_data);

    m_renderer->RemoveAllViewProps();
    renderOCTMass();
    m_renderer->ResetCamera();

    VTK_NEW(vtkTransform, trans);
    trans->Identity();
    renderAxes(m_oct_axes_actor, trans);

    this->m_ui->status_bar->showMessage("Reading file... done!");
    QApplication::processEvents();

    m_has_oct_mass = true;
    m_waiting_response = false;
    updateUIStates();
  }
}

void Form::on_save_oct_surf_button_clicked() {
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setDefaultSuffix("surf");
  dialog.setFilter(tr("OCT surface file (*.surf)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    updateUIStates();

    // Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    m_crossbar->writePolyData(m_oct_surf_poly_data,
                              file_name.toStdString().c_str());

    m_waiting_response = false;
    updateUIStates();

    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_save_oct_mass_button_clicked() {
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setDefaultSuffix("mass");
  dialog.setFilter(tr("OCT mass file (*.mass)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    updateUIStates();

    // Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    m_crossbar->writePolyData(m_oct_mass_poly_data,
                              file_name.toStdString().c_str());

    m_waiting_response = false;
    updateUIStates();

    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_view_oct_surf_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering OCT surface... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  m_renderer->RemoveAllViewProps();
  renderOCTSurface();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);

  this->m_ui->status_bar->showMessage("Rendering OCT surface... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_oct_mass_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering OCT tumour segmentation... ");
  QApplication::processEvents();

  m_waiting_response = true;
  updateUIStates();

  m_renderer->RemoveAllViewProps();
  renderOCTMass();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);

  this->m_ui->status_bar->showMessage(
      "Rendering OCT tumour segmentation... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}


void Form::on_accu_reset_button_clicked() {
  m_has_stereo_cache = false;
  m_ui->accu_progress->setValue(0);

  ROS_INFO("Accu reset");
  updateUIStates();

  Q_EMIT setAccumulatorSize(m_ui->accu_spinbox->value());
  Q_EMIT resetAccumulators();
}


void Form::on_request_left_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering left image...");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  std::vector<uint32_t> left_vector;
  m_crossbar->readVector(STEREO_LEFT_CACHE_PATH, left_vector);
  m_crossbar->intVectorToImageData2D(left_vector, m_stereo_left_image);

  render2DImageData(m_stereo_left_image);

  this->m_ui->status_bar->showMessage("Rendering left image... done!");
  QApplication::processEvents();

  m_has_left_image = true;
  m_waiting_response = false;
  updateUIStates();
}

void Form::on_request_right_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering right image... ");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  std::vector<uint32_t> right_vector;
  m_crossbar->readVector(STEREO_RIGHT_CACHE_PATH, right_vector);
  m_crossbar->intVectorToImageData2D(right_vector, m_stereo_right_image);

  render2DImageData(m_stereo_right_image);

  this->m_ui->status_bar->showMessage("Rendering right image... done!");
  QApplication::processEvents();

  m_has_right_image = true;
  m_waiting_response = false;
  updateUIStates();
}

void Form::on_request_disp_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering displacement image... ");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  std::vector<uint32_t> disp_vector;
  m_crossbar->readVector(STEREO_DISP_CACHE_PATH, disp_vector);
  m_crossbar->intVectorToImageData2D(disp_vector, m_stereo_disp_image);

  render2DImageData(m_stereo_disp_image);

  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_has_disp_image = true;
  m_waiting_response = false;
  updateUIStates();
}

void Form::on_request_depth_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering depth map... ");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  std::vector<float> depth_vector;
  m_crossbar->readVector(STEREO_DEPTH_CACHE_PATH, depth_vector);

  std::cout << "on_request_depth_image_button_clicked, size: " << depth_vector.size() << std::endl;

  m_crossbar->floatVectorToImageData2D(depth_vector, m_stereo_depth_image);

  render2DImageData(m_stereo_depth_image);

  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_has_depth_image = true;
  m_waiting_response = false;
  updateUIStates();
}

void Form::on_browse_left_image_button_clicked() {
  // getOpenFileName displays a file dialog and returns the full file path of
  // the selected file, or an empty string if the user canceled the dialog
  // The tr() function makes the dialog language proof (chinese characters, etc)
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setFilter(tr("Stereocamera left PNG image (*.png)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    m_viewing_overlay = false;
    updateUIStates();

    // Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    //    std::vector<uint32_t> left_vector;
    //    m_crossbar->readVector(file_name.toStdString().c_str(), left_vector);
    //    m_crossbar->intVectorToImageData2D(left_vector, m_stereo_left_image);

    VTK_NEW(vtkPNGReader, png_reader);
    png_reader->SetFileName(file_name.toStdString().c_str());
    png_reader->Update();
    m_stereo_left_image = png_reader->GetOutput();

    render2DImageData(m_stereo_left_image);

    this->m_ui->status_bar->showMessage("Reading file... done!");
    QApplication::processEvents();

    m_has_left_image = true;
    m_waiting_response = false;
    updateUIStates();
  }
}

void Form::on_browse_right_image_button_clicked() {
  // getOpenFileName displays a file dialog and returns the full file path of
  // the selected file, or an empty string if the user canceled the dialog
  // The tr() function makes the dialog language proof (chinese characters, etc)
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setFilter(tr("Stereocamera right PNG image (*.png)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    m_viewing_overlay = false;
    updateUIStates();

    // Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    //    std::vector<uint32_t> right_vector;
    //    m_crossbar->readVector(file_name.toStdString().c_str(), right_vector);
    //    m_crossbar->intVectorToImageData2D(right_vector,
    // m_stereo_right_image);

    VTK_NEW(vtkPNGReader, png_reader);
    png_reader->SetFileName(file_name.toStdString().c_str());
    png_reader->Update();
    m_stereo_right_image = png_reader->GetOutput();

    render2DImageData(m_stereo_right_image);

    this->m_ui->status_bar->showMessage("Reading file... done!");
    QApplication::processEvents();

    m_has_right_image = true;
    m_waiting_response = false;
    updateUIStates();
  }
}

void Form::on_browse_disp_image_button_clicked() {
  // getOpenFileName displays a file dialog and returns the full file path of
  // the selected file, or an empty string if the user canceled the dialog
  // The tr() function makes the dialog language proof (chinese characters, etc)
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setFilter(tr("Stereocamera displacement PNG image (*.png)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    m_viewing_overlay = false;
    updateUIStates();

    // Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    //    std::vector<uint32_t> disp_vector;
    //    m_crossbar->readVector(file_name.toStdString().c_str(), disp_vector);
    //    m_crossbar->intVectorToImageData2D(disp_vector, m_stereo_disp_image);

    VTK_NEW(vtkPNGReader, png_reader);
    png_reader->SetFileName(file_name.toStdString().c_str());
    png_reader->Update();
    m_stereo_disp_image = png_reader->GetOutput();

    render2DImageData(m_stereo_disp_image);

    this->m_ui->status_bar->showMessage("Reading file... done!");
    QApplication::processEvents();

    m_has_disp_image = true;
    m_waiting_response = false;
    updateUIStates();
  }
}

void Form::on_browse_depth_image_button_clicked() {
  // getOpenFileName displays a file dialog and returns the full file path of
  // the selected file, or an empty string if the user canceled the dialog
  // The tr() function makes the dialog language proof (chinese characters, etc)
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setFilter(tr("Stereocamera depth PNG image (*.depth)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    m_viewing_overlay = false;
    updateUIStates();

    // Allows the file dialog to close before moving on
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    std::vector<float> depth_vector;
    m_crossbar->readVector(file_name.toStdString().c_str(), depth_vector);
    m_crossbar->floatVectorToImageData2D(depth_vector, m_stereo_depth_image);

    render2DImageData(m_stereo_depth_image);

    this->m_ui->status_bar->showMessage("Reading file... done!");
    QApplication::processEvents();

    m_has_depth_image = true;
    m_waiting_response = false;
    updateUIStates();
  }
}

void Form::on_save_left_image_button_clicked() {
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setDefaultSuffix("png");
  dialog.setFilter(tr("Stereocamera left PNG image (*.png)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    updateUIStates();

    // Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    //    std::vector<uint32_t> left_vector;
    //    m_crossbar->imageData2DtoIntVector(m_stereo_left_image, left_vector);
    //    m_crossbar->writeVector(left_vector, file_name.toStdString().c_str());

    VTK_NEW(vtkPNGWriter, png_writer);
    png_writer->SetInput(m_stereo_left_image);
    png_writer->SetFileName(file_name.toStdString().c_str());
    png_writer->Write();

    m_waiting_response = false;
    updateUIStates();

    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_save_right_image_button_clicked() {
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setDefaultSuffix("png");
  dialog.setFilter(tr("Stereocamera right PNG image (*.png)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    updateUIStates();

    // Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    //    std::vector<uint32_t> right_vector;
    //    m_crossbar->imageData2DtoIntVector(m_stereo_right_image,
    // right_vector);
    //    m_crossbar->writeVector(right_vector,
    // file_name.toStdString().c_str());

    VTK_NEW(vtkPNGWriter, png_writer);
    png_writer->SetInput(m_stereo_right_image);
    png_writer->SetFileName(file_name.toStdString().c_str());
    png_writer->Write();

    m_waiting_response = false;
    updateUIStates();

    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_save_disp_image_button_clicked() {
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setDefaultSuffix("png");
  dialog.setFilter(tr("Stereocamera displacement PNG image (*.png)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    updateUIStates();

    // Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    //    std::vector<uint32_t> disp_vector;
    //    m_crossbar->imageData2DtoIntVector(m_stereo_disp_image, disp_vector);
    //    m_crossbar->writeVector(disp_vector, file_name.toStdString().c_str());

    VTK_NEW(vtkPNGWriter, png_writer);
    png_writer->SetInput(m_stereo_disp_image);
    png_writer->SetFileName(file_name.toStdString().c_str());
    png_writer->Write();

    m_waiting_response = false;
    updateUIStates();

    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_save_depth_image_button_clicked() {
  QString file_name;
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setDefaultSuffix("depth");
  dialog.setFilter(tr("Stereocamera depth image PCL file (*.depth)"));
  if (dialog.exec()) {
    file_name = dialog.selectedFiles().first();
  } else {
    return;
  }

  if (!file_name.isEmpty()) {
    m_waiting_response = true;
    updateUIStates();

    // Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    std::vector<float> depth_vector;
    m_crossbar->imageData2DToFloatVector(m_stereo_depth_image, depth_vector);
    m_crossbar->writeVector(depth_vector, file_name.toStdString().c_str(), false);

    m_waiting_response = false;
    updateUIStates();

    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}

void Form::on_view_left_image_button_clicked() {
  this->m_ui->status_bar->showMessage("Rendering left image...");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  render2DImageData(m_stereo_left_image);

  this->m_ui->status_bar->showMessage("Rendering left image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_right_image_button_clicked(){
  this->m_ui->status_bar->showMessage("Rendering right image... ");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  render2DImageData(m_stereo_right_image);

  this->m_ui->status_bar->showMessage("Rendering right image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_disp_image_button_clicked(){
  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  render2DImageData(m_stereo_disp_image);

  this->m_ui->status_bar->showMessage("Rendering displacement image... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
}

void Form::on_view_depth_image_button_clicked(){
  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_waiting_response = true;
  m_viewing_overlay = false;
  updateUIStates();

  render2DImageData(m_stereo_depth_image);

  this->m_ui->status_bar->showMessage("Rendering depth map... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  updateUIStates();
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

    renderOCTSurface();
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

    renderOCTMass();
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
        "Adding stereocamera reconstruction to"
        " overlay view...", 3000);
    QApplication::processEvents();

    //Check to see if we have everything we need
    if(m_stereo_left_image == NULL
            || m_stereo_left_image->GetNumberOfPoints() == 0)
    {
        ROS_WARN("Need to have a stereocamera left image loaded!");
        m_ui->over_depth_checkbox->setChecked(false);
        return;
    }
    if(m_stereo_depth_image == NULL ||
            m_stereo_depth_image->GetNumberOfPoints() == 0)
    {
        ROS_WARN("Need to have a stereocamera depth image loaded!");
        m_ui->over_depth_checkbox->setChecked(false);
        return;
    }

    renderStereocameraReconstruction();

    } else {
    this->m_ui->status_bar->showMessage(
        "Removing stereocamera reconstruction from"
        " overlay view...",
        3000);
    QApplication::processEvents();
    m_renderer->RemoveActor(m_stereo_reconstr_actor);
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
        "view...", 3000);
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
        "view...", 3000);
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


void Form::on_calc_transform_button_clicked() {
  m_waiting_response = true;
  updateUIStates();

  // Write our current depth image to the cache
  std::vector<float> depth_vector;
  m_crossbar->imageData2DToFloatVector(m_stereo_depth_image,
                                       depth_vector);
  m_crossbar->writeVector(depth_vector, STEREO_DEPTH_CACHE_PATH);

  // Write our current oct surface to the cache
  pcl::PointCloud<pcl::PointXYZ>::Ptr oct_surf_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  m_crossbar->VTKPolyDataToPCLxyz(m_oct_surf_poly_data, oct_surf_pcl);
  m_crossbar->writePCL(oct_surf_pcl, OCT_SURF_CACHE_PATH);

  Q_EMIT requestRegistration();

  this->m_ui->status_bar->showMessage(
      "Waiting for OCT registration transform... ");
  QApplication::processEvents();
}

void Form::on_browse_transform_button_clicked()
{
    // getOpenFileName displays a file dialog and returns the full file path of
    // the selected file, or an empty string if the user canceled the dialog
    // The tr() function makes the dialog language proof (chinese characters)
    QString file_name;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setFilter(tr("OCT to stereocamera transform binary file (*.trans)"));
    if (dialog.exec()) {
      file_name = dialog.selectedFiles().first();
    } else {
      return;
    }

    if (!file_name.isEmpty()) {
      m_waiting_response = true;
      m_viewing_overlay = false;
      updateUIStates();

      // Allows the file dialog to close before moving on
      QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

      this->m_ui->status_bar->showMessage("Reading transform binary file... ");
      QApplication::processEvents();

      m_crossbar->readTransform(file_name.toStdString().c_str(),
                                m_oct_stereo_trans);

      this->m_ui->status_bar->showMessage("Reading transform binary file... "
                                          "done!");
      QApplication::processEvents();

      m_has_transform = true;
      m_waiting_response = false;
      updateUIStates();
    }

}

void Form::on_save_transform_button_clicked()
{
    QString file_name;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDefaultSuffix("trans");
    dialog.setFilter(tr("OCT to stereocamera transform binary file (*.trans)"));
    if (dialog.exec()) {
      file_name = dialog.selectedFiles().first();
    } else {
      return;
    }

    if (!file_name.isEmpty()) {
      m_waiting_response = true;
      updateUIStates();

      // Allows the file dialog to close before resuming computations
      QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

      this->m_ui->status_bar->showMessage("Writing transform data to binary "
                                          "file...");
      QApplication::processEvents();

      m_crossbar->writeTransform(m_oct_stereo_trans,
                                 file_name.toStdString().c_str());

      this->m_ui->status_bar->showMessage("Writing transform data to binary "
                                          "file... done!");
      QApplication::processEvents();

      m_waiting_response = false;
      updateUIStates();
    }
}

void Form::on_print_transform_button_clicked() {

  if(m_has_transform == false)
  {
      ROS_WARN("No transform has been calculated yet!");
      return;
  }

  this->m_ui->status_bar->showMessage("Printing transform to console... ",
                                      3000);
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


void Form::on_over_start_button_clicked()
{
    this->m_ui->status_bar->showMessage("Opening overlayed leftcamera"
                                        "image feed... ", 3000);
    QApplication::processEvents();

    m_renderer->GetActiveCamera()->SetPosition(0, 0, -30);
    m_renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
    m_renderer->GetActiveCamera()->Yaw(0);
    m_renderer->GetActiveCamera()->Pitch(0);
    m_renderer->GetActiveCamera()->Roll(180);

    m_viewing_realtime_overlay = true;
    m_waiting_response = true;
    updateUIStates();

    Q_EMIT startOverlay();
}

void Form::on_over_stop_button_clicked()
{
    Q_EMIT stopOverlay();

    this->m_ui->status_bar->showMessage("Stopping overlayed leftcamera"
                                        "image feed... ", 3000);
    m_viewing_realtime_overlay = false;
    m_waiting_response = false;
    updateUIStates();

    //m_renderer->RemoveActor(m_stereo_left_actor);
    //m_ui->over_left_checkbox->setChecked(false);
    //this->m_ui->qvtkWidget->update();
}


//------------QNODE CALLBACKS---------------------------------------------------


void Form::receivedRawOCTData(OCTinfo params) {
  this->m_ui->status_bar->showMessage(
      "Waiting for OCT scanner response... "
      "done!");
  QApplication::processEvents();

  std::vector<uint8_t> raw_data;
  m_crossbar->readVector(OCT_RAW_CACHE_PATH, raw_data);

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
  m_crossbar->writeVector(raw_data, OCT_RAW_CACHE_PATH);

  m_crossbar->ucharVectorToPolyData(raw_data, m_current_params,
                                    m_oct_poly_data);

  m_renderer->RemoveAllViewProps();
  renderOCTVolumePolyData();
  m_renderer->ResetCamera();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);

  m_has_raw_oct = true;
  m_waiting_response = false;
  updateUIStates();
}

void Form::receivedOCTSurfData(OCTinfo params) {
  this->m_ui->status_bar->showMessage(
      "Waiting for OCT surface response... "
      "done!");
  QApplication::processEvents();

  pcl::PointCloud<pcl::PointXYZ>::Ptr surf_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  m_crossbar->readPCL(OCT_SURF_CACHE_PATH, surf_point_cloud);
  m_crossbar->PCLxyzToVTKPolyData(surf_point_cloud, m_oct_surf_poly_data);

  m_renderer->RemoveAllViewProps();
  renderOCTSurface();
  m_renderer->ResetCamera();

  VTK_NEW(vtkTransform, trans);
  trans->Identity();
  renderAxes(m_oct_axes_actor, trans);

  m_has_oct_surf = true;
  m_waiting_response = false;
  updateUIStates();
}

void Form::receivedStereoImages() {
  m_has_stereo_cache = true;
  updateUIStates();
}

void Form::accumulated(float new_ratio) {
  m_ui->accu_progress->setValue((int)(new_ratio * 100));
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

  this->m_ui->status_bar->showMessage(
      "Waiting for OCT registration transform... done!");
  QApplication::processEvents();

  m_waiting_response = false;
  m_has_transform = true;
  updateUIStates();

  //Manually calls the print button callback to print the transform to console
  on_print_transform_button_clicked();
}

void Form::newSurface(vtkSmartPointer<vtkPolyData> surf)
{
    if(m_viewing_realtime_overlay)
    {
        m_stereo_reconstr_poly_data = surf;
        renderReadyStereoSurface();

        m_waiting_response = true;
        updateUIStates();
    }
}

void Form::newBackground(vtkSmartPointer<vtkImageData> back)
{
    if(m_viewing_background)
    {

//        std::cout << "Before drawing edges: "
//                  << m_quad_edges[0] << ", "
//                  << m_quad_edges[1] << ", "
//                  << m_quad_edges[2] << ", "
//                  << m_quad_edges[3] << ", "
//                  << m_quad_edges[4] << ", "
//                  << m_quad_edges[5] << ", "
//                  << m_quad_edges[6] << ", "
//                  << m_quad_edges[7] << ", "
//                  << m_quad_edges[8] << ", "
//                  << m_quad_edges[9] << ", "
//                  << m_quad_edges[10] << ", "
//                  << m_quad_edges[11] << std::endl;

        //Create our quad, used for rendering the background while overlaying
        VTK_NEW(vtkPoints, points);
        points->InsertNextPoint(m_quad_edges[0], m_quad_edges[1], m_quad_edges[2]+5.0f);
        points->InsertNextPoint(m_quad_edges[3], m_quad_edges[4], m_quad_edges[5]+5.0f);
        points->InsertNextPoint(m_quad_edges[6], m_quad_edges[7], m_quad_edges[8]+5.0f);
        points->InsertNextPoint(m_quad_edges[9], m_quad_edges[10], m_quad_edges[11]+5.0f);

        VTK_NEW(vtkPolygon, polygon);
        polygon->GetPointIds()->SetNumberOfIds(4); //make a quad
        polygon->GetPointIds()->SetId(0, 0);
        polygon->GetPointIds()->SetId(1, 1);
        polygon->GetPointIds()->SetId(2, 2);
        polygon->GetPointIds()->SetId(3, 3);

        VTK_NEW(vtkCellArray, polygons);
        polygons->InsertNextCell(polygon);

        VTK_NEW(vtkFloatArray, tex_coords);
        tex_coords->SetNumberOfComponents(3);
        tex_coords->SetName("TextureCoordinates");

        float tuple[3] = {1.0, 1.0, 0.0};
        tex_coords->InsertNextTuple(tuple);
        tuple[0] = 1.0; tuple[1] = 0.0; tuple[2] = 0.0;
        tex_coords->InsertNextTuple(tuple);
        tuple[0] = 0.0; tuple[1] = 0.0; tuple[2] = 0.0;
        tex_coords->InsertNextTuple(tuple);
        tuple[0] = 0.0; tuple[1] = 1.0; tuple[2] = 0.0;
        tex_coords->InsertNextTuple(tuple);

        m_tex_quad_poly_data->SetPoints(points);
        m_tex_quad_poly_data->SetPolys(polygons);
        m_tex_quad_poly_data->GetPointData()->SetTCoords(tex_coords);

        VTK_NEW(vtkTexture, texture);
        texture->SetInput(back);

        VTK_NEW(vtkTransform, trans);
        trans->Scale(1.0, 1.0, 1.0);

        VTK_NEW(vtkTransformFilter, trans_filter)
        trans_filter->SetInput(m_tex_quad_poly_data);
        trans_filter->SetTransform(trans);

        VTK_NEW(vtkPolyDataMapper, mapper);
        mapper->SetInputConnection(trans_filter->GetOutputPort());

        m_background_actor->SetMapper(mapper);
        m_background_actor->SetTexture(texture);

        m_renderer->AddActor(m_background_actor);

        this->m_ui->qvtkWidget->update();
        QApplication::processEvents();
    }
}

void Form::on_over_background_checkbox_clicked()
{
    // If we started viewing overlay from another tab, then clear actors
    if (!m_viewing_overlay) {
      m_renderer->RemoveAllViewProps();
      m_viewing_overlay = true;
    }

    if (m_ui->over_background_checkbox->isChecked()) {
      this->m_ui->status_bar->showMessage(
          "Adding stereocamera background to overlay view...");
      QApplication::processEvents();

      m_quad_edges[ 0] = 0;
      m_quad_edges[ 1] = 0;
      m_quad_edges[ 2] = 0;
      m_quad_edges[ 3] = 0;
      m_quad_edges[ 4] = 0;
      m_quad_edges[ 5] = 0;
      m_quad_edges[ 6] = 0;
      m_quad_edges[ 7] = 0;
      m_quad_edges[ 8] = 0;
      m_quad_edges[ 9] = 0;
      m_quad_edges[10] = 0;
      m_quad_edges[11] = 0;

      m_renderer->GetActiveCamera()->SetParallelProjection(1);
      m_viewing_background = true;
      updateUIStates();

    } else {
      this->m_ui->status_bar->showMessage(
          "Removing stereocamera background from overlay view...", 3000);
      QApplication::processEvents();

      m_viewing_background = false;
      updateUIStates();

      m_renderer->GetActiveCamera()->SetParallelProjection(0);
      m_renderer->RemoveActor(m_background_actor);
      this->m_ui->qvtkWidget->update();
    }
}

void Form::newEdges(std::vector<float> new_edges)
{
    //If the new quad edges belong to a more wide-spread quad, then we
    //take them
    if(new_edges[ 0] < m_quad_edges[ 0]) m_quad_edges[ 0] = new_edges[ 0];
    if(new_edges[ 1] < m_quad_edges[ 1]) m_quad_edges[ 1] = new_edges[ 1];
    if(new_edges[ 2] > m_quad_edges[ 2]) m_quad_edges[ 2] = new_edges[ 2];

    if(new_edges[ 3] > m_quad_edges[ 3]) m_quad_edges[ 3] = new_edges[ 3];
    if(new_edges[ 4] < m_quad_edges[ 4]) m_quad_edges[ 4] = new_edges[ 4];
    if(new_edges[ 5] > m_quad_edges[ 5]) m_quad_edges[ 5] = new_edges[ 5];

    if(new_edges[ 6] > m_quad_edges[ 6]) m_quad_edges[ 6] = new_edges[ 6];
    if(new_edges[ 7] > m_quad_edges[ 7]) m_quad_edges[ 7] = new_edges[ 7];
    if(new_edges[ 8] > m_quad_edges[ 8]) m_quad_edges[ 8] = new_edges[ 8];

    if(new_edges[ 9] < m_quad_edges[ 9]) m_quad_edges[ 9] = new_edges[ 9];
    if(new_edges[10] > m_quad_edges[10]) m_quad_edges[10] = new_edges[10];
    if(new_edges[11] > m_quad_edges[11]) m_quad_edges[11] = new_edges[11];
}
