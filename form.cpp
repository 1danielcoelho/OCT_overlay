#include "form.h"
#include "ui_form.h"





Form::Form(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  m_ui(new Ui::Form)
{
  m_ui->setupUi(this);

  //Initialize the oct parameters
  m_len_steps = this->m_ui->len_steps_spinbox->value();
  m_wid_steps = this->m_ui->wid_steps_spinbox->value();
  m_dep_steps = this->m_ui->dep_steps_spinbox->value();
  m_len_range = this->m_ui->len_range_spinbox->value();
  m_wid_range = this->m_ui->wid_range_spinbox->value();
  m_dep_range = this->m_ui->dep_range_spinbox->value();
  m_len_offset = this->m_ui->len_off_spinbox->value();
  m_wid_offset = this->m_ui->wid_off_spinbox->value();

  //Turn off the request scan button until we get a connection to Master
  m_ui->request_scan_button->setDisabled(true);

  //Turn off the save button until we have some received data
  m_ui->save_button->setDisabled(true);

  //Creates qnode and it's thread, connecting signals and slots
  m_qthread = new QThread;
  m_qnode = new QNode(argc, argv);
  m_qnode->moveToThread(m_qthread);
  connect(m_qnode, SIGNAL(rosMasterChanged(bool)), this,
          SLOT(on_connected_master_checkbox_clicked(bool)));
  connect(this,
          SIGNAL(requestScan(int, int, int, float, float, float, float, float)),
          m_qnode,
          SLOT(requestScan(int, int, int, float, float, float, float, float)),
          Qt::QueuedConnection); //Runs slot on receiving thread, according to
                                 //its own event loop
  connect(m_qnode, SIGNAL(receivedData()), this,
          SLOT(on_received_data_checkbox_clicked()), Qt::QueuedConnection);
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
  //Filters
  m_vert_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  //Mappers
  m_poly_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  //Actors
  m_actor = vtkSmartPointer<vtkActor>::New();
  m_axes_actor = vtkSmartPointer<vtkAxesActor>::New();
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





void Form::loadRawOCTData(std::vector<uint8_t>& oct_data,
      int file_header /*= 512*/, int frame_header /*= 40*/, int length /*= 0*/,
      int width /*= 0*/, int depth /*= 0*/, float length_range /*= 0*/,
      float width_range /*= 0*/)
{
  //Default parameters means we're opening a img file
  if(file_header == 512 && (length == 0 || width == 0 || depth == 0))
  {
    //Extract the data dimensions from the header
    std::memcpy(&length, &(oct_data[16]), 4*sizeof(uint8_t));
    std::memcpy(&width, &(oct_data[20]), 4*sizeof(uint8_t));
    std::memcpy(&depth, &(oct_data[24]), 4*sizeof(uint8_t));
    std::memcpy(&length_range, &(oct_data[72]), 4*sizeof(uint8_t));
    std::memcpy(&width_range, &(oct_data[76]), 4*sizeof(uint8_t));
  }

  //Checks for the datetime string: If it exists, this is a thorlabs img
  //file, and the frameheader needs to be 40. It should be 0 otherwise
  uint32_t checker = 0;
  std::memcpy(&checker, &(oct_data[44]), 4*sizeof(uint8_t));
  if(checker == 0) //Still 0
  {
    frame_header = 0;
  }

  float length_incrm = 1;
  float width_incrm = 1;
  float depth_incrm = 1;

  //Determines the spatial increments for each dimension
  if(length_range != 0 && width_range !=0)
  {
    length_incrm = length_range/length;
    width_incrm = width_range/width;
    depth_incrm = 2.762/1024; //Fixed axial resolution
  }

  //Creates an array for point coordinates, and one for the scalars
  VTK_NEW(vtkPoints, points);
  points->Reset();
  VTK_NEW(vtkTypeUInt8Array, dataArray);
  dataArray->Reset();

  points->SetNumberOfPoints(length * width * depth);
  dataArray->SetNumberOfValues(length * width * depth);

  //Update status bar
  this->statusBar()->showMessage("Building raw point data... ");
  QApplication::processEvents();

  int id = 0;
  for(int i = 0; i < length; i++)
  {
    for(int j = 0; j < width; j++)
    {
      for(int k = 0; k < depth; k++, id++)
      {
        //Thorlabs img files use 40 bytes of NULL between each B-scan
        int val = oct_data[id + frame_header*i + file_header];

        points->SetPoint(id, i*length_incrm, j*width_incrm, k*depth_incrm);
        dataArray->SetValue(id,val);
        //std::cout << "id: " << id << "\t\tx: " << i*length_incrm << "\t\ty: " << j*width_incrm <<
        //   "\t\tz: " << k*depth_incrm << "\t\tval: " << val << std::endl;
      }
    }

    //Update status bar
    this->statusBar()->showMessage("Building raw point data... " +
        QString::number(i) + " of " + QString::number(length));
    QApplication::processEvents();
  }

  //Abandon all data
  m_raw_oct_poly_data->Reset();

  //Add scalar data and point data
  m_raw_oct_poly_data->SetPoints(points);
  m_raw_oct_poly_data->GetPointData()->SetScalars(dataArray);

//  std::cout << "Loaded with ls: " << length << ", ws: " << width <<
//        ", ds: " << depth << ", lr: " << length_range << ", wr: " <<
//        width_range << ", size: " << m_raw_oct_poly_data->GetNumberOfPoints() <<
//        std::endl;

  //Update status bar
  this->statusBar()->showMessage("Building raw point data... done!");
  QApplication::processEvents();
}





void Form::renderPointPolyData()
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

  //Loop over m_raw_oct_poly_data
  //Use rand to decide if a point should be inserted based on its scalar
  //Insert it into m_vis_poly_data
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
    this->m_ui->status_bar->showMessage("Reading file... ");
    QApplication::processEvents();

    //Create the input_file. C file I/O is the fastest there is, don't use
    //C++ streams unless you have to!
    std::FILE* input_file;
    input_file = std::fopen(file_name.toStdString().c_str(), "rb");

    //If the file isn't open, show an error box
    if(input_file == NULL)
    {
      QMessageBox::critical(this, tr("Error"), tr("Could not open file!"));
      return;
    }

    //Get the file size
    std::fseek(input_file, 0, SEEK_END);
    int file_size = std::ftell(input_file);
    std::rewind(input_file);

    //Read the file into data
    std::vector<uint8_t> data(file_size);
    int bytes_read = std::fread(&(data[0]), 1, file_size, input_file);

    std::fclose(input_file);

    //Check to see if we read everything
    if(bytes_read != file_size)
    {
      QMessageBox::critical(this, tr("Error"),
          tr("Could not read the entire file!"));
      return;
    }

    //Update status bar
    this->m_ui->status_bar->showMessage("Loading data into point cloud... ");
    QApplication::processEvents();

    //Loads the raw oct data vector into m_raw_oct_poly_data
    loadRawOCTData(data);

    //Renders the data
    renderPointPolyData();

    //Displays the file name in the ui
    this->m_ui->file_name_lineedit->setText(file_name);

    //Update status bar
    this->m_ui->status_bar->showMessage("Readying cloud for render... ");
    QApplication::processEvents();
  }
}



void Form::on_connected_master_checkbox_clicked(bool checked)
{
  //Only enable this button if connected to master
  m_ui->request_scan_button->setDisabled(!checked);

  this->m_ui->connected_master_checkbox->setChecked(checked);
  this->m_ui->connected_master_checkbox_2->setChecked(checked);
}





void Form::on_len_steps_spinbox_editingFinished()
{
  m_len_steps = this->m_ui->len_steps_spinbox->value();
}

void Form::on_wid_steps_spinbox_editingFinished()
{
  m_wid_steps = this->m_ui->wid_steps_spinbox->value();
}

void Form::on_dep_steps_spinbox_editingFinished()
{
  m_dep_steps = this->m_ui->dep_steps_spinbox->value();

  //The axial sensor has a fixed resolution, so we change m_dep_range to match
  this->m_ui->dep_range_spinbox->setValue((float)m_dep_steps/1024.0*2.762);
}

void Form::on_len_range_spinbox_editingFinished()
{
  m_len_range = this->m_ui->len_range_spinbox->value();
}

void Form::on_wid_range_spinbox_editingFinished()
{
  m_wid_range = this->m_ui->wid_range_spinbox->value();
}

void Form::on_dep_range_spinbox_editingFinished()
{
  m_dep_range = this->m_ui->dep_range_spinbox->value();

  //The axial sensor has a fixed resolution, so we change m_dep_steps to match
  this->m_ui->dep_steps_spinbox->setValue(m_dep_range/2.762*1024);
}

void Form::on_len_off_spinbox_editingFinished()
{
  m_len_offset = this->m_ui->len_off_spinbox->value();
}

void Form::on_wid_off_spinbox_editingFinished()
{
  m_wid_offset = this->m_ui->wid_off_spinbox->value();
}






void Form::on_request_scan_button_clicked()
{
  //Requests a scan from our qnode
  qDebug() << "Send request";

  //Disables controls until we get a response
  m_ui->len_steps_spinbox->setDisabled(true);
  m_ui->wid_steps_spinbox->setDisabled(true);
  m_ui->dep_steps_spinbox->setDisabled(true);
  m_ui->len_range_spinbox->setDisabled(true);
  m_ui->wid_range_spinbox->setDisabled(true);
  m_ui->dep_range_spinbox->setDisabled(true);
  m_ui->len_off_spinbox->setDisabled(true);
  m_ui->wid_off_spinbox->setDisabled(true);
  m_ui->request_scan_button->setDisabled(true);
  m_ui->save_button->setDisabled(true);

  m_ui->received_data_checkbox->setChecked(false);

  Q_EMIT requestScan(m_len_steps, m_wid_steps, m_dep_steps,
                     m_len_range, m_wid_range, m_dep_range,
                     m_len_offset, m_wid_offset);
}






void Form::on_received_data_checkbox_clicked()
{
  m_ui->received_data_checkbox->setChecked(true);

  //Loads m_qnode's data vector into m_raw_oct_poly_data
  loadRawOCTData(m_qnode->getDataReference(), 0, 0, m_len_steps, m_wid_steps,
      m_dep_steps, m_len_range, m_wid_range);

  //Immediately render it
  renderPointPolyData();

  //Re-enables controls for a potential new scan
  m_ui->len_steps_spinbox->setDisabled(false);
  m_ui->wid_steps_spinbox->setDisabled(false);
  m_ui->dep_steps_spinbox->setDisabled(false);
  m_ui->len_range_spinbox->setDisabled(false);
  m_ui->wid_range_spinbox->setDisabled(false);
  m_ui->dep_range_spinbox->setDisabled(false);
  m_ui->len_off_spinbox->setDisabled(false);
  m_ui->wid_off_spinbox->setDisabled(false);
  m_ui->request_scan_button->setDisabled(false);
  m_ui->save_button->setDisabled(false);
}






void Form::on_save_button_clicked()
{
  if(!m_ui->received_data_checkbox->isChecked())
  {
    QMessageBox::critical(this, tr("Error"), tr("No received data to save!"));
    return;
  }

  QString file_name = QFileDialog::getSaveFileName(this, tr("Save as img file"),
      "", tr("Image file (*.img)"));

  if(!file_name.isEmpty())
  {
    //Allows the file dialog to close before resuming computations
    QApplication::processEvents(QEventLoop::ExcludeSocketNotifiers, 10);

    //Update status bar
    this->m_ui->status_bar->showMessage("Writing data to file... ");
    QApplication::processEvents();

    std::FILE* output_file;
    output_file = std::fopen(file_name.toStdString().c_str(), "wb"); //write,
                                                                     //binary
    //If the file isn't open, show an error box
    if(output_file == NULL)
    {
      QMessageBox::critical(this, tr("Error"), tr("Could not open file!"));
      return;
    }

    std::vector<uint8_t> header;
    header.resize(512);

    memcpy(&header[16], &m_len_steps, 4*sizeof(uint8_t));
    memcpy(&header[20], &m_wid_steps, 4*sizeof(uint8_t));
    memcpy(&header[24], &m_dep_steps, 4*sizeof(uint8_t));
    memcpy(&header[72], &m_len_range, 4*sizeof(uint8_t));
    memcpy(&header[76], &m_wid_range, 4*sizeof(uint8_t));

//    std::cout << "Saved with ls: " << m_len_steps << ", ws: " << m_wid_steps <<
//        ", ds: " << m_dep_steps << ", lr: " << m_len_range << ", wr: " <<
//        m_wid_range << std::endl;

    //Writes header.size() elements of size 1, starting at &(data[0]) to output
    int bytes_written = fwrite(&(header[0]), 1, header.size(), output_file);

    //Write the actual data
    std::vector<uint8_t> data = m_qnode->getDataReference();
    bytes_written += fwrite(&(data[0]), 1, data.size(), output_file);

    fclose(output_file);

    //Check to see if we wrote everything
    if(bytes_written != (header.size() + data.size()))
    {
      QMessageBox::critical(this, tr("Error"),
          tr("Could not write the entire header and data!"));
    }

    //Update status bar
    this->m_ui->status_bar->showMessage("Writing data to file... done!");
    QApplication::processEvents();
  }
}
