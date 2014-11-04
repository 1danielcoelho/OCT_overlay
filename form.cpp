#include "form.h"
#include "ui_form.h"





Form::Form(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  m_ui(new Ui::Form),
  m_qnode(argc, argv)
{
  m_ui->setupUi(this);

  QObject::connect(this, SIGNAL(shutdownROS()), &m_qnode, SLOT(terminateThread()));

  //Keeps the checkbox updated with the status of the master
  QObject::connect(&m_qnode, SIGNAL(rosMasterChanged(bool)),
   this, SLOT(on_connected_master_checkbox_clicked(bool)));

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

  //Adds our renderer to the QVTK widget
  this->m_ui->qvtkWidget->GetRenderWindow()->AddRenderer(m_renderer);
}





Form::~Form()
{
  delete m_ui;
  Q_EMIT shutdownROS();
}





void Form::loadRawOCTData(std::vector<uint8_t>& oct_data)
{
  uint32_t raw_length, raw_width, raw_depth, file_header, frame_header;
  float raw_length_mm, raw_width_mm;

  file_header = 512;
  frame_header = 40;

  //Extract the data dimensions from the header
  std::memcpy(&raw_length, &(oct_data[16]), 4*sizeof(uint8_t));
  std::memcpy(&raw_width, &(oct_data[20]), 4*sizeof(uint8_t));
  std::memcpy(&raw_depth, &(oct_data[24]), 4*sizeof(uint8_t));
  std::memcpy(&raw_length_mm, &(oct_data[72]), 4*sizeof(uint8_t));
  std::memcpy(&raw_width_mm, &(oct_data[76]), 4*sizeof(uint8_t));

  //Determines the spatial increments for each dimension
  float length_incrm = raw_length_mm/raw_length;
  float width_incrm = raw_width_mm/raw_width;
  float depth_incrm = 2.762/1024; //Fixed axial resolution

  //Creates an array for point coordinates, and one for the scalars
  VTK_NEW(vtkPoints, points);
  VTK_NEW(vtkTypeUInt8Array, dataArray);

  points->SetNumberOfPoints(raw_length * raw_width * raw_depth);
  dataArray->SetNumberOfValues(raw_length * raw_width * raw_depth);

  //Update status bar
  this->statusBar()->showMessage("Building raw point data... ");
  QApplication::processEvents();

  for(unsigned int i = 0; i < raw_length; i++)
  {
    for(unsigned int j = 0; j < raw_width; j++)
    {
      for(unsigned int k = 0; k < raw_depth; k++)
      {
        //Thorlabs img files use 40 bytes of NULL between each B-scan
        int val = oct_data[i*raw_width*raw_depth + j*raw_depth +
                           k + frame_header*i + file_header];

        points->InsertNextPoint(i*length_incrm, j*width_incrm, k*depth_incrm);
        dataArray->InsertNextValue(val);
      }
    }

    //Update status bar
    this->statusBar()->showMessage("Building raw point data... " +
        QString::number(i) + " of " + QString::number(raw_length));
    QApplication::processEvents();
  }

  //Abandon all data
  m_raw_oct_poly_data->Reset();

  //Add scalar data and point data
  m_raw_oct_poly_data->SetPoints(points);
  m_raw_oct_poly_data->GetPointData()->SetScalars(dataArray);

  //Update status bar
  this->statusBar()->showMessage("Building raw point data... done!");
  QApplication::processEvents();
}





void Form::renderPointPolyData()
{
  //Get the number of points
  uint32_t num_pts = m_raw_oct_poly_data->GetPointData()->GetNumberOfTuples();

  //float mask_ratio = 1;

  if(num_pts == 0)
  {
    qDebug() << "m_raw_oct_poly_data is empty!";
    return;
  }
//  else if (num_pts > MAX_RENDER_POINTS)
//  {
//    //Calculate the integer mask ratio to get approx. up to MAX_RENDER_POINTS
//    //vertices
//    mask_ratio = (uint32_t)(totalPoints/MAX_RENDER_POINTS + 0.5);
//  }

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
    if(value > 100)
    {
      new_points->InsertNextPoint(old_points->GetPoint(i));
      new_data_array->InsertNextValue(value);
    }
  }

  //Update status bar
  this->statusBar()->showMessage("Preparing PolyData for display... done!");
  QApplication::processEvents();

  m_vis_poly_data->SetPoints(new_points);
  m_vis_poly_data->GetPointData()->SetScalars(new_data_array);
  //m_vis_poly_data->Print(std::cout);

  m_vert_filter->SetInput(m_vis_poly_data);

  m_poly_mapper->SetInputConnection(m_vert_filter->GetOutputPort());
  m_poly_mapper->SetScalarVisibility(1);

  m_actor->SetMapper(m_poly_mapper);

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
    this->m_ui->status_bar->showMessage("Analyzing file... ");
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
  this->m_ui->connected_master_checkbox->setChecked(checked);
}
