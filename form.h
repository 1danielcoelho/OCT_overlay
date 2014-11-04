#ifndef FORM_H
#define FORM_H

//C, C++
#include <stdint.h>
#include <assert.h>

//QT
#include <QMainWindow>
#include <QFileDialog>
#include <QFile>
#include <QDebug>
#include <QElapsedTimer>
#include <QMessageBox>

//PCL
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>

//VTK
//Data structures
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkTypeUInt8Array.h>
//Filters
#include <vtkVertexGlyphFilter.h>
//Mappers
#include <vtkPolyDataMapper.h>
//Actors
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
//Others
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkTextProperty.h>


//Project files
#include "qnode.h"

//Maximum number of points displayed on the QVTK widget
#define MAX_RENDER_POINTS 2000000
#define VTK_NEW(type, instance); vtkSmartPointer<type> instance = \
                                 vtkSmartPointer<type>::New();

namespace Ui {
  class Form;
}

class Form : public QMainWindow
{
  Q_OBJECT

  public:

  //Constructor
  explicit Form(int argc, char** argv, QWidget *parent = 0);
  //Destructor
  ~Form();
  //Loads data from a depth-fast, width-medium, length-slow vector octdata
  //Into raw_oct_poly_data, using frame and file headers for parsing
  void loadRawOCTData(std::vector<uint8_t>& oct_data);

  //Renders a poly data containing points as individual vertices. Prunes
  //points based on their scalar values to keep up to MAX_RENDER_POINTS
  void renderPointPolyData();

  private Q_SLOTS:
  void on_browse_button_clicked();
  void on_connected_master_checkbox_clicked(bool checked);

  Q_SIGNALS:
  void shutdownROS();

  private:
  Ui::Form *m_ui;
  QNode m_qnode;

  //VTK objects
  //Data structures
  vtkSmartPointer<vtkPolyData> m_raw_oct_poly_data;
  vtkSmartPointer<vtkPolyData> m_vis_poly_data;
  //Filters
  vtkSmartPointer<vtkVertexGlyphFilter> m_vert_filter;
  //Mappers
  vtkSmartPointer<vtkPolyDataMapper> m_poly_mapper;
  //Actors
  vtkSmartPointer<vtkActor> m_actor;
  vtkSmartPointer<vtkAxesActor> m_axes_actor;
  //Others
  vtkSmartPointer<vtkRenderer> m_renderer;

};

#endif // FORM_H
