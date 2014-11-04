#ifndef FORM_H
#define FORM_H

//C, C++
#include <stdint.h>
#include <assert.h>

//QT
#include <QThread>
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
  void loadRawOCTData(std::vector<uint8_t>& oct_data, int file_header = 512,
      int frame_header = 40, int length = 0, int width = 0, int depth = 0,
      float length_range = 0, float width_range = 0);

  //Renders a poly data containing points as individual vertices. Prunes
  //points based on their scalar values to keep up to MAX_RENDER_POINTS
  void renderPointPolyData();

private Q_SLOTS:
  void on_browse_button_clicked();
  void on_connected_master_checkbox_clicked(bool checked);
  void on_len_steps_spinbox_editingFinished();
  void on_wid_steps_spinbox_editingFinished();
  void on_dep_steps_spinbox_editingFinished();
  void on_len_range_spinbox_editingFinished();
  void on_wid_range_spinbox_editingFinished();
  void on_dep_range_spinbox_editingFinished();
  void on_len_off_spinbox_editingFinished();
  void on_wid_off_spinbox_editingFinished();
  void on_request_scan_button_clicked();

  void on_received_data_checkbox_clicked();

  Q_SIGNALS:
  void requestScan(int length_steps, int width_steps,
                   int depth_steps, float length_range,
                   float width_range, float depth_range,
                   float length_offset, float width_offset);

private:
  Ui::Form *m_ui;
  QNode* m_qnode;
  QThread* m_qthread;

  int m_len_steps;
  int m_wid_steps;
  int m_dep_steps;
  float m_len_range;
  float m_wid_range;
  float m_dep_range;
  float m_len_offset;
  float m_wid_offset;

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
