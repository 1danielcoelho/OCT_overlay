#ifndef FORM_H
#define FORM_H

//C, C++
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <algorithm>

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
#include <vtkImageData.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkTypeUInt8Array.h>
#include <vtkUnsignedCharArray.h>
#include <vtkTransform.h>
//Filters
#include <vtkVertexGlyphFilter.h>
#include <vtkImageReslice.h>
#include <vtkTransformFilter.h>
#include <vtkDelaunay2D.h>
//Mappers
#include <vtkPolyDataMapper.h>
#include <vtkImageMapper.h>
//Actors
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkActor2D.h>
//Others
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkTextProperty.h>
//Project files
#include "qnode.h"
#include "filemanager.h"
#include "octinfo.h"

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

  //Uses the boolean state variables to figure out which buttons and spinboxes
  //should be enabled and which should be disabled
  void updateUIStates();

  //---------------INPUT--------------------------------------------------------

  //Determines the oct params, including file and frame headers, loads them into
  //m_current_params and finally clears the full_array from file and frame
  //header bytes (it should be a simple array of raw data now)
  void processOCTHeader(std::vector<uint8_t>& full_array);

  //Loads data from a depth-fast, width-medium, length-slow vector octdata
  //Into raw_oct_poly_data
  void loadVectorToPolyData(std::vector<uint8_t>& oct_data);

  //Loads a PCL point cloud saved as a binary file at file_path and builds a
  //poly_data for visualization
  void loadPCLCacheToPolyData(const char* file_path,
                              vtkSmartPointer<vtkPolyData> depth_image);

  //Loads either the left, right or displacement map images produced by the
  //stereocamera into a vtkImageData object
  void load2DVectorCacheToImageData(const char* file_path,
                                    vtkSmartPointer<vtkImageData> image_data);

  //------------PROCESSING------------------------------------------------------

  //The OCT scanner produces an artifact where the very top of every A scan
  //consists of false, intensity = 255 samples. Here we discard those (set them
  //to 0) so it doesn't trouble the other algorithms
  void discardTop(std::vector<uint8_t>& input, float fraction_to_discard);

  //2D, 8-neighbor median filter. Sets edge elements to zero
  void medianFilter2D(std::vector<uint8_t>& input);

  //3D, 26-neighbor median filter. Sets edge elements to zero
  void medianFilter3D(std::vector<uint8_t>& input);

  //Finds the maximum, maps it to 255 and linearly maps the rest of the vector
  void normalize(std::vector<uint8_t>& input);

  //-------------RENDERING------------------------------------------------------

  //Adds an actor with x,y,z axes to m_renderer
  void renderAxes();

  //Adds m_oct_vol_actor, containing points as individual vertices, to
  //m_renderer. Prunes points based on their scalar values, according to
  //m_vis_threshold
  void renderOCTVolumePolyData();

  //Adds m_oct_surf_actor, a 1-sample-thick PolyData actor, to m_renderer
  void renderPolyDataSurface(vtkSmartPointer<vtkPolyData> depth_image);

  //Clears m_renderer from actors and renders either the left, right or
  //displacement map vtkImageData objects from by load2DVectorCacheToImageData
  void render2DImageData(vtkSmartPointer<vtkImageData> image_data);

  //Renders the oct_surface and depth_map simultaneously
  void renderOverlay(vtkSmartPointer<vtkPolyData> oct_surface,
                     vtkSmartPointer<vtkPolyData> depth_map);

  //--------------UI CALLBACKS--------------------------------------------------

private Q_SLOTS:
  void on_browse_button_clicked();
  void on_connected_master_checkbox_clicked(bool checked);
  void on_dep_steps_spinbox_editingFinished();
  void on_dep_range_spinbox_editingFinished();
  void on_request_scan_button_clicked();
  void on_save_button_clicked();
  void on_view_raw_oct_button_clicked();  
  void on_calc_oct_surf_button_clicked();
  void on_view_left_image_button_clicked();
  void on_view_right_image_button_clicked();
  void on_view_disp_image_button_clicked();
  void on_view_depth_image_button_clicked();
  void on_reset_params_button_clicked();
  void on_calc_transform_button_clicked();
  void on_print_transform_button_clicked();
  void on_view_oct_surf_button_clicked();
  void on_view_simple_overlay_button_clicked();
  void on_view_oct_vol_oct_surf_clicked();
  void on_raw_min_vis_spinbox_editingFinished();
  void on_raw_max_vis_spinbox_editingFinished();
  void on_raw_min_vis_slider_valueChanged(int value);
  void on_raw_max_vis_slider_valueChanged(int value);
  void on_raw_min_vis_slider_sliderReleased();
  void on_raw_max_vis_slider_sliderReleased();
  void on_over_min_vis_slider_valueChanged(int value);
  void on_over_max_vis_slider_valueChanged(int value);
  void on_over_min_vis_slider_sliderReleased();
  void on_over_max_vis_slider_sliderReleased();
  void on_over_min_vis_spinbox_editingFinished();
  void on_over_max_vis_spinbox_editingFinished();

  //------------QNODE CALLBACKS-------------------------------------------------

  void receivedRawOCTData(OCTinfo params);
  void receivedOCTSurfData(OCTinfo params);
  void receivedStereoData();
  void receivedRegistration();

Q_SIGNALS:
  void requestScan(OCTinfo);
  void requestSegmentation(OCTinfo);
  void requestRegistration();

private:
  Ui::Form *m_ui;
  QNode* m_qnode;
  QThread* m_qthread;
  FileManager* m_file_manager;

  //Controls minimum displayed intensity value when viewing raw OCT
  uint8_t m_min_vis_thresh;
  uint8_t m_max_vis_thresh;

  //State booleans
  bool m_connected_to_master;
  bool m_has_ros_raw_oct;
  bool m_has_raw_oct;
  bool m_waiting_response;
  bool m_has_oct_surf;
  bool m_has_oct_mass;
  bool m_has_stereo_data;
  bool m_has_transform;

  //Holds our current raw oct parameters (steps, ranges, offsets)
  OCTinfo m_current_params;

  //VTK objects
  //Data structures
  vtkSmartPointer<vtkPolyData> m_oct_poly_data;
  vtkSmartPointer<vtkTransform> m_oct_stereo_trans;
  //Actors
  vtkSmartPointer<vtkActor> m_oct_vol_actor;
  vtkSmartPointer<vtkActor> m_oct_surf_actor;
  vtkSmartPointer<vtkActor2D> m_stereo_2d_actor;
  //Others
  vtkSmartPointer<vtkRenderer> m_renderer;

};

#endif // FORM_H
