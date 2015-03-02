#ifndef FORM_H
#define FORM_H
#include "defines.h"

// C, C++
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <algorithm>

// QT
#include <QThread>
#include <QMainWindow>
#include <QFileDialog>
#include <QFile>
#include <QDebug>
#include <QElapsedTimer>
#include <QMessageBox>

// PCL
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>

// VTK
// Data structures
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkImageData.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkTypeUInt8Array.h>
#include <vtkDoubleArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkTransform.h>
#include <vtkUnstructuredGrid.h>
#include <vtkTypeFloat32Array.h>
// Filters
#include <vtkContourFilter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkImageReslice.h>
#include <vtkTransformFilter.h>
#include <vtkDelaunay2D.h>
//#include <vtkImageConvolve.h>
#include <vtkImageFFT.h>
#include <vtkImageIdealLowPass.h>
#include <vtkImageRFFT.h>
#include <vtkImageDivergence.h>
#include <vtkImageGradient.h>
#include <vtkImageMarchingCubes.h>
#include <vtkImageExtractComponents.h>
#include <vtkImageMathematics.h>
#include <vtkImageContinuousErode3D.h>
#include <vtkImageContinuousDilate3D.h>
#include <vtkImageGradientMagnitude.h>
#include <vtkImageShiftScale.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkMassProperties.h>
#include <vtkAppendPolyData.h>
#include <vtkImageReslice.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkDelaunay3D.h>
#include <vtkCleanPolyData.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkWarpScalar.h>
#include <vtkPPolyDataNormals.h>
#include <vtkGlyph3D.h>
#include <vtkTransformPolyDataFilter.h>

// Mappers
#include <vtkPolyDataMapper.h>
#include <vtkImageMapper.h>
#include <vtkImageMapToColors.h>
// Actors
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkActor2D.h>
#include <vtkImageActor.h>
// Others
#include <vtkMath.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkTextProperty.h>
#include <vtkGarbageCollector.h>
#include <vtkInteractorStyleImage.h>
#include <vtkArrowSource.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>

// Project files
#include "qnode.h"
#include "crossbar.h"
#include "octinfo.h"
#include "sliceinteractor.h"
#include "clustering.h"

namespace Ui {
class Form;
}

class Form : public QMainWindow {
  Q_OBJECT

 public:
  // Constructor
  explicit Form(int argc, char** argv, QWidget* parent = 0);

  // Destructor
  ~Form();

  // Uses the boolean state variables to figure out which buttons and spinboxes
  // should be enabled and which should be disabled
  void updateUIStates();

  //---------------INPUT--------------------------------------------------------

  // Determines the oct params, including file and frame headers, loads them
  // into
  // m_current_params and finally clears the full_array from file and frame
  // header bytes (it should be a simple array of raw data now)
  void processOCTHeader(std::vector<uint8_t>& full_array);

  // Loads data from a depth-fast, width-medium, length-slow vector octdata
  // Into raw_oct_poly_data
  void loadVectorToPolyData(std::vector<uint8_t>& oct_data);

  //------------PROCESSING------------------------------------------------------

  // 2D, 8-neighbor median filter. Sets edge elements to zero
  void medianFilter2D(std::vector<uint8_t>& input);

  // 3D, 26-neighbor median filter. Sets edge elements to zero
  void medianFilter3D(std::vector<uint8_t>& input);

  // The OCT scanner produces an artifact where the very top of every A scan
  // consists of false, intensity = 255 samples. Here we discard those (set them
  // to 0) so it doesn't trouble the other algorithms
  void discardTop(std::vector<uint8_t>& input, float fraction_to_discard);

  // Finds the maximum, maps it to 255 and linearly maps the rest of the vector
  void normalize(std::vector<uint8_t>& input);

  // Sets the vertical edges (not parallel to an horizontal plane) of an
  // imageData to be zero. frac_i go from 0.0 to 1.0, and it indicate the
  // fraction of the total extent to discard
  // Only works if the inputdata is unsigned char. Will assert
  void discardImageSides(vtkSmartPointer<vtkImageData> input, float frac_x,
                         float frac_y);

  // Averages all point coordinates and puts the center of mass at the passed
  // three-dimensional vector
  void getCenterOfMass(vtkSmartPointer<vtkPolyData> mesh, std::vector<double>&);

  // Uses the passed in vtkPolyData with the surface information, as well as
  // m_oct_poly_data, to extract meshes of the tumor masses. The result is then
  // stored in m_oct_mass_poly_data
  void segmentTumour(vtkSmartPointer<vtkActor> actor,
                     vtkSmartPointer<vtkPolyData> surf);

  //-------------RENDERING------------------------------------------------------

  // Adds an actor with x,y,z axes to m_renderer. Origin, scale, orientation set
  // by a 4x4 matrix
  void renderAxes(vtkSmartPointer<vtkAxesActor> actor,
                  vtkSmartPointer<vtkTransform> trans);

  // Adds m_oct_vol_actor, containing points as individual vertices, to
  // m_renderer. Prunes points based on their scalar values, according to
  // m_vis_threshold
  void renderOCTVolumePolyData();


  void renderOCTSurface();

  void renderDepthImage();

  // Clears m_renderer from actors and renders either the left, right or
  // displacement map vtkImageData objects from by load2DVectorCacheToImageData
  void render2DImageData(vtkSmartPointer<vtkImageData> image_data);

  // Renders the mesh stored in m_oct_mass_poly_data as a triangular mesh
  void renderOCTMass();

  //--------------UI CALLBACKS--------------------------------------------------

 private
Q_SLOTS:
  void on_browse_button_clicked();
  void on_connected_master_checkbox_clicked(bool checked);

  void on_dep_steps_spinbox_editingFinished();
  void on_dep_range_spinbox_editingFinished();

  void on_request_scan_button_clicked();
  void on_save_button_clicked();
  void on_reset_params_button_clicked();

  void on_view_raw_oct_button_clicked();
  void on_calc_oct_surf_button_clicked();
  void on_view_oct_surf_button_clicked();
  void on_calc_oct_mass_button_clicked();

  void on_view_left_image_button_clicked();
  void on_view_right_image_button_clicked();
  void on_view_disp_image_button_clicked();
  void on_view_depth_image_button_clicked();

  void on_calc_transform_button_clicked();
  void on_print_transform_button_clicked();

  void on_raw_min_vis_spinbox_editingFinished();
  void on_raw_max_vis_spinbox_editingFinished();
  void on_raw_min_vis_slider_valueChanged(int value);
  void on_raw_max_vis_slider_valueChanged(int value);
  void on_raw_min_vis_slider_sliderReleased();
  void on_raw_max_vis_slider_sliderReleased();

  void on_over_min_vis_spinbox_editingFinished();
  void on_over_max_vis_spinbox_editingFinished();
  void on_over_min_vis_slider_valueChanged(int value);
  void on_over_max_vis_slider_valueChanged(int value);
  void on_over_min_vis_slider_sliderReleased();
  void on_over_max_vis_slider_sliderReleased();

  void on_over_raw_checkbox_clicked();
  void on_over_oct_surf_checkbox_clicked();
  void on_over_oct_mass_checkbox_clicked();
  void on_over_depth_checkbox_clicked();
  void on_over_oct_axes_checkbox_clicked();
  void on_over_trans_axes_checkbox_clicked();

  //------------QNODE CALLBACKS-------------------------------------------------

  void receivedRawOCTData(OCTinfo params);
  void receivedOCTSurfData(OCTinfo params);
  void receivedStereoImages();
  void receivedRegistration();
  void accumulated(float new_ratio);

  void on_browse_oct_surf_button_clicked();

  void on_save_oct_surf_button_clicked();

  void on_browse_oct_mass_button_clicked();

  void on_save_oct_mass_button_clicked();

  void on_view_oct_mass_button_clicked();

  void on_request_left_image_button_clicked();

  void on_request_right_image_button_clicked();

  void on_request_disp_image_button_clicked();

  void on_request_depth_image_button_clicked();

  void on_browse_left_image_button_clicked();

  void on_browse_right_image_button_clicked();

  void on_browse_disp_image_button_clicked();

  void on_browse_depth_image_button_clicked();

  void on_save_left_image_button_clicked();

  void on_save_right_image_button_clicked();

  void on_save_disp_image_button_clicked();

  void on_save_depth_image_button_clicked();

  void on_accu_reset_button_clicked();

  void on_accu_spinbox_editingFinished();

  Q_SIGNALS:
  void requestScan(OCTinfo);
  void requestSegmentation(OCTinfo);
  void requestRegistration();
  void setAccumulatorSize(unsigned int);
  void resetAccumulators();

 private:
  Ui::Form* m_ui;
  QNode* m_qnode;
  QThread* m_qthread;
  Crossbar* m_crossbar;

  // Controls minimum displayed intensity value when viewing raw OCT
  uint8_t m_min_vis_thresh;
  uint8_t m_max_vis_thresh;

  // State booleans
  bool m_connected_to_master;
  bool m_has_ros_raw_oct;
  bool m_has_raw_oct;
  bool m_waiting_response;
  bool m_has_oct_surf;
  bool m_has_oct_mass;
  bool m_has_stereo_cache;
  bool m_has_left_image;
  bool m_has_right_image;
  bool m_has_disp_image;
  bool m_has_depth_image;
  bool m_has_transform;
  bool m_viewing_overlay;

  // Holds our current raw oct parameters (steps, ranges, offsets)
  OCTinfo m_current_params;

  // VTK objects
  vtkSmartPointer<vtkPolyData> m_oct_poly_data;
  vtkSmartPointer<vtkPolyData> m_oct_mass_poly_data;
  vtkSmartPointer<vtkPolyData> m_oct_surf_poly_data;
  vtkSmartPointer<vtkImageData> m_stereo_left_image;
  vtkSmartPointer<vtkImageData> m_stereo_right_image;
  vtkSmartPointer<vtkImageData> m_stereo_disp_image;
  vtkSmartPointer<vtkPolyData> m_stereo_depth_image;
  vtkSmartPointer<vtkTransform> m_oct_stereo_trans;
  // Actors are kept since we need their references when we add/remove actors in
  // the Overlay section of the program
  vtkSmartPointer<vtkActor> m_oct_vol_actor;
  vtkSmartPointer<vtkActor> m_oct_surf_actor;
  vtkSmartPointer<vtkActor> m_oct_mass_actor;
  vtkSmartPointer<vtkActor> m_stereo_depth_actor;
  vtkSmartPointer<vtkActor2D> m_stereo_2d_actor;
  vtkSmartPointer<vtkAxesActor> m_oct_axes_actor;
  vtkSmartPointer<vtkAxesActor> m_trans_axes_actor;
  // Others
  vtkSmartPointer<vtkRenderer> m_renderer;
};

#endif  // FORM_H
