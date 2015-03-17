#ifndef DEFINES_H
#define DEFINES_H

//--------------------------------GENERIC DEFINES-------------------------------
// Macro that facilitates creating vtk objects
// Usage: VTK_NEW(vtkPolyData, mesh);
#define VTK_NEW(type, instance) \
  ;                             \
  vtkSmartPointer<type> instance = vtkSmartPointer<type>::New();

// Allows me to test the program at home only changing this line
//#define AT_HOME

//--------------------------------FILE MANAGER----------------------------------
// Size of the header in bytes that we use for the raw OCT vector
#define OCT_HEADER_BYTES 512
// Folder where the .cache files will end up. Leave empty for project bin folder
#define CACHE_PATH ""
#define OCT_RAW_CACHE_PATH CACHE_PATH "oct_raw.cache"
#define OCT_SURF_CACHE_PATH CACHE_PATH "oct_surf.cache"
#define OCT_MASS_CACHE_PATH CACHE_PATH "oct_mass.cache"
#define STEREO_LEFT_CACHE_PATH CACHE_PATH "stereo_left.cache"
#define STEREO_RIGHT_CACHE_PATH CACHE_PATH "stereo_right.cache"
#define STEREO_DISP_CACHE_PATH CACHE_PATH "stereo_disp.cache"
#define STEREO_DEPTH_CACHE_PATH CACHE_PATH "stereo_depth.cache"
#define VIS_TRANS_CACHE_PATH CACHE_PATH "vis_trans.cache"
//"/opt/imesLS_ROS/laser_interface_new/octCamRegistration.yaml"

//-------------------------------TUMOR SEGMENTATION-----------------------------
//The segmented OCT surface sits at the very top of the surface. This indicates
//how deep below that we should also discard (we discard everything ABOVE the
//surface during the tumour segmentation)
#define SURFACE_THICKNESS 0.2
// Low pass cutoff frequency for when we convert the entire volume to frequency
// domain. DEFAULT: 2, 2, 2
#define FFT_LOWPASS_CUTOFF_X 2
#define FFT_LOWPASS_CUTOFF_Y 2
#define FFT_LOWPASS_CUTOFF_Z 2
// We discard the sides of the volume whenever we take the gradient since the
// algorithm used always produces harsh gradients at edges. DEFAULT: 0.02, 0.02
#define DISCARD_SIDES_PERCENT_X 0.02
#define DISCARD_SIDES_PERCENT_Y 0.02
// Kernel size for the erode and dilate algorithms. DEFAULT: 7,7,7  5,5,5
#define ERODE_KERNEL_X 8
#define ERODE_KERNEL_Y 8
#define ERODE_KERNEL_Z 8
#define DILATE_KERNEL_X 8
#define DILATE_KERNEL_Y 8
#define DILATE_KERNEL_Z 8
// Meshes that have an estimated volume below this value are discarded (air
// bubbles, anomalies, etc). DEFAULT: 0.02
#define MIN_VOLUME 0.02
// Minimum ammount of similarity necessary for classes to be fused together.
// 0.25 seems to be a good default
#define SIMILAR_THRESHOLD 0.15
// The mesh generated is always larger than ideal by a constant factor. We
// calculate the center of mass of the meshes, translate to origin, scale by
// this amount, then translate back. DEFAULT: 0.9
#define FIT_SCALING_X 0.9
#define FIT_SCALING_Y 0.9
#define FIT_SCALING_Z 0.9
// Parallel to the above scaling, we slide every point along its normal by this
// amount. DEFAULT: -0.06
#define WARP_FACTOR_X 0
#define WARP_FACTOR_Y 0
#define WARP_FACTOR_Z 0

//------------------------------------OVERLAY----------------------------------
#define WINDOW_NAME "Stereocamera left image with overlay"
#define CALIB_STEREO_FILE "/opt/imesLS_ROS/cameracalibration/launch/SOMIKON/calib_full_stereo.yaml"
#define CALIB_LEFT_FILE "/opt/imesLS_ROS/cameracalibration/launch/SOMIKON/calib_full_left.yaml"

#endif  // DEFINES_H
