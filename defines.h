#ifndef DEFINES_H
#define DEFINES_H

#include <vtkSmartPointer.h>

//Macro that facilitates creating vtk objects
#define VTK_NEW(type, instance) \
  ;                             \
  vtkSmartPointer<type> instance = vtkSmartPointer<type>::New();

// Allows me to test the program at home only changing this line
#define AT_HOME


//File manager parameters
//Size of the header in bytes that we use for the raw OCT vector
#define OCT_HEADER_BYTES 512
//Folder where the .cache files will end up. Leave empty for project bin folder
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







//segmentTumour
#define FFT_LOWPASS_CUTOFF_X 2
#define FFT_LOWPASS_CUTOFF_Y 2
#define FFT_LOWPASS_CUTOFF_Z 2

#define DISCARD_SIDES_PERCENT_X 0.02
#define DISCARD_SIDES_PERCENT_Y 0.02

#define ERODE_KERNEL_X 7
#define ERODE_KERNEL_Y 7
#define ERODE_KERNEL_Z 1

#define DILATE_KERNEL_X 5
#define DILATE_KERNEL_Y 5
#define DILATE_KERNEL_Z 2

#define CUBES_VALUE 30

#define MIN_VOLUME 0.02

#define FIT_SCALING_X 0.9
#define FIT_SCALING_Y 0.9
#define FIT_SCALING_Z 0.9

#define WARP_FACTOR_X -0.06
#define WARP_FACTOR_Y -0.06
#define WARP_FACTOR_Z -0.06

#endif //DEFINES_H
