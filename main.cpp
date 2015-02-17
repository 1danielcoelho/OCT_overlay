/*  TODO:
*
*	-over_oct_mass_checkbox_clicked possibly re-calculates the surface when that
*  probably wasnt really necessary
*
*	-IDEA TO SEGMENT TUMOR: Average filter with a humongous kernel size, then
	 create contours based on the voxel value of the average voxels. The tumor
	 should have a significantly lower average value; resistance to noise an
	 inhomogeneities due to a large kernel; Automatically gives "trust ranges"
	 around the tumor; Automatically sets a lower bound on the tumor in case its
	 hard to see where the bottom ends
*
* -Find the correct way of intializing/zeroing the accs in resetAccumulators
*
* -Changing accu size should make the view buttons blank out
*
* -Fix handling the displacement map image
*  -use cv::normalize
*
* -Use CV_64FC3 for the depth map accumulator, then cast it to 32FC3 for writing
*
* -left accu has cast itself to another format so it can be used in the depth
*  map write. That potentially doesn't work
*
* -Try using delaunay with the stereo depth map and see if it colors it
*  automagically
*
* -See how the inversion of x and y impacts the OCT registration
*
* -Consider adding timers to the showmessage statements*
* */

/* NEXT EXPERIMENT:
* 
*
* */

/* NOTES:
* -Needed to install libvtk5.8-qt4 for the lib files of the QVTK widget
*
* -Needed to add the correct export tags on OCT_seg and reg's manifest files
*
* -LED light circle to 9 and 12 volts, power supply disconnected
*
* -stereomathing_SOMIKON.launch -> visualizationOn to true, should also publish
*
* -Change brightness/contrast with dynamic reconfigure
*
* -Use Smaract_Jog_Mode/Jog_Mode/Jog_Mode.pro -> run executable -> Connect ->
*  -> Enc. nullen -> 34 mm -> GO !
*
*/

#include <QtGui>
#include <QApplication>

#include "form.h"
#include "qnode.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  Form w(argc,argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
