/*  TODO:
* 
* -Delete conversion functions in form.ccp that went to the crossbar. Like the "load..." functions
*
* -Cleanup Form.h, Form.cpp, qnode.h, qnode.cpp, crossbar, etc. Re-order functions and so on
*
* -Ensure that if depth map uses left image, then guarantee both use same resolution
*
* -Some functions dont need parameters anymore, like the rendering functions
*
* -Bring the Raw OCT inputs to the same style as the others: Bring the browse button
*  down, maybe the request and save buttons too. Allow saving an image file as another?
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
* -Test and setup registration
*
* */

/*  POSSIBLE BUGS / THINGS TO CHECK:
 *
 * -Generated OCT surface does not have a point corresponding to every x,y
 *  coordinate of the volume
 *
 * -Registration transform not written/read from the correct location
 *
 *
 *
 *
 *
 * */

/*NOTES:
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
* */

#include <QtGui>
#include <QApplication>

#include "form.h"
#include "qnode.h"

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  Form w(argc, argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
