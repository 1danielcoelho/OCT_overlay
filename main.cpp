/*  TODO:
* 
* -The view functions should setup pipelines that can be updated by checking a checkbox
*  which changes a transform from identity to the registration transform, making it easy
*  to view its effects with a single click
*
* -Cleanup Form.h, Form.cpp, qnode.h, qnode.cpp, crossbar, etc. Re-order functions and so on
*
* -Bring the Raw OCT inputs to the same style as the others: Bring the browse button
*  down, maybe the request and save buttons too. Allow saving an image file as another?
*
* -Find the correct way of intializing/zeroing the accs in resetAccumulators
*
* -See how the inversion of x and y impacts the OCT registration
*
* */

/* NEXT EXPERIMENT:
* -Test stereocamera images, saving, loading
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
 * -Ensure that if depth map uses left image, then guarantee both use same resolution
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
