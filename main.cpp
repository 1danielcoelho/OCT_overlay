/*  TODO:
*
* -Fix handling the displacement map image
*  -Somehow save an example so we can look at it later
*  -Ask Andreas for the image format
*
* -Try using delaunay with the stereo depth map and see if it colors it
*  automagically
*
* -See how the inversion of x and y impacts the OCT registration
*
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
