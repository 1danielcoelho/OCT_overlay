/*  TODO:
*
* -Have things checked in Overlay tab -> Request new segmentation -> Go back to
*  overlay tab and things are still checked
*
* -Invert x and y coordinates correctly before sending to OCT_registration
*
* */

/* NEXT EXPERIMENT:
* 
* -Get the actual stereocamera node and look at the code for opening a window
*  with live camera feed from left camera
*
* */

/* CHANGES DONE:
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
