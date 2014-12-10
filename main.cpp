/*  TODO:
*
* -Split m_waiting_response into actual waiting response and "processing"
*   -We should be able to change a greyscale range while we wait for
*    a node, for example
*
* -Open an extra Qt window
*
* -Look for where they render video with openCV, try to understand it
* 
* -Try looking at peretti3d stuff, see if you can find samples from the 4 imgs
*
* */

/* NEXT EXPERIMENT:
* 
* 
*
* */

/* CHANGES DONE:
* -Needed to install libvtk5.8-qt4 for the lib files of the QVTK widget
*
* -Needed to add the correct export tags on OCT_seg and reg's manifest files
*
* 
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
