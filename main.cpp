/*  TODO:
* 
* -6 neighbor median filter
*   -Do it in form, filter, then write over the cache
*
* -remove 255 top 
*
* -Implement surface reconstruction for OCT segmentation output and stereocam
*  depth map
*    -Delaunay2D, Shepard's method
*
* 
*
* */

/* NEXT EXPERIMENT:
* 
* -Test the roslaunch without stereomatching. Check if nodes and topics are
*  ok with rosnode and rostopic
*   -roslaunch OCT_overlay OCT_overlay_without_stereocamera.launch
*
* -Turn on the windows PC
*
* -Turn on the OCT
*
* -Setup the OCT pedestal for a sensible image using the Thorlabs software
*
* -Turn on the OCT server on the windows PC
*
* -Test a simple OCT request using the OCT_overlay window
*
* -Save to file and open it again to make sure it works

* -Request another OCT image and make sure the surface segmentation is ok
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
