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
