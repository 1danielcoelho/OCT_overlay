/*  TODO:
*
* -Implement surface reconstruction for OCT segmentation output and stereocam
*  depth map
*    -Delaunay2D, Shepard's method
* 
*
* */

/* NEXT EXPERIMENT:
* 
* -Fix OCT_segmentation
*   -Different region sizes, look at member functions of defaultSomething,
*    print sizes
*   -Now that we can load using qtcreator, look at the member functions to help
*    print these bounds
* 
* -Try rendering the OCT surface on top of OCT data, see if they coincide
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
