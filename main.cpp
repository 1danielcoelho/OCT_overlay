/*  TODO:
 *
 *
 *
 * */

Change OCT_overlay to work with the other ROS packages (uncomment things, remove
tests, etc)

 /* NEXT EXPERIMENT:
  *
  * -Get this file (Hello!)
  *

  *
  * -Check SVN state of oct_client, OCT_segmentation, OCT_registration and
  *  commit the potential uncommited changes
  *   -svn status       Check which branch im in. Should be trunk, no problems
  *   -svn switch https://svn.imes.uni-hannover.de/svn/Projekte/OctLaser/trunk/imesLS_ROS/OCT_segmentation
  *   -svn commit -m "Commits work that was left uncommited"
  *
  * -Rebuild OCT_segmentation to make sure it's working before my changes
  *   -rosmake OCT_segmentation
  *   -roscore
  *   -rosrun OCT_segmentation
  *
  * -Apply my changes to OCT_segmentation according to my text file
  *
  * -Build OCT_segmentation and repair potential problems
  *   -rosmake OCT_segmentation
  *   -fix problems
  *   -svn commit -m "Implements a new service to segment a passed array of oct
  *                   data"
  *
  * -Create a folder for my package
  *   -mkdir /opt/imesLS_ROS/OCT_overlay
  *
  * -Clone my repository into the package folder
  *   -cd /opt/imesLS_ROS/OCT_overlay
  *   -git clone https://github.com/1danielcoelho/OCT_overlay.git
  *
  * -Configure OCT_overlay makefile (especially the VTK part; Make sure to leave
  *  the other part commented so it's still compatible with my laptop)
  *
  * -Try and build OCT_overlay and repair potential problems
  *   -rosmake OCT_overlay
  *
  * -Try loading OCT scan .img files from my dropbox
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
  *
  * -Request another OCT image and make sure the surface segmentation is ok
  *
  * -Try setting up a launchfile with peretti3D just to test the stereocamera
  *  images. I'm pretty sure the left/right images are inverted/BGR'd
  *
  * -Work complete for now, unless the stereocamera is setup
  *
  * */


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
