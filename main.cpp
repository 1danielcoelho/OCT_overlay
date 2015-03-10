/*  TODO:

-Cleanup Form.h, Form.cpp, qnode.h, qnode.cpp, crossbar, etc. Re-order functions and so on

-Use QNode for the actual overlay, ironically
   -User clicks a "Begin overlaying in real time" button
   -Create an OpenCV window named WINDOW_NAME
   -Send a signal from Form to Qnode that it can show images on window called WINDOW_NAME
   -QNode changes it's update frequency and goes into a function like "renderOverlay"
       -Load the OCT visualization from a file one, enter an infinite loop
          -Read the left and depth image from the subscriptions
          -Determine the placement on the left image for the OCT visualization somehow
           either by reconstructing the entire stereocamera surface and displaying that,
           or by just somehow keeping track of the 3d position of every pixel in the left image
          -Add the OCT visualization to the image
          -Use cv::imgshow(WINDOW_NAME, overlayed_image); to display it
          -Check for events. If Form has sent a terminate signal, change back update frequency and
           resume normal operation. Form then kills the window
* */

/* NEXT EXPERIMENT:
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
