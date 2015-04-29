/*  TODO LIST:
 *
 *  -Change m_oct_mass_poly_data_stereo3D to something less stupid
 *
 *  -QNode is sending out depth_image with double scalars, but previously we
 *   converted it to uchar. We have to make it so that the depth image gets
 *   transmitted either as double or float, and properly handled on Form. This
 *   also includes making it so opening and saving an image converts from vector
 *   to double/float, as well as making sure that render2DImageData can cast
 *   down from double/float to a sensible unsigned char
 *
 *  -Fix weird bug that happened during presentation. I think it had something
 *   to do with Starting in 2D mode as opposed to 3D, but I couldn't get it to
 *   happen again. At least fix that to make sure everything is properly
 *   initialized
 *
 *  -Make sure that ticking stereo reconstruction in overlay and starting/
 *   stopping do similar things in similar ways
 *
 *  -Implement the silhouette visualization from the OCT POV
 *    -Apply the inverse OCT->stereo transform; Discard Z coordinate; Find all
 *     edges in the mass poly that belong to only one triangle (outer edges);
 *

 *
 *  -Maybe find a way to implement opacity encoding
 *
 *  -Have P be read from the file as opposed to being hard-coded
 *
 *
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
 * -Ensure that if depth map uses left image, then guarantee both use same
 *resolution
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
