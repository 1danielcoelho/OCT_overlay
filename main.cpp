/*  TODO LIST:
 *
 *  -Change m_oct_mass_poly_data_stereo3D to something less stupid
 *
 *  -Use ResetCamera to place the surfaces in front of the camera when rendered
 *
 *  -Decide wether to ignore the opacity bug or not. This is a consequence of
 *   not reordering the translucent geometry every frame (which is expensive)
 *
 *  -Fix weird bug that happened during presentation. I think it had something
 *   to do with Starting in 2D mode as opposed to 3D, but I couldn't get it to
 *   happen again. At least fix that to make sure everything is properly
 *   initialized
 *
 *  -Implement the silhouette visualization from the OCT POV
 *    -Apply the inverse OCT->stereo transform; Discard Z coordinate; Find all
 *     edges in the mass poly that belong to only one triangle (outer edges);
 *
 *  -Maybe find a way to implement opacity encoding
 *
 *  -Change color of all masses to full red
 *
 *  -Handle resetting camera position
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
