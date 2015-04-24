/*  TODO LIST:
 *
 *  -Implement the silhouette visualization from the stereocamera POV
 *    -Need to export the origin/spacing/extent from the left camera image AFTER
 *     it has been rendered (render2DImageData changes them to fit window size)
 *    -Need to find a way to clean up the silhouette: It has inner lines that
 *     don't handle being stencils super well
 *    -Find a way to use the colored reconstruction/left image as input to the
 *     stencil filter
 *
 *  -Implement the silhouette visualization from the OCT POV
 *    -Apply the inverse OCT->stereo transform; Discard Z coordinate; Find all
 *     edges in the mass poly that belong to only one triangle (outer edges);
 *
 *  -Change m_oct_mass_poly_data_stereo3D to something less stupid
 *
 *  -Maybe find a way to implement opacity encoding
 *
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
