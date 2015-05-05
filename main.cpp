/*  TODO LIST:
 *
 *  -Something strange is happening with the alpha levels on the reconstruction
 *   surface. It seems to always reset to zero, for some reason. This causes
 *   the 2D mode to also map to zero
 *
 *  -Have it remove and add all the proper actors between 2d and 3d mode: Some
 *   still show up below the 2D image, like the oct surface
 *
 *  -Change m_oct_mass_poly_data_stereo3D to something less stupid
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
 *  -Kill the z=-1 points in stereo reconstruction. Watch out: Simply discarding
 *   the points will make the StereoProj encoding fall out of sync (it uses
 *   rows and cols to determine point index)
 *
 *  -Now that the z=-1 points are gone, using ResetCamera might produce better
 *   results
 *
 *  -Maybe find a way to implement opacity encoding
 *
 *  -Have P be read from the file as opposed to being hard-coded
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
