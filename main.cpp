/*  TODO LIST:
 *
 *  -OCT proj: Transform stereo surf to OCT KS -> Discard it's and the mass' z
 *   coordinates -> Mass is made into a set of polygons. Enveloped by circles.
 *   (beforehand) -> For every point of stereo reconstr, check the distance to
 *   the center of the circle. If it's less than the radius, perform
 *   vtkPolygon::PointInPolygon(). If it is in the polygon, perform normal color
 *   encoding directly on the stereo surf -> The rest (including 2d mapping) is
 *   performed just like for stereo proj
 *    -Discard Z coordinate -> Delaunay2D (already aligned to XY, so great) ->
 *     Select all edges that belong to a single triangle only (inner edges will
 *     belong to two triangles) -> Craft polygon
 *    -NEXT STEPS: Run the constructOCTPOVPolygons through a
 *     vtkPolyDataConnectivityFilter to isolate independent meshes, then try
 *     using PointInPolygon
 *
 *  -Fix weird bug that happened during presentation. I think it had something
 *   to do with Starting in 2D mode as opposed to 3D, but I couldn't get it to
 *   happen again. At least fix that to make sure everything is properly
 *   initialized
 *
 *  -Stop streaming while in 2D and add ators -> 2D actors still remain
 *
 *  -Click to view something outside of overlay -> Things in overlay remain
 *   ticked; Silhouette overlays need to be reselected whenever the mass changes
 *    -Implement some sort of "clearOverlayTab" function, that resets everything
 *     to default
 *
 *  -Implement the silhouette visualization from the OCT POV
 *    -Apply the inverse OCT->stereo transform; Discard Z coordinate; Find all
 *     edges in the mass poly that belong to only one triangle (outer edges);
 *
 *  -Maybe find a way to implement opacity encoding
 *
 *  -Fix all instances of 640x480
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
 *  resolution
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
* -The opacity bug is a consequence of not reordering the translucent geometry
*  every frame (which is expensive). Depth Peeling (which does just that) can
*  be enabled in the constructor of Form, but will will halve FPS at best.
*  Volume scans with depth peeling on are virtually unmanageable
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
