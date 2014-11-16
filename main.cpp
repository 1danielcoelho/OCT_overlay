/*  TODO:
 *
 *  -Fix depth_range not being set on Thorlabs img files
 *
 *  -Convert the stereocamera pcl clouds into VTK stuff and render them
 *
 *  -Actually use/delete m_vis_poly_data
 *
 *  -Flip and mirror the CV matrices, BGR, because somebody thought it was a
 *   great idea to do everything backwards
 *
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
