/*  TODO:
 *
 *  -Survive between roscores
 *    .Doesn't shut down when app quits
 *    .Doesn't reconnect when new roscore starts
 *    .Doesn't update the checkbox on first boot
 *
 *  -Progress bar widget inside status bar
 *
 *  -Scale the sample based on scanWidth/scanDepth/etc
 *
 *  -Find a cheap way to change the color to white
 *
 *  -Implement OCT_wrapper request
 *
 *
 *
 *
 *
 *
 *
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
