/*  TODO:
 *
 *  -Test BGR and column/row major alignment with an actual image
 *
 *  -Setup launch files to startup oct_client_tcp, OCT_seg, and friends
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
