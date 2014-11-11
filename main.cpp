/*  TODO:
 *
 *  -Maybe implement rendering the sample based on the offset as opposed to just
 *   passing it along
 *
 *  -Change editing finished slots to value changed slots
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
