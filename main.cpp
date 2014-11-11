/*  TODO:
 *
 *  -Bug by 50,50,100, 5.0, 5.0, x
 *
 *  -Change editing finished slots to value changed slots
 *
 *  -Don't allow scan requests if not connected to master
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
