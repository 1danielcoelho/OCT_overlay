#ifndef FORM_H
#define FORM_H

#include <stdint.h>

#include <QMainWindow>
#include <QFileDialog>
#include <QFile>
#include <QDebug>
#include <QElapsedTimer>
#include <QMessageBox>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

#include "qnode.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
  class Form;
}

class Form : public QMainWindow
{
  Q_OBJECT

  public:
  explicit Form(int argc, char** argv, QWidget *parent = 0);
  ~Form();
  void loadDataIntoCloud(PointCloudT::Ptr cloud, std::vector<uint8_t>& data,
                         uint32_t length, uint32_t width, uint32_t depth,
                         uint32_t frame_header, uint32_t file_header);
  void readyCloudForRender(PointCloudT::Ptr cloud);

  public Q_SLOTS: //same as 'slots'

  private Q_SLOTS:
  void on_browse_button_clicked();

  private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
  PointCloudT::Ptr m_raw_cloud;
  PointCloudT::Ptr m_vis_cloud;
  Ui::Form *m_ui;
  QNode m_qnode;
};

#endif // FORM_H
