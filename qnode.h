#ifndef OCT_stereocamera_overlay_QNODE_HPP_
#define OCT_stereocamera_overlay_QNODE_HPP_

// Allows me to test the program at home only changing this line
#define AT_HOME

// C, C++
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>

// QT
#include <QObject>
#include <QFile>
#include <QStringList>
#include <QTextStream>
#include <QThread>
#include <QStringListModel>
#include <QDebug>
#include <QCoreApplication>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#ifndef AT_HOME
#include <oct_client/octClientServiceTCP.h>
#include <OCT_segmentation/segmentationServiceFromDataArray.h>
#include <OCT_registration/registrationService.h>
#endif

// Boost
#include <boost/filesystem.hpp>

// OpenCV
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "octinfo.h"
#include "filemanager.h"

// Create a synchronization policy used to match messages based on their
// approximate timestamp. Permits a single callback for multiple,
// not simultaneous messages
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
    sensor_msgs::Image> myPolicyType;

// Simply shortens code. This namespace is used by the cv_bridge conversions
namespace enc = sensor_msgs::image_encodings;

// Tells Qt about our new type. When we register it later, we can use it in
// signals and slots
Q_DECLARE_METATYPE(OCTinfo)
Q_DECLARE_METATYPE(std::vector<uint8_t>)

class QNode : public QObject {
  Q_OBJECT

 public:
  enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

  QNode(int argc, char** argv);
  virtual ~QNode();

  // Waits for a ROS master node. Creates a new node whenever it shows up
  void connectToMaster();

  // Creates subscriptions to topics and services
  void setupSubscriptions();

  // Callback to the stereocamera topic subscription. Packs the four incoming
  // images into the apropriate formats and saves to the correct files
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_left,
                     const sensor_msgs::ImageConstPtr& msg_right,
                     const sensor_msgs::ImageConstPtr& msg_disp,
                     const sensor_msgs::ImageConstPtr& msg_depth);

  // Kills subscriptions and the current node
  void stopCurrentNode();

 public
Q_SLOTS:
  // Infinite execution loop
  void process();

  // Uses oct_client's service to request an OCT scan with the passed params.
  // Writes the result to a cache file also known by Form
  void requestScan(OCTinfo params);

  // Uses the new OCT_segmentation service to perform surface segmentation
  // Input is not only the passed params, but also a raw data vector read from
  // a cache location also known by Form
  void requestSegmentation(OCTinfo params);

  // Uses OCT_registration's service to perform a stereocamera depth-map to oct
  // surface registration. Both are read from cache locations also known by Form
  void requestRegistration();

  // Sets the size 'n' of the accumulator for the left image. The 'left' images
  // written to cache by imageCallback are the result of a mean of the last 'n'
  // of such 'left' images
  void setLeftAccumulatorSize(unsigned int n);

  // Sets the size 'n' of the accumulator for the depth image. The depth images
  // written to cache by imageCallback are the result of a mean of the last 'n'
  // of such depth images
  void setDepthAccumulatorSize(unsigned int n);

  // Cleans the current accumulators and resets the counters for the number of
  // images held by them, as well as fetching new right and disp images
  void resetAccumulators();

Q_SIGNALS:  // Same as 'signals'
  void rosMasterChanged(bool);
  void finished();
  void receivedOCTRawData(OCTinfo params);
  void receivedOCTSurfData(OCTinfo params);
  void receivedLeftImage();
  void receivedRightImage();
  void receivedDispImage();
  void receivedDepthImage();
  void receivedRegistration();

 private:
  ros::NodeHandle* m_nh;
  ros::ServiceClient m_oct_tcp_client, m_segmentation_client,
      m_registration_client;  
  FileManager* m_file_manager;

  // We don't use command line arguments, but ros::init needs something anyway
  int no_argc;
  char** no_argv;

  // Turns true when its time to shutdown
  bool m_shutdown;

  uint32_t m_left_accu_count;
  uint32_t m_depth_accu_count;
  uint32_t m_right_img_count;
  uint32_t m_disp_img_count;

  uint32_t m_left_accu_size;
  uint32_t m_depth_accu_size;

  cv::Mat m_left_accu;
  cv::Mat m_depth_accu;

  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> >
      m_left_image_sub;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> >
      m_right_image_sub;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> >
      m_disp_image_sub;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> >
      m_depth_image_sub;
  boost::shared_ptr<message_filters::Synchronizer<myPolicyType> >
      m_synchronizer;
};

#endif /* OCT_stereocamera_overlay_QNODE_HPP_ */
