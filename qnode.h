#ifndef OCT_stereocamera_overlay_QNODE_HPP_
#define OCT_stereocamera_overlay_QNODE_HPP_

//C, C++
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>

//QT
#include <QObject>
#include <QFile>
#include <QStringList>
#include <QTextStream>
#include <QThread>
#include <QStringListModel>
#include <QDebug>
#include <QCoreApplication>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
//#include <oct_client/octClientServiceTCP.h>
//#include <OCT_segmentation/segmentationServiceFromDataArray.h

//Boost
#include <boost/filesystem.hpp>

//OpenCV
#include <cv.h>
#include <opencv2/opencv.hpp>

#include "octinfo.h"
#include "filemanager.h"

//Create a synchronization policy used to match messages based on their
//approximate timestamp. Permits a single callback for multiple,
//not simultaneous messages
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::Image>
        myPolicyType;

//Simply shortens code. This namespace is used by the cv_bridge conversions
namespace enc = sensor_msgs::image_encodings;



//Tells Qt about our new type. When we register it later, we can use it in
//signals and slots
Q_DECLARE_METATYPE(OCTinfo)
Q_DECLARE_METATYPE(std::vector<uint8_t>)



class QNode : public QObject
{
	Q_OBJECT

public:
	enum LogLevel
	{
		Debug,
		Info,
		Warn,
		Error,
		Fatal
	};

	QNode(int argc, char** argv );
	virtual ~QNode();

	//Waits for a ROS master node. Creates a new node whenever it shows up
	void connectToMaster();

	//Creates subscriptions to topics and services
	void setupSubscriptions();

	//Callback to the stereocamera topic subscription. Packs the four incoming
	//images into the apropriate formats and saves to the correct files
	void imageCallback(const sensor_msgs::ImageConstPtr &msg_left,
										 const sensor_msgs::ImageConstPtr &msg_right,
										 const sensor_msgs::ImageConstPtr &msg_disp,
										 const sensor_msgs::ImageConstPtr &msg_depth);

	//Kills subscriptions and the current node
	void stopCurrentNode();

Q_SIGNALS: //Same as 'signals'
	void rosMasterChanged(bool);
	void finished();
	void receivedOCTRawData(OCTinfo params);
	void receivedOCTSurfData(OCTinfo params);
	void receivedStereoData();
	void receivedRegistration();

public Q_SLOTS:
	void process();
	void requestScan(OCTinfo params);
	void requestSegmentation(OCTinfo params);
	void requestRegistration();

private:
	ros::NodeHandle* m_nh;
	ros::ServiceClient m_oct_tcp_client, m_segmentation_client,
										 m_registration_client;
	cv_bridge::CvImagePtr m_cv_image_ptr;
	FileManager* m_file_manager;

	//We don't use command line arguments, but ros::init needs something anyway
	int no_argc;
	char** no_argv;

	//Turns true when its time to shutdown
	bool m_shutdown;

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
