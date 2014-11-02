#ifndef OCT_stereocamera_overlay_QNODE_HPP_
#define OCT_stereocamera_overlay_QNODE_HPP_


#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/filesystem.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>

#include <QObject>
#include <QFile>
#include <QStringList>
#include <QTextStream>
#include <QThread>
#include <QStringListModel>

//Create a synchronization policy used to match messages based on their
//approximate timestamp. Permits a single callback for multiple,
//not simultaneous messages
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::Image>
        sync_policy;

//Simply shortens code. This namespace is used by the cv_bridge conversions
namespace enc = sensor_msgs::image_encodings;

class QNode : public QThread
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
	//This reimplements the QThread's run() function. This run() will get called
	//after the call to QThread::start() in the constructor. Returning from
	//this run() method will end the execution of this thread
	void run();
	void setupSubscriptions();
	void imageCallback(const sensor_msgs::ImageConstPtr &msg_left,
										 const sensor_msgs::ImageConstPtr &msg_right,
										 const sensor_msgs::ImageConstPtr &msg_disp,
										 const sensor_msgs::ImageConstPtr &msg_depth);

Q_SIGNALS: //Same as 'signals'
	//Signal emitted when the node is about to shut down
	void rosShutdown();	

private:
	ros::NodeHandle* m_nh;
	std::string* m_topic_names;
	cv_bridge::CvImagePtr m_cv_image_ptr;
};


#endif /* OCT_stereocamera_overlay_QNODE_HPP_ */
