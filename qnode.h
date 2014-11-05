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

//Boost
#include <boost/filesystem.hpp>

//OpenCV
#include <cv.h>
#include <opencv2/opencv.hpp>


//Create a synchronization policy used to match messages based on their
//approximate timestamp. Permits a single callback for multiple,
//not simultaneous messages
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::Image>
        myPolicyType;

//Simply shortens code. This namespace is used by the cv_bridge conversions
namespace enc = sensor_msgs::image_encodings;






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
	//Checks to see if a master node exists. If it does, it creates a handle for
	//this node and returns true
	void connectToMaster();
	//This reimplements the QThread's run() function. This run() will get called
	//after the call to QThread::start() in the constructor. Returning from
	//this run() method will end the execution of this thread
	void setupSubscriptions();
	void imageCallback(const sensor_msgs::ImageConstPtr &msg_left,
										 const sensor_msgs::ImageConstPtr &msg_right,
										 const sensor_msgs::ImageConstPtr &msg_disp,
										 const sensor_msgs::ImageConstPtr &msg_depth);
	void stopCurrentNode();
	std::vector<uint8_t>& getDataReference();

Q_SIGNALS: //Same as 'signals'
	void rosMasterChanged(bool);
	void finished();
	void receivedData();

public Q_SLOTS:
	//Just calls the constructor. Used by the UI application to signal that
	//we should shut down for good
	void process();
	void requestScan(int length_steps, int width_steps,
									 int depth_steps, float length_range,
									 float width_range, float depth_range,
									 float length_offset, float width_offset);

private:
	ros::NodeHandle* m_nh;
	ros::ServiceClient m_clientTCPOCT;
	cv_bridge::CvImagePtr m_cv_image_ptr;
	std::vector<uint8_t> m_data;

	//We don't use these
	int no_argc;
	char** no_argv;

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
