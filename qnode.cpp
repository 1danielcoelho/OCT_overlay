#include "qnode.h"



QNode::QNode(int argc, char** argv ) : no_argc(argc), no_argv(argv)
{
	m_finish_this_thread = false;

	//Begin executing this QThread. After this, it will call this->run().
	//Returning from run() will end the execution of the thread
	QThread::start();
}





QNode::~QNode()
{
	ROS_INFO("Shutting down");

	qDebug() << "Called destructor";
}





void QNode::connectToMaster()
{
  qDebug() << "connectToMaster called";
  ros::init(no_argc,no_argv,"OCT_overlay");

  //Wait for a master
  while(!ros::master::check())
  {
    ;
  }

  //This needs to be called before a node is created to prevent ros from
  //permanently shutting us down after said node is explicitly deleted
  ros::start();

  qDebug() << "Master found";

	//Signal the UI that Master has connected
	Q_EMIT rosMasterChanged(true);

  //Create a new handle
  m_nh = new ros::NodeHandle;

  //Setup subscriptions
  this->setupSubscriptions();
}





void QNode::run()
{
	qDebug() << "run called\n";

	//We can't use ros::Rate before Master is running
	connectToMaster();

	ros::Rate loop_rate(1);

	while (!m_finish_this_thread)
	{		
		qDebug() << "loop";

		loop_rate.sleep();

		ros::spinOnce();

		//If Master has shut down, kill node and wait for a new master
		if(!ros::master::check())
		{
			qDebug() << "run - shutting down\n";
			stopCurrentNode();

			Q_EMIT rosMasterChanged(false);
			connectToMaster();
		}		
	}
}





void QNode::setupSubscriptions()
{
  //Fetches the image topic names from the ROS param server, or uses the
  //default values (last arguments in the param calls)
  m_topic_names = new std::string[4];
  m_nh->param<std::string>("leftImageTopicName", m_topic_names[0],
      "/stereomatching/image_left");
  m_nh->param<std::string>("rightImageTopicName", m_topic_names[1],
      "/stereomatching/image_right");
  m_nh->param<std::string>("dispImageTopicName", m_topic_names[2],
      "/stereomatching/disparity_map");
  m_nh->param<std::string>("depthImageTopicName", m_topic_names[3],
      "/stereomatching/depth_map");
  ROS_INFO("Topic names fetched");

  //Subscribes to all four image topics. We use message_filters here since
  //the messages published to these topics will come at slightly different
  //time points. Last param is queue size
  m_left_image_sub = new
  message_filters::Subscriber<sensor_msgs::Image>(*m_nh, m_topic_names[0], 1);
  m_right_image_sub = new
  message_filters::Subscriber<sensor_msgs::Image>(*m_nh, m_topic_names[1], 1);
  m_disp_image_sub = new
  message_filters::Subscriber<sensor_msgs::Image>(*m_nh, m_topic_names[2], 1);
  m_depth_image_sub = new
  message_filters::Subscriber<sensor_msgs::Image>(*m_nh, m_topic_names[3], 1);

  //Using the policy typedef'd in the header file, we approximate the
  //message publish time instants and produce a single callback for all
  //of them. syncPolicy takes a queue size as its constructor argument,
  //which we need to be 4 to hold all four images before syncing
  m_synchronizer = new message_filters::Synchronizer<myPolicyType>
      (myPolicyType(4),*m_left_image_sub, *m_right_image_sub, *m_disp_image_sub,
      *m_depth_image_sub);
  //Register which callback will receive the sync'd messages.
  //registerCallback expects a functor (not func pointer) so we use
  //boost::bind
  m_synchronizer->registerCallback(boost::bind(
      &QNode::imageCallback, this, _1, _2, _3, _4));
  ROS_INFO("Topic subscription and synchronization completed");

}





void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg_left,
                          const sensor_msgs::ImageConstPtr &msg_right,
                          const sensor_msgs::ImageConstPtr &msg_disp,
                          const sensor_msgs::ImageConstPtr &msg_depth)
{
  ROS_INFO("imageCallback called");

  //Creates CV matrices for all incoming images
  cv::Mat image_left, image_right, disp_map, depth_map;

  //cv_bridges convert between ROS image messages and opencv images.
  //Here, we create cv images from ROS image messages by doing a deep
  //copy of the ROS image data and returning a mutable object
  m_cv_image_ptr = cv_bridge::toCvCopy(msg_left, enc::RGB8);
  image_left = m_cv_image_ptr->image;
  m_cv_image_ptr = cv_bridge::toCvCopy(msg_right, enc::RGB8);
  image_right = m_cv_image_ptr->image;
  m_cv_image_ptr = cv_bridge::toCvCopy(msg_disp, enc::TYPE_32FC1);
  disp_map = m_cv_image_ptr->image;
  m_cv_image_ptr = cv_bridge::toCvCopy(msg_depth, enc::TYPE_32FC3);
  depth_map = m_cv_image_ptr->image;
  ROS_INFO("Image data fetched");

}





void QNode::stopCurrentNode()
{
  if(ros::isStarted())
  {
    qDebug() << " is started";
    ros::shutdown();
    ros::waitForShutdown();
  }
  delete m_nh;
}




void QNode::terminateThread()
{
  qDebug() << "Called terminateThread\n";
  stopCurrentNode();

  m_finish_this_thread = true;
}





