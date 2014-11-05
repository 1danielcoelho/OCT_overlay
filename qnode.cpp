#include "qnode.h"



QNode::QNode(int argc, char** argv ) : no_argc(argc), no_argv(argv)
{
	m_shutdown = false;
}





QNode::~QNode()
{
	stopCurrentNode();
	m_shutdown = true;

	m_data.clear();

	//Tells the GUI that we're finished deconstructing everything
	Q_EMIT finished();
}





void QNode::connectToMaster()
{
  ros::init(no_argc,no_argv,"OCT_overlay");

  //Wait for a master, or break out of we need to shutdown with no Master
  while(!ros::master::check())
  {
    if(m_shutdown) return;
  }

  //This needs to be called before a node is created to prevent ros from
  //permanently shutting us down after said node is explicitly deleted
  ros::start();

	//Signal the UI that Master has connected
	Q_EMIT rosMasterChanged(true);

  //Create a new handle
  m_nh = new ros::NodeHandle;

  //Setup subscriptions
  this->setupSubscriptions();
}





void QNode::setupSubscriptions()
{
  //Setup a service client for the TCP OCT service from oct_client
  //m_clientTCPOCT = m_nh->
  //    serviceClient<oct_client::octClientServiceTCP>("oct_client_service_TCP");

  //Fetches the image topic names from the ROS param server, or uses the
  //default values (last arguments in the param calls)
  std::string topic_names[4];
  m_nh->param<std::string>("leftImageTopicName", topic_names[0],
      "/stereomatching/image_left");
  m_nh->param<std::string>("rightImageTopicName", topic_names[1],
      "/stereomatching/image_right");
  m_nh->param<std::string>("dispImageTopicName", topic_names[2],
      "/stereomatching/disparity_map");
  m_nh->param<std::string>("depthImageTopicName", topic_names[3],
      "/stereomatching/depth_map");
  ROS_INFO("Topic names fetched");

  //Subscribes to all four image topics. We use message_filters here since
  //the messages published to these topics will come at slightly different
  //time points. Last param is queue size
  m_left_image_sub.reset( new
  message_filters::Subscriber<sensor_msgs::Image>(*m_nh, topic_names[0], 1));
  m_right_image_sub.reset( new
  message_filters::Subscriber<sensor_msgs::Image>(*m_nh, topic_names[1], 1));
  m_disp_image_sub.reset( new
  message_filters::Subscriber<sensor_msgs::Image>(*m_nh, topic_names[2], 1));
  m_depth_image_sub.reset( new
  message_filters::Subscriber<sensor_msgs::Image>(*m_nh, topic_names[3], 1));

  //Using the policy typedef'd in the header file, we approximate the
  //message publish time instants and produce a single callback for all
  //of them. syncPolicy takes a queue size as its constructor argument,
  //which we need to be 4 to hold all four images before syncing
  m_synchronizer.reset( new message_filters::Synchronizer<myPolicyType>
      (myPolicyType(4),*m_left_image_sub, *m_right_image_sub, *m_disp_image_sub,
      *m_depth_image_sub));
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

  m_cv_image_ptr.reset();

  ROS_INFO("Image data fetched");

}





void QNode::stopCurrentNode()
{
  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();

    //Deletes the node handle manually, so it can be created again
    delete m_nh;
  }
}





void QNode::process()
{
	//We can't use ros::Rate before Master is running
	connectToMaster();

	while (!m_shutdown)
	{
		static ros::Rate loop_rate(1);
		loop_rate.sleep();

		ROS_INFO("Executing");

		ros::spinOnce();
		QCoreApplication::processEvents(); //Check for signals from main thread

		//If Master has shut down, kill node and wait for a new master
		if(!ros::master::check())
		{
			stopCurrentNode();

			//Updates the checkbox
			Q_EMIT rosMasterChanged(false);

			connectToMaster();
		}		
	}
}





void QNode::requestScan(int length_steps, int width_steps,
		int depth_steps, float length_range, float width_range,
		float depth_range,float length_offset,float width_offset)
{
	ROS_INFO("Requested a scan");

	m_data.resize(length_steps * width_steps * depth_steps);

	for(int i = 0; i < m_data.size(); i++)
	{
		m_data[i] = i%256;
	}

//	oct_client::octClientServiceTCP octSrvMessage;

//	octSrvMessage.request.x_steps = length_steps;
//	octSrvMessage.request.y_steps = width_steps;
//	octSrvMessage.request.z_steps = depth_steps;

//	octSrvMessage.request.x_range = length_range;
//	octSrvMessage.request.y_range = width_range;
//	octSrvMessage.request.z_range = depth_range;

//	octSrvMessage.request.x_offset = length_offset;
//	octSrvMessage.request.y_offset = depth_offset;

	//Waits for a response
//	if(m_clientTCPOCT.exists())
//	{
//		if(m_clientTCPOCT.call(octSrvMessage))
//		{
//			//This should be a deep copy
//			m_data = octSrvMessage.response.octImage;
//		}
//		else
//		{
//			ROS_WARN("Call to service failed!");
//		}
//	}
//	else
//	{
//		ROS_WARN("Service does not exist!");
//	}


	Q_EMIT receivedData();
}





std::vector<uint8_t>& QNode::getDataReference()
{
	return m_data;
}


