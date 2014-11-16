#include "qnode.h"




QNode::QNode(int argc, char** argv ) : no_argc(argc), no_argv(argv)
{
	m_shutdown = false;
	m_file_manager = new FileManager;
}





QNode::~QNode()
{
	stopCurrentNode();
	m_shutdown = true;

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

  ROS_INFO("Connected to the Master node");

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
  //m_oct_tcp_client = m_nh->
  //    serviceClient<oct_client::octClientServiceTCP>("oct_client_service_TCP");

  //Subscribes to OCT_segmentation's segmentation service
//  m_segmentation_client = m_nh->serviceClient
//      <OCT_segmentation::segmentationServiceFromDataArray>
//      ("segmentation_service_from_data_array");

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

  //Swaps the current synchronizer object for nothing, effectively deleting the
  //previous one. Since it needs to disconnect it's previous subscriptions, we
  //need to run this, disconnecting them before they get deleted right below
  m_synchronizer.reset();

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
  m_synchronizer.reset( new
  message_filters::Synchronizer<myPolicyType>(myPolicyType(4),*m_left_image_sub,
      *m_right_image_sub, *m_disp_image_sub, *m_depth_image_sub));

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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts(
        new pcl::PointCloud<pcl::PointXYZRGB>);

  //Build a PCL cloud out of the depth map
  uint32_t rows = depth_map.rows;
  uint32_t cols = depth_map.cols;
  for (uint32_t idx = 0; idx < cols; idx++)
  {
    for (uint32_t idy = 0; idy < rows; idy++)
    {
      pcl::PointXYZRGB point;
      point.x = depth_map.at<cv::Vec3f>(idy,idx)[0];
      point.y = depth_map.at<cv::Vec3f>(idy,idx)[1];
      point.z = depth_map.at<cv::Vec3f>(idy,idx)[2];
      uint32_t rgb = (image_left.at<cv::Vec3b>(idy,idx)[0] << 16 |
                      image_left.at<cv::Vec3b>(idy,idx)[1] << 8 |
                      image_left.at<cv::Vec3b>(idy,idx)[2]);
      point.rgb = *reinterpret_cast<float*>(&rgb);
      pts->points.push_back(point);
    }
  }

  //Write PCL cloud to disk
  m_file_manager->writePCL(pts, STEREO_DEPTH_CACHE_PATH);

  ROS_INFO("Depth map PCL cloud written");

  //Build uin32_t arrays
  std::vector<uint32_t> left, right, disp, header;

  //Populate header with information about the left image dimensions
  rows = image_left.rows;
  cols = image_left.cols;
  header.clear();
  header.resize(8);
  memcpy(&header[0],  &rows,  4*sizeof(uint8_t));
  memcpy(&header[4],  &cols,  4*sizeof(uint8_t));

  //Populate the left vector with the raw data from the left image
  left.reserve(rows*cols);
  for(uint32_t i = 0; i < rows; i++)
  {
    for(uint32_t j = 0; j < cols; j++)
    {
      left.push_back(image_left.at<cv::Vec3b>(i,j)[0] << 16 |
                     image_left.at<cv::Vec3b>(i,j)[1] << 8 |
                     image_left.at<cv::Vec3b>(i,j)[2]);
    }
  }

  //Write the header and append the vector to the same file
  m_file_manager->writeVector(header, STEREO_LEFT_CACHE_PATH, false);
  m_file_manager->writeVector(left, STEREO_LEFT_CACHE_PATH, true);

  ROS_INFO("Left image vector written");

  //Populate header with information about the right image dimensions
  rows = image_right.rows;
  cols = image_left.cols;
  header.clear();
  header.resize(8);
  memcpy(&header[0],  &rows,  4*sizeof(uint8_t));
  memcpy(&header[4],  &cols,  4*sizeof(uint8_t));

  //Populate the right vector with the raw data from the right image
  right.reserve(rows*cols);
  for(uint32_t i = 0; i < rows; i++)
  {
    for(uint32_t j = 0; j < cols; j++)
    {
      //Assembles a BGR 32bit int
      right.push_back(image_right.at<cv::Vec3b>(i,j)[0] << 16 |
                      image_right.at<cv::Vec3b>(i,j)[1] << 8 |
                      image_right.at<cv::Vec3b>(i,j)[2]);
    }
  }

  //Write the header and append the vector to the same file
  m_file_manager->writeVector(header, STEREO_RIGHT_CACHE_PATH, false);
  m_file_manager->writeVector(right, STEREO_RIGHT_CACHE_PATH, true);

  ROS_INFO("Right image vector written");

  //Populate header with information about the displacement image dimensions
  rows = disp_map.rows;
  cols = disp_map.cols;
  header.clear();
  header.resize(8);
  memcpy(&header[0],  &rows,  4*sizeof(uint8_t));
  memcpy(&header[4],  &cols,  4*sizeof(uint8_t));

  //Populate displacement vector with the raw data from the disp map
  disp.reserve(rows*cols);
  for(uint32_t i = 0; i < rows; i++)
  {
    for(uint32_t j = 0; j < cols; j++)
    {
      disp.push_back(disp_map.at<float>(i,j));
    }
  }

  //Write the header and append the vector to the same file
  m_file_manager->writeVector(header, STEREO_DISP_CACHE_PATH, false);
  m_file_manager->writeVector(disp, STEREO_DISP_CACHE_PATH, true);

  ROS_INFO("Displacement map vector written");

  Q_EMIT receivedStereoData();
}





void QNode::stopCurrentNode()
{
  ROS_INFO("Stopping current ROS node");

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

	testImages();

	while (!m_shutdown)
	{
		static ros::Rate loop_rate(1);
		loop_rate.sleep();

		//ROS_INFO("Executing");

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





void QNode::requestScan(OCTinfo params)
{
	ROS_INFO("OCT scan requested");

	std::vector<uint8_t> data;
	data.resize(params.length_steps * params.width_steps * params.depth_steps);

	for(int i = 0; i < data.size(); i++)
	{
		data[i] = i%256;
	}


  std::cout << "Requested with ls: " << params.length_steps << ", ws: " <<
      params.width_steps << ", ds: " << params.depth_steps << ", lr: " <<
      params.length_range << ", wr: " << params.width_range << ", size: " <<
      data.size() << std::endl;

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
//			data = octSrvMessage.response.octImage;

//			ROS_INFO("OCT scan completed");

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

	//Writes our data vector to a file to save some RAM. Form will read this
	m_file_manager->writeVector(data, OCT_RAW_CACHE_PATH);
	data.clear();

	//Emit this only after writeVector returned, so the file is closed
	Q_EMIT receivedOCTRawData(params);
}




void QNode::requestSegmentation(OCTinfo params, std::vector<uint8_t> raw_data)
{
	ROS_INFO("OCT surface segmentation requested");

	//Pack params and raw_data into a segmentationServerFromDataArray srv request
//	OCT_segmentation::segmentationServiceFromDataArray segmentationMessage;
//	segmentationMessage.request.length_steps = params.length_steps;
//	segmentationMessage.request.width_steps = params.width_steps;
//	segmentationMessage.request.depth_steps = params.depth_steps;
//	segmentationMessage.request.length_range = params.length_range;
//	segmentationMessage.request.width_range = params.width_range;
//	segmentationMessage.request.depth_range = params.depth_range;
//	segmentationMessage.request.length_offset = params.length_offset;
//	segmentationMessage.request.width_offset = params.width_offset;
//	segmentationMessage.request.data = raw_data;

	//Call service
//	if(m_oct_tcp_client.exists())
//	{
//		if(m_oct_tcp_client.call(segmentationMessage))
//		{
//			//Unpack surface into our member storage
//			m_oct_pcl_surface = segmentationMessage.response.pclSurface;

//			ROS_INFO("OCT surface segmentation completed");

//		}
//		else
//		{
//			ROS_WARN("Call to segmentation service failed!");
//		}
//	}
//	else
//	{
//		ROS_WARN("Segmentation service does not exist!");
//	}

	//Lets the UI know that it can already pickup it's PCL point cloud
	Q_EMIT receivedOCTSurfData(params);
}





void QNode::testImages()
{
  //Build uin32_t arrays
  std::vector<uint32_t> left, disp, header;

  //Populate header with information about the left image dimensions
  uint32_t rows = 256;
  uint32_t cols = 200;
  header.clear();
  header.resize(2);
  memcpy(&header[0],  &rows,  4*sizeof(uint8_t));
  memcpy(&header[1],  &cols,  4*sizeof(uint8_t));

  //Populate the left vector with the raw data from the left image
  left.reserve(rows*cols);
  for(uint32_t i = 0; i < rows; i++)
  {
    for(uint32_t j = 0; j < cols; j++)
    {
      left.push_back( (0 & 0xff) << 16 | (i & 0xff) << 8 | (j & 0xff) << 0);
    }
  }

  //Write the header and append the vector to the same file
  m_file_manager->writeVector(header, STEREO_LEFT_CACHE_PATH, false);
  m_file_manager->writeVector(left, STEREO_LEFT_CACHE_PATH, true);

  ROS_INFO("Left image vector written");

  //Populate header with information about the displacement image dimensions
  rows = 512;
  cols = 512;
  header.clear();
  header.resize(2);
  memcpy(&header[0],  &rows,  4*sizeof(uint8_t));
  memcpy(&header[1],  &cols,  4*sizeof(uint8_t));

  //Populate displacement vector with the raw data from the disp map
  disp.reserve(rows*cols);
  for(uint32_t i = 0; i < rows; i++)
  {
    for(uint32_t j = 0; j < cols; j++)
    {
      disp.push_back(i*j*10);
    }
  }

  //Write the header and append the vector to the same file
  m_file_manager->writeVector(header, STEREO_DISP_CACHE_PATH, false);
  m_file_manager->writeVector(disp, STEREO_DISP_CACHE_PATH, true);

  ROS_INFO("Displacement map vector written");

  Q_EMIT receivedStereoData();
}

