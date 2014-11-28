#include "qnode.h"

//============================================================================//
//                                                                            //
//  PUBLIC METHODS                                                            //
//                                                                            //
//============================================================================//


QNode::QNode(int argc, char** argv ) : no_argc(argc), no_argv(argv)
{
	m_shutdown = false;
	m_file_manager = new FileManager;
}

//------------------------------------------------------------------------------

QNode::~QNode()
{
	stopCurrentNode();
	m_shutdown = true;

	//Tells the GUI that we're finished deconstructing everything
	Q_EMIT finished();
}

//------------------------------------------------------------------------------

void QNode::connectToMaster()
{
  ros::init(no_argc,no_argv,"OCT_overlay");

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

	m_nh = new ros::NodeHandle;

	this->setupSubscriptions();
}

//------------------------------------------------------------------------------

void QNode::setupSubscriptions()
{
  m_oct_tcp_client = m_nh->
      serviceClient<oct_client::octClientServiceTCP>("oct_client_service_TCP");

  m_segmentation_client = m_nh->serviceClient
      <OCT_segmentation::segmentationServiceFromDataArray>
      ("segmentation_service_from_data_array");

  m_registration_client = m_nh->serviceClient
      <OCT_registration::registrationService>("registration_service");

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

  //Register which callback will receive the sync'd messages
  m_synchronizer->registerCallback(boost::bind(
      &QNode::imageCallback, this, _1, _2, _3, _4));

  ROS_INFO("Topic subscription and synchronization completed");

}

//------------------------------------------------------------------------------

void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg_left,
                          const sensor_msgs::ImageConstPtr &msg_right,
                          const sensor_msgs::ImageConstPtr &msg_disp,
                          const sensor_msgs::ImageConstPtr &msg_depth)
{
  ROS_INFO("imageCallback called");

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

//------------------------------------------------------------------------------

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

//============================================================================//
//                                                                            //
//  PUBLIC Q_SLOTS                                                            //
//                                                                            //
//============================================================================//

void QNode::process()
{	
	connectToMaster();

	while (!m_shutdown)
	{
		static ros::Rate loop_rate(1);
		loop_rate.sleep();

		//ROS_INFO("Executing");

		ros::spinOnce();

		//Check for signals from main thread
		QCoreApplication::processEvents();

		if(!ros::master::check())
		{
			stopCurrentNode();

			//Updates the checkbox
			Q_EMIT rosMasterChanged(false);

			connectToMaster();
		}		
	}
}

//------------------------------------------------------------------------------

void QNode::requestScan(OCTinfo params)
{
    ROS_INFO("OCT scan requested");

	oct_client::octClientServiceTCP octSrvMessage;

    octSrvMessage.request.x_steps = params.length_steps;
    octSrvMessage.request.y_steps = params.width_steps;
    octSrvMessage.request.z_steps = params.depth_steps;

    octSrvMessage.request.x_range = params.length_range;
    octSrvMessage.request.y_range = params.width_range;
    octSrvMessage.request.z_range = params.depth_range;

    octSrvMessage.request.x_offset = params.length_offset;
    octSrvMessage.request.y_offset = params.width_offset;

	if(m_oct_tcp_client.exists())
	{
		if(m_oct_tcp_client.call(octSrvMessage))
        {
            m_file_manager->writeVector(octSrvMessage.response.octImage.data,
                                        OCT_RAW_CACHE_PATH);

            ROS_INFO("OCT scan completed");

		}
		else
		{
			ROS_WARN("Call to service failed!");
		}
	}
	else
	{
		ROS_WARN("Service does not exist!");
    }

	//Emit this only after writeVector returned, so the file is closed
	Q_EMIT receivedOCTRawData(params);
}

//------------------------------------------------------------------------------

void QNode::requestSegmentation(OCTinfo params)
{
	ROS_INFO("OCT surface segmentation requested");

  std::vector<uint8_t> data;
  m_file_manager->readVector(OCT_RAW_CACHE_PATH, data);

  //Pack params and raw_data into a segmentationServerFromDataArray srv request
  OCT_segmentation::segmentationServiceFromDataArray segmentationMessage;
  segmentationMessage.request.length_steps = params.length_steps;
  segmentationMessage.request.width_steps = params.width_steps;
  segmentationMessage.request.depth_steps = params.depth_steps;
  segmentationMessage.request.length_range = params.length_range;
  segmentationMessage.request.width_range = params.width_range;
  segmentationMessage.request.depth_range = params.depth_range;
  segmentationMessage.request.length_offset = params.length_offset;
  segmentationMessage.request.width_offset = params.width_offset;
  segmentationMessage.request.data = data;

  if(m_segmentation_client.exists())
  {
    if(m_segmentation_client.call(segmentationMessage))
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pts(
            new pcl::PointCloud<pcl::PointXYZ>);

      const sensor_msgs::PointCloud2 pclMessage =
          segmentationMessage.response.pclSurface;

      pcl::fromROSMsg(pclMessage, *pts);

      m_file_manager->writePCL(pts, OCT_SURF_CACHE_PATH);

      ROS_INFO("OCT surface segmentation completed");

      //Lets the UI know that it can already pickup its PCL point cloud
      Q_EMIT receivedOCTSurfData(params);
    }
    else
    {
      ROS_WARN("Call to segmentation service failed!");
    }
  }
  else
  {
    ROS_WARN("Segmentation service does not exist!");
  }
}

//------------------------------------------------------------------------------

void QNode::requestRegistration()
{
  ROS_INFO("OCT surface to depth map registration requested");

  //Read Depth map as PCL and build a sensor_msgs::Image with it
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_map_pcl(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  m_file_manager->readPCL(STEREO_DEPTH_CACHE_PATH, depth_map_pcl);
  sensor_msgs::Image depth_map_msg;
  pcl::toROSMsg(*depth_map_pcl, depth_map_msg);

  //Read OCT surface PCL and create a PointCloud2 with it
  pcl::PointCloud<pcl::PointXYZ>::Ptr oct_surface(
      new pcl::PointCloud<pcl::PointXYZ>);
  m_file_manager->readPCL(OCT_SURF_CACHE_PATH, oct_surface);
  sensor_msgs::PointCloud2 oct_surface_msg;
  pcl::toROSMsg(*oct_surface, oct_surface_msg);

  //Create a service request message
  OCT_registration::registrationService registrationMessage;
  registrationMessage.request.registrationMatrixSavePath = VIS_TRANS_CACHE_PATH;
  registrationMessage.request.pclOctSurface = oct_surface_msg;
  registrationMessage.request.cvDepthMap = depth_map_msg;

  //Call the service passing the transform path
  if(m_registration_client.exists())
  {
    if(m_registration_client.call(registrationMessage))
    {
      if(registrationMessage.response.success)
      {
        ROS_INFO("OCT surface to depth map registration completed");

        //Lets the UI know that it can already pickup its transform
        Q_EMIT receivedRegistration();
      }
      else
      {
        ROS_WARN("Registration algorithm failed!");
      }
    }
    else
    {
      ROS_WARN("Call to registration service failed!");
    }
  }
  else
  {
    ROS_WARN("Registration service does not exist!");
  }
}
