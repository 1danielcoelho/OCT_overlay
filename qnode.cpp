#include "qnode.h"

QNode::QNode(int argc, char **argv) : no_argc(argc), no_argv(argv) {
  m_shutdown = false;
  m_file_manager = new FileManager;

  m_left_accu_size = 1;
  m_depth_accu_size = 1;
}

QNode::~QNode() {
  stopCurrentNode();
  m_shutdown = true;

  // Tells the GUI that we're finished deconstructing everything so it can kill
  // our thread
  Q_EMIT finished();
}

void QNode::connectToMaster() {
  ros::init(no_argc, no_argv, "OCT_overlay");

  while (!ros::master::check()) {
    if (m_shutdown) return;
  }

  ROS_INFO("Connected to the Master node");

  // This needs to be called before a node is created to prevent ros from
  // permanently shutting us down after said node is explicitly deleted
  ros::start();

  // Signal the UI that Master has connected
  Q_EMIT rosMasterChanged(true);

  m_nh = new ros::NodeHandle;

  this->setupSubscriptions();
}

void QNode::setupSubscriptions() {

#ifndef AT_HOME

  m_oct_tcp_client = m_nh->serviceClient<oct_client::octClientServiceTCP>(
      "oct_client_service_TCP");

  m_segmentation_client =
      m_nh->serviceClient<OCT_segmentation::segmentationServiceFromDataArray>(
          "segmentation_service_from_data_array");

  m_registration_client =
      m_nh->serviceClient<OCT_registration::registrationService>(
          "registration_service");

#endif

  // Fetches the image topic names from the ROS param server, or uses the
  // default values (last arguments in the param calls)
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

  // Deletes the current synchronizer object (if it exists). This needs to run
  // before the lines below, where its old subscriptions (if they exist) are
  // deleted
  m_synchronizer.reset();

  // Subscribes to all four image topics. We use message_filters here since
  // the messages published to these topics will come at slightly different
  // time points. Last param is queue size
  m_left_image_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      *m_nh, topic_names[0], 1));
  m_right_image_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      *m_nh, topic_names[1], 1));
  m_disp_image_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      *m_nh, topic_names[2], 1));
  m_depth_image_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      *m_nh, topic_names[3], 1));

  // Using the policy typedef'd in the header file, we approximate the
  // message publish time instants and produce a single callback for all
  // of them. syncPolicy takes a queue size as its constructor argument,
  // which we need to be 4 to hold all four images before syncing
  m_synchronizer.reset(new message_filters::Synchronizer<myPolicyType>(
      myPolicyType(4), *m_left_image_sub, *m_right_image_sub, *m_disp_image_sub,
      *m_depth_image_sub));

  // Register which callback will receive the sync'd messages
  m_synchronizer->registerCallback(
      boost::bind(&QNode::imageCallback, this, _1, _2, _3, _4));

  // Initialize the accumulator matrices with camera dimensions
  std::cout << "setting up subs" << std::endl;
  resetAccumulators();

  ROS_INFO("Topic subscription and synchronization completed");
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg_left,
                          const sensor_msgs::ImageConstPtr &msg_right,
                          const sensor_msgs::ImageConstPtr &msg_disp,
                          const sensor_msgs::ImageConstPtr &msg_depth) {
  ROS_INFO("imageCallback called");

  if (m_left_accu_count < m_left_accu_size) {
    cv::Mat image_left;

    // Convert our image message to a cv::Mat
    cv_bridge::CvImagePtr cv_image_ptr =
        cv_bridge::toCvCopy(msg_left, enc::RGB8);
    image_left = cv_image_ptr->image;

    ROS_INFO("Accumulating to left image: %u of %u", m_left_accu_count,
             m_left_accu_size);

    cv::accumulate(image_left, m_left_accu);
    m_left_accu_count++;
  }

  if (m_left_accu_count == m_left_accu_size) {
    // Make sure we only come back here if we reset the accumulator
    m_left_accu_count++;
    m_left_accu /= m_left_accu_size;

    uint32_t rows = m_left_accu.rows;
    uint32_t cols = m_left_accu.cols;

    // Cast our 32FC3 to a 8UC3, for display
    // Need to directly cast as opposed to cv::convertScaleAbs, since that would
    // produce distorted brightness levels (a sequence of dark images would have
    // been converted to a single brighter image)
    //cv::Mat result = cv::Mat(rows, cols, CV_8U);
    m_left_accu.convertTo(m_left_accu, CV_8U); //Cast itself to 8U, so we can
                                               //use it at the depth map write

    std::vector<uint32_t> header;
    header.clear();
    header.resize(2);
    memcpy(&header[0], &rows, 4);
    memcpy(&header[1], &cols, 4);

    // Populate the left vector with the raw data from the left image
    std::vector<uint32_t> left;
    left.reserve(rows * cols);
    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {
        left.push_back(m_left_accu.at<cv::Vec3b>(i, j)[0] << 16 |
                       m_left_accu.at<cv::Vec3b>(i, j)[1] << 8 |
                       m_left_accu.at<cv::Vec3b>(i, j)[2]);
      }
    }

    // Write the header and append the vector to the same file
    m_file_manager->writeVector(header, STEREO_LEFT_CACHE_PATH, false);
    m_file_manager->writeVector(left, STEREO_LEFT_CACHE_PATH, true);

    ROS_INFO("Left image vector written");

    Q_EMIT receivedLeftImage();
  }

  if (m_right_img_count == 0) {
    m_right_img_count++;

    cv::Mat image_right;

    // Convert our image message to a cv::Mat
    cv_bridge::CvImagePtr cv_image_ptr =
        cv_bridge::toCvCopy(msg_right, enc::RGB8);
    image_right = cv_image_ptr->image;

    uint32_t rows = image_right.rows;
    uint32_t cols = image_right.cols;

    std::vector<uint32_t> header;
    header.clear();
    header.resize(2);
    memcpy(&header[0], &rows, 4);
    memcpy(&header[1], &cols, 4);

    // Populate the right vector with the raw data from the right image
    std::vector<uint32_t> right;
    right.reserve(rows * cols);
    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {
        // Assembles a BGR 32bit int
        right.push_back(image_right.at<cv::Vec3b>(i, j)[0] << 16 |
                        image_right.at<cv::Vec3b>(i, j)[1] << 8 |
                        image_right.at<cv::Vec3b>(i, j)[2]);
      }
    }

    // Write the header and append the vector to the same file
    m_file_manager->writeVector(header, STEREO_RIGHT_CACHE_PATH, false);
    m_file_manager->writeVector(right, STEREO_RIGHT_CACHE_PATH, true);

    ROS_INFO("Right image vector written");

    Q_EMIT receivedRightImage();
  }

  if (m_disp_img_count == 0) {
    m_disp_img_count++;

    cv::Mat disp_map;

    // Convert our image message to a cv::Mat
    cv_bridge::CvImagePtr cv_image_ptr =
        cv_bridge::toCvCopy(msg_disp, enc::TYPE_32FC1);
    disp_map = cv_image_ptr->image;

    uint32_t rows = disp_map.rows;
    uint32_t cols = disp_map.cols;

    std::vector<uint32_t> header;
    header.clear();
    header.resize(2);
    memcpy(&header[0], &rows, 4);
    memcpy(&header[1], &cols, 4);

    // Maps our 32FC1 displacement map to 8UC1, while stretching histogram to
    // the range of [0, 255]
    double min, max;
    cv::minMaxIdx(disp_map, &min, &max);
    cv::Mat result;
    cv::convertScaleAbs(disp_map, result, 255 / max);

    // Populate displacement vector with the raw data from the disp map
    std::vector<uint8_t> disp;
    disp.reserve(rows * cols);
    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {
        disp.push_back(result.at<uint8_t>(i, j));
      }
    }

    // Write the header and append the vector to the same file
    m_file_manager->writeVector(header, STEREO_DISP_CACHE_PATH, false);
    m_file_manager->writeVector(disp, STEREO_DISP_CACHE_PATH, true);

    ROS_INFO("Displacement map vector written");

    Q_EMIT receivedDispImage();
  }

  if (m_depth_accu_count < m_depth_accu_size) {
    cv::Mat depth_map;

    // Convert our image message to a cv::Mat
    cv_bridge::CvImagePtr cv_image_ptr =
        cv_bridge::toCvCopy(msg_depth, enc::TYPE_32FC3);
    depth_map = cv_image_ptr->image;

    ROS_INFO("Accumulating to depth image: %u of %u", m_depth_accu_count,
             m_depth_accu_size);

    cv::accumulate(depth_map, m_depth_accu);
    m_depth_accu_count++;
  }

  if (m_depth_accu_count == m_depth_accu_size &&
      m_left_accu_count > m_left_accu_size) {  // We need the left accu ready
    m_depth_accu_count++; //Only come back here once we reset the depth stack
    m_depth_accu /= m_depth_accu_size;

    uint32_t rows = m_depth_accu.rows;
    uint32_t cols = m_depth_accu.cols;

    // Cast our 64FC3 to 32FC3
    // Here its important to cast directly as opposed to using
    // cv::convertScaleAbs, which would stretch the point cloud to the 32bit max
    cv::Mat result = cv::Mat(rows, cols, CV_32FC3);
    m_left_accu.convertTo(result, CV_32FC3);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    // Build a PCL cloud out of the depth map    
    for (uint32_t idx = 0; idx < cols; idx++) {
      for (uint32_t idy = 0; idy < rows; idy++) {
        pcl::PointXYZRGB point;
        point.x = result.at<cv::Vec3f>(idy, idx)[0];
        point.y = result.at<cv::Vec3f>(idy, idx)[1];
        point.z = result.at<cv::Vec3f>(idy, idx)[2];
        uint32_t rgb = (m_left_accu.at<cv::Vec3b>(idy, idx)[0] << 16 |
                        m_left_accu.at<cv::Vec3b>(idy, idx)[1] << 8 |
                        m_left_accu.at<cv::Vec3b>(idy, idx)[2]);
        point.rgb = *reinterpret_cast<float *>(&rgb);
        pts->points.push_back(point);
      }
    }

    // Write PCL cloud to disk
    m_file_manager->writePCL(pts, STEREO_DEPTH_CACHE_PATH);

    ROS_INFO("Depth map PCL cloud written");

    Q_EMIT receivedDepthImage();
  }
}

void QNode::stopCurrentNode() {
  ROS_INFO("Stopping current ROS node");

  if (ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();

    // Deletes the node handle manually, so it can be created again whenever we
    // find a new Master
    delete m_nh;

    m_left_accu.release();
    m_depth_accu.release();
  }
}

void QNode::process() {
  connectToMaster();

  while (!m_shutdown) {
    static ros::Rate loop_rate(1);
    loop_rate.sleep();

    // ROS_INFO("Executing");

    ros::spinOnce();

    // Check for signals
    QCoreApplication::processEvents();

    if (!ros::master::check()) {
      stopCurrentNode();

      // Updates the checkbox at Form
      Q_EMIT rosMasterChanged(false);

      connectToMaster();
    }
  }
}

void QNode::requestScan(OCTinfo params) {
  ROS_INFO("OCT scan requested");

#ifndef AT_HOME

  oct_client::octClientServiceTCP octSrvMessage;

  octSrvMessage.request.x_steps = params.length_steps;
  octSrvMessage.request.y_steps = params.width_steps;
  octSrvMessage.request.z_steps = params.depth_steps;

  octSrvMessage.request.x_range = params.length_range;
  octSrvMessage.request.y_range = params.width_range;
  octSrvMessage.request.z_range = params.depth_range;

  octSrvMessage.request.x_offset = params.length_offset;
  octSrvMessage.request.y_offset = params.width_offset;

  if (m_oct_tcp_client.exists()) {
    if (m_oct_tcp_client.call(octSrvMessage)) {
      m_file_manager->writeVector(octSrvMessage.response.octImage.data,
                                  OCT_RAW_CACHE_PATH);

      ROS_INFO("OCT scan completed");

    } else {
      ROS_WARN("Call to service failed!");
    }
  } else {
    ROS_WARN("Service does not exist!");
  }

#endif

  // Even if the call failed, emit this signal. Form will fail to read the
  // vector
  // but at least it won't wait indefinitely
  Q_EMIT receivedOCTRawData(params);
}

void QNode::requestSegmentation(OCTinfo params) {
  ROS_INFO("OCT surface segmentation requested");

#ifndef AT_HOME

  std::vector<uint8_t> data;
  m_file_manager->readVector(OCT_RAW_CACHE_PATH, data);

  //.imgs created by the Thorlabs software don't create a depth range since
  // the axial resolution is fixed. In that case it would be zero, so we fix it
  if (params.depth_range == 0) {
    params.depth_range = params.depth_steps / 1024.0 * 2.762;
  }

  // Pack params and raw_data into a segmentationServerFromDataArray srv request
  OCT_segmentation::segmentationServiceFromDataArray segmentationMessage;
  segmentationMessage.request.length_steps = params.width_steps;

  // Inverted length and range. For some reason these are necessary
  segmentationMessage.request.width_steps = params.length_steps;
  segmentationMessage.request.depth_steps = params.depth_steps;
  segmentationMessage.request.length_range = params.width_range;
  segmentationMessage.request.width_range = params.length_range;

  segmentationMessage.request.depth_range = params.depth_range;
  segmentationMessage.request.length_offset = params.width_offset;
  segmentationMessage.request.width_offset = params.length_offset;
  segmentationMessage.request.data.swap(data);

  if (m_segmentation_client.exists()) {
    if (m_segmentation_client.call(segmentationMessage)) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pts(
          new pcl::PointCloud<pcl::PointXYZ>);

      const sensor_msgs::PointCloud2 pclMessage =
          segmentationMessage.response.pclSurface;

      pcl::fromROSMsg(pclMessage, *pts);

      m_file_manager->writePCL(pts, OCT_SURF_CACHE_PATH);

      ROS_INFO("OCT surface segmentation completed");
    } else {
      ROS_WARN("Call to segmentation service failed!");
    }
  } else {
    ROS_WARN("Segmentation service does not exist!");
  }

#else

  pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);

  // Determine the consecutive increments of the oct data
  double length_incrm = params.length_range / params.length_steps;
  double width_incrm = params.width_range / params.width_steps;

  pts->resize(params.width_steps * params.length_steps);

  // Build a random surface
  int index = 0;
  for (int i = 0; i < params.length_steps; i++) {
    for (int j = 0; j < params.width_steps; j++, index++) {
      pts->points[index].x = i * length_incrm;
      pts->points[index].y = j * width_incrm;
      pts->points[index].z =
          0.001 * (rand() % 100) +
          0.5 * sin(3 * 3.141592 * (1.0 * i) / params.length_steps) + 1.5;
    }
  }

  m_file_manager->writePCL(pts, OCT_SURF_CACHE_PATH);

#endif

  // Even if the call failed, emit this signal. Form will fail to read the file
  // but at least it won't wait indefinitely
  Q_EMIT receivedOCTSurfData(params);
}

void QNode::requestRegistration() {
  ROS_INFO("OCT surface to depth map registration requested");

  // Read Depth map as PCL and build a sensor_msgs::Image with it
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_map_pcl(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  m_file_manager->readPCL(STEREO_DEPTH_CACHE_PATH, depth_map_pcl);
  sensor_msgs::Image depth_map_msg;
  pcl::toROSMsg(*depth_map_pcl, depth_map_msg);

  // Read OCT surface PCL and create a PointCloud2 with it
  pcl::PointCloud<pcl::PointXYZ>::Ptr oct_surface(
      new pcl::PointCloud<pcl::PointXYZ>);
  m_file_manager->readPCL(OCT_SURF_CACHE_PATH, oct_surface);
  sensor_msgs::PointCloud2 oct_surface_msg;
  pcl::toROSMsg(*oct_surface, oct_surface_msg);

#ifndef AT_HOME

  // Create a service request message
  OCT_registration::registrationService registrationMessage;
  registrationMessage.request.registrationMatrixSavePath = VIS_TRANS_CACHE_PATH;
  registrationMessage.request.pclOctSurface = oct_surface_msg;
  registrationMessage.request.cvDepthMap = depth_map_msg;

  // Call the service passing the transform path
  if (m_registration_client.exists()) {
    if (m_registration_client.call(registrationMessage)) {
      if (registrationMessage.response.success) {
        ROS_INFO("OCT surface to depth map registration completed");

      } else {
        ROS_WARN("Registration algorithm failed!");
      }
    } else {
      ROS_WARN("Call to registration service failed!");
    }
  } else {
    ROS_WARN("Registration service does not exist!");
  }

#endif
  // Lets the UI know that it can already pickup its transform
  // Even if the call failed, emit this signal. Form will fail to read the file
  // but at least it won't wait indefinitely
  Q_EMIT receivedRegistration();
}

void QNode::setLeftAccumulatorSize(unsigned int n) {
  n > 0 ? m_left_accu_size = n : m_left_accu_size = 1;
  resetAccumulators();
}

void QNode::setDepthAccumulatorSize(unsigned int n) {
  n > 0 ? m_left_accu_size = n : m_left_accu_size = 1;
  resetAccumulators();
}

void QNode::resetAccumulators() {
  int rows, cols;
  m_nh->param<int>("imageHeight", rows, 480);
  m_nh->param<int>("imageWidth", cols, 640);

  // Only re-allocates if dimensions or type are different
  m_left_accu.create(rows, cols, CV_32FC3);
  m_depth_accu.create(rows, cols, CV_32FC3);

  // Reset, in case it hasn't re-allocated
  m_left_accu = cv::Mat::zeros(rows, cols, CV_32FC3);
  m_depth_accu = cv::Mat::zeros(rows, cols, CV_64FC3);

  m_left_accu_count = 0;
  m_depth_accu_count = 0;
  m_right_img_count = 0;
  m_disp_img_count = 0;
}
