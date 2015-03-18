#include "qnode.h"


QNode::QNode(int argc, char **argv) : no_argc(argc), no_argv(argv) {

  m_shutdown = false;
  m_crossbar = new Crossbar;

  m_overlaying = false;
  m_accu_size = 1;
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
  resetAccumulators();

  ROS_INFO("Topic subscription and synchronization completed");
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg_left,
                          const sensor_msgs::ImageConstPtr &msg_right,
                          const sensor_msgs::ImageConstPtr &msg_disp,
                          const sensor_msgs::ImageConstPtr &msg_depth) {
  //ROS_INFO("imageCallback called");

  //If we're in overlaying mode, just display the images
  if(m_overlaying)
  {
      cv::Mat image_left, image_depth;

      // Convert our image message to a cv::Mat
      cv_bridge::CvImagePtr cv_image_ptr;

      cv_image_ptr = cv_bridge::toCvCopy(msg_left, enc::RGB8);
      image_left = cv_image_ptr->image;
      cv_image_ptr = cv_bridge::toCvCopy(msg_depth, enc::TYPE_32FC3);
      image_depth = cv_image_ptr->image;

      int rows = image_left.rows;
      int cols = image_left.cols;
      int num_pts = rows * cols;

      VTK_NEW(vtkPoints, points);
      points->SetNumberOfPoints(num_pts);

      VTK_NEW(vtkTypeUInt8Array, color_array);
      color_array->SetNumberOfComponents(3);
      color_array->SetNumberOfTuples(num_pts);
      color_array->SetName("Colors");

      VTK_NEW(vtkImageData, left_imagedata);
      left_imagedata->SetDimensions(cols, rows, 1);
      left_imagedata->SetNumberOfScalarComponents(3);
      left_imagedata->SetScalarTypeToUnsignedChar();
      left_imagedata->AllocateScalars();

      int point_id = 0;
      for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < cols; j++) {

            float pt_x = image_depth.at<cv::Vec3f>(i,j)[0];
            float pt_y = image_depth.at<cv::Vec3f>(i,j)[1];
            float pt_z = image_depth.at<cv::Vec3f>(i,j)[2];

            unsigned char color[3];
            color[0] = image_left.at<cv::Vec3b>(i, j)[0];
            color[1] = image_left.at<cv::Vec3b>(i, j)[1];
            color[2] = image_left.at<cv::Vec3b>(i, j)[2];

            float color_float[3];
            color_float[0] = (float) color[0];
            color_float[1] = (float) color[1];
            color_float[2] = (float) color[2];

            unsigned char *pixel = static_cast<unsigned char *>(
                left_imagedata->GetScalarPointer(j, i, 0));

            points->SetPoint(point_id, pt_x, pt_y, pt_z);
            color_array->SetTuple(point_id, color_float);
            memcpy(&pixel[0], &color[0], 3);

            point_id++;
        }
      }

      VTK_NEW(vtkPolyData, surf_poly);
      surf_poly->SetPoints(points);
      surf_poly->GetPointData()->SetScalars(color_array);

      Q_EMIT newSurface(surf_poly);

      Q_EMIT newBackground(left_imagedata);
  }

  //We're not overlaying: Accumulate and write images to disk
  else
  {
    // Accumulator is not full yet; Accumulate
    if (m_accu_count < m_accu_size) {
    ROS_INFO("Accumulating images: %u of %u", m_accu_count, m_accu_size);

    cv::Mat image_left, image_right, image_disp, image_depth;

    // Convert our image message to a cv::Mat
    cv_bridge::CvImagePtr cv_image_ptr;

    cv_image_ptr = cv_bridge::toCvCopy(msg_left, enc::RGB8);
    image_left = cv_image_ptr->image;

    cv_image_ptr = cv_bridge::toCvCopy(msg_right, enc::RGB8);
    image_right = cv_image_ptr->image;

    cv_image_ptr = cv_bridge::toCvCopy(msg_disp, enc::TYPE_32FC1);
    image_disp = cv_image_ptr->image;

    cv_image_ptr = cv_bridge::toCvCopy(msg_depth, enc::TYPE_32FC3);
    image_depth = cv_image_ptr->image;

    cv::accumulate(image_left, m_left_accu);
    cv::accumulate(image_right, m_right_accu);
    cv::accumulate(image_disp, m_disp_accu);
    cv::accumulate(image_depth, m_depth_accu);

    m_accu_count++;

    Q_EMIT accumulated((1.0f*m_accu_count)/m_accu_size);
  }

  // Finish accumulating, write to files
  if (m_accu_count == m_accu_size) {
    m_accu_count++; // Only come back here again after accumulator is reset

    test_depth = *msg_depth;

    m_left_accu /= m_accu_size;
    m_right_accu /= m_accu_size;
    m_disp_accu /= m_accu_size;
    m_depth_accu /= m_accu_size;

    uint32_t rows = m_left_accu.rows;
    uint32_t cols = m_left_accu.cols;

    // Cast our 32FC3 to a 8UC3, for display
    // Need to directly cast as opposed to cv::convertScaleAbs, since that would
    // produce distorted brightness levels (a sequence of dark images would have
    // been converted to a single brighter image)
    // cv::Mat result = cv::Mat(rows, cols, CV_8U);
    m_left_accu.convertTo(m_left_accu, CV_8U);
    m_right_accu.convertTo(m_right_accu, CV_8U);

    // Write a simple header
    std::vector<uint32_t> header;
    header.clear();
    header.resize(2);
    memcpy(&header[0], &rows, 4);
    memcpy(&header[1], &cols, 4);

    // Write the three images
    std::vector<uint32_t> left, right, disp;
    left.reserve(rows * cols);
    right.reserve(rows * cols);
    disp.reserve(rows * cols);

    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {
        left.push_back(m_left_accu.at<cv::Vec3b>(i, j)[0] << 16 |
                       m_left_accu.at<cv::Vec3b>(i, j)[1] << 8 |
                       m_left_accu.at<cv::Vec3b>(i, j)[2]);
      }
    }

    m_crossbar->writeVector(header, STEREO_LEFT_CACHE_PATH, false);
    m_crossbar->writeVector(left, STEREO_LEFT_CACHE_PATH, true);

    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {
        right.push_back(m_right_accu.at<cv::Vec3b>(i, j)[0] << 16 |
                       m_right_accu.at<cv::Vec3b>(i, j)[1] << 8 |
                       m_right_accu.at<cv::Vec3b>(i, j)[2]);
      }
    }

    m_crossbar->writeVector(header, STEREO_RIGHT_CACHE_PATH, false);
    m_crossbar->writeVector(right, STEREO_RIGHT_CACHE_PATH, true);


    // Disp map is CV_32FC1; Here we scale it down to CV_8UC1 while normalizing
    // it so the largest value in m_disp_accu gets mapped to 255, and the lowest
    // to 0. We won't use the disp map for anything really, it will just be
    // displayed for debugging, so this is fine
    cv::Mat result;
    cv::normalize(m_disp_accu, result, 0, 255, cv::NORM_MINMAX, CV_8U);

    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {
        disp.push_back(
            result.at<uint8_t>(i, j));  // Implicit uint8_t -> uint32_t
      }
    }

    m_crossbar->writeVector(header, STEREO_DISP_CACHE_PATH, false);
    m_crossbar->writeVector(disp, STEREO_DISP_CACHE_PATH, true);

    std::vector<float> depth;

    //    memcpy(&header_depth[0], &rows_f, 4);
    //    memcpy(&header_depth[1], &cols_f, 4);
    depth.reserve(rows * cols * 3); //3-channel

    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {

          float red = m_depth_accu.at<cv::Vec3f>(i,j)[0];
          float green = m_depth_accu.at<cv::Vec3f>(i,j)[1];
          float blue = m_depth_accu.at<cv::Vec3f>(i,j)[2];

          depth.push_back(red);
          depth.push_back(green);
          depth.push_back(blue);
      }
    }

    m_crossbar->writeVector(header, STEREO_DEPTH_CACHE_PATH, false);
    m_crossbar->writeVector(depth, STEREO_DEPTH_CACHE_PATH, true);

    //Lets Form know it can read the stereo images from the cache locations
    Q_EMIT receivedStereoImages();
    }
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
      m_crossbar->writeVector(octSrvMessage.response.octImage.data,
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
  m_crossbar->readVector(OCT_RAW_CACHE_PATH, data);

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

      // Store our surface to disk so we can use it for registration
      m_crossbar->writePCL(pts, OCT_SURF_CACHE_PATH);

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
          0.1 * sin(3 * 3.141592 * (1.0 * i) / params.length_steps) + 1.5;
    }
  }

  m_crossbar->writePCL(pts, OCT_SURF_CACHE_PATH);

#endif

  // Even if the call failed, emit this signal. Form will fail to read the file
  // but at least it won't wait indefinitely
  Q_EMIT receivedOCTSurfData(params);
}

void QNode::requestRegistration() {
  ROS_INFO("OCT surface to depth map registration requested");

  //Read our vector from cache
  std::vector<float> depth_vector;
  m_crossbar->readVector(STEREO_DEPTH_CACHE_PATH, depth_vector);

  //Convert the vector to a ROS image message
  cv::Mat depth_mat;
  m_crossbar->floatVectorToCvMat(depth_vector, depth_mat);

  //CvImagePtr is a typedef of boost::shared_ptr<CvImage>
  cv_bridge::CvImagePtr depth_cv_image_ptr;
  depth_cv_image_ptr.reset(new cv_bridge::CvImage());

  depth_cv_image_ptr->image = depth_mat;
  depth_cv_image_ptr->encoding = "32FC3";
  sensor_msgs::ImagePtr depth_ros_image_msg = depth_cv_image_ptr->toImageMsg();

  // Read OCT surface PCL and create a PointCloud2 with it
  pcl::PointCloud<pcl::PointXYZ>::Ptr oct_surface(
      new pcl::PointCloud<pcl::PointXYZ>);
  m_crossbar->readPCL(OCT_SURF_CACHE_PATH, oct_surface);
  sensor_msgs::PointCloud2 oct_surface_msg;
  pcl::toROSMsg(*oct_surface, oct_surface_msg);

#ifndef AT_HOME

  // Create a service request message
  OCT_registration::registrationService registrationMessage;
  registrationMessage.request.registrationMatrixSavePath = VIS_TRANS_CACHE_PATH;
  registrationMessage.request.pclOctSurface = oct_surface_msg;
  registrationMessage.request.cvDepthMap = *depth_ros_image_msg;

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

void QNode::setAccumulatorSize(unsigned int n) {
  n > 0 ? m_accu_size = n : m_accu_size = 1;
  resetAccumulators();
}

void QNode::resetAccumulators() {
  int rows, cols;
  // Tries to grab the rows and cols from the ROS param server; Uses third
  // arguments if can't
  m_nh->param<int>("imageHeight", rows, 480);
  m_nh->param<int>("imageWidth", cols, 640);

  // Left and right accus are also float or else they would overflow
  m_left_accu.create(rows, cols, CV_32FC3);
  m_right_accu.create(rows, cols, CV_32FC3);
  m_disp_accu.create(rows, cols, CV_32FC1);
  m_depth_accu.create(rows, cols, CV_32FC3);

  // Reset, in case it hasn't re-allocated
  m_left_accu = cv::Mat::zeros(rows, cols, CV_32FC3);
  m_right_accu = cv::Mat::zeros(rows, cols, CV_32FC3);
  m_disp_accu = cv::Mat::zeros(rows, cols, CV_32FC1);
  m_depth_accu = cv::Mat::zeros(rows, cols, CV_32FC3);

  m_accu_count = 0;
}

void QNode::startOverlay()
{
    m_overlaying = true;

    ROS_INFO("Loading visualization mesh");

    while(m_overlaying)
    {
        // Cameras run at 30 fps
        static ros::Rate loop_rate(30);
        loop_rate.sleep();

        // Checks our subscriptions
        ros::spinOnce();

        // Check for signals
        QCoreApplication::processEvents();
    }
}

void QNode::stopOverlay()
{
    m_overlaying = false;
}
