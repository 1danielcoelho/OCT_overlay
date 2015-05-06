#include "qnode.h"

QNode::QNode(int argc, char **argv) : no_argc(argc), no_argv(argv) {

  m_shutdown = false;
  m_crossbar = new Crossbar;

  m_overlaying = false;
  m_form_is_ready = true;
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

  ROS_INFO("Topic subscription and synchronization completed");
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg_left,
                          const sensor_msgs::ImageConstPtr &msg_right,
                          const sensor_msgs::ImageConstPtr &msg_disp,
                          const sensor_msgs::ImageConstPtr &msg_depth) {
  // If we're in overlaying mode: Reconstruct the colored surface; Send raw
  // pointers to it and to the left stereocamera image as Qt Signals
  if (m_overlaying) {

    // Qt has no "signal dropping", so we need to make sure we don't overwhelm
    // form with new images. This gets set to true by form whenever it is ready
    if (!m_form_is_ready) return;

    m_form_is_ready = false;

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
    color_array->SetNumberOfComponents(4);
    color_array->SetNumberOfTuples(num_pts);
    color_array->SetName("Colors");

    // Both this and the polydata we'll be sending as signals need
    // to be raw pointers. Smart pointers would self-delete if the garbage
    // collector caught them before the signal arrived on Form
    vtkImageData *left_imagedata = vtkImageData::New();
    left_imagedata->SetDimensions(cols, rows, 1);
    left_imagedata->SetNumberOfScalarComponents(3);
    left_imagedata->SetScalarTypeToUnsignedChar();
    left_imagedata->AllocateScalars();

    int point_id = 0;
    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {
        float pt_x = image_depth.at<cv::Vec3f>(i, j)[0];
        float pt_y = image_depth.at<cv::Vec3f>(i, j)[1];
        float pt_z = image_depth.at<cv::Vec3f>(i, j)[2];

        unsigned char color[3];
        color[0] = image_left.at<cv::Vec3b>(i, j)[0];
        color[1] = image_left.at<cv::Vec3b>(i, j)[1];
        color[2] = image_left.at<cv::Vec3b>(i, j)[2];

        float color_float[4];
        color_float[0] = (float)color[0];
        color_float[1] = (float)color[1];
        color_float[2] = (float)color[2];
        color_float[3] = 255; //All points start off full opaque

        // Sets our color in the background image
        unsigned char *pixel = static_cast<unsigned char *>(
            left_imagedata->GetScalarPointer(j, rows - i - 1, 0));
        memcpy(&pixel[0], &color[0], 4);

        // Sets our point in the polydata arrays
        points->SetPoint(point_id, pt_x, pt_y, pt_z);
        color_array->SetTuple(point_id, color_float);

        point_id++;
      }
    }

    vtkPolyData *surf_poly = vtkPolyData::New();
    surf_poly->SetPoints(points);
    surf_poly->GetPointData()->SetScalars(color_array);

    Q_EMIT newBackground(left_imagedata);

    Q_EMIT newSurface(surf_poly);
  }

  // We're not overlaying: Send raw pointers to all four stereocamera images to
  // Form as Qt Signals
  else {
    // Qt has no "signal dropping", so we need to make sure we don't overwhelm
    // form with new images. This gets set to true by form whenever it is ready
    if (!m_form_is_ready) return;

    m_form_is_ready = false;

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

    // Convert disp image from 32FC1 to 8UC1
    cv::Mat image_disp_r;
    cv::normalize(image_disp, image_disp_r, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Convert depth image from 32FC3 to 8UC3
//    cv::Mat image_depth_rgb;
//    cv::normalize(image_depth, image_depth_rgb, 0, 255, cv::NORM_MINMAX,
//                  CV_8UC3);

    int rows = image_left.rows;
    int cols = image_left.cols;

    ROS_INFO("Rows: %d, cols: %d", rows, cols);

    // All the imagedata pointers we'll be sending as signals need
    // to be raw pointers. Smart pointers would self-delete if the garbage
    // collector caught them before the signal arrived on Form
    vtkImageData *left_imagedata = vtkImageData::New();
    left_imagedata->SetDimensions(cols, rows, 1);
    left_imagedata->SetNumberOfScalarComponents(3);
    left_imagedata->SetScalarTypeToUnsignedChar();
    left_imagedata->AllocateScalars();

    vtkImageData *right_imagedata = vtkImageData::New();
    right_imagedata->SetDimensions(cols, rows, 1);
    right_imagedata->SetNumberOfScalarComponents(3);
    right_imagedata->SetScalarTypeToUnsignedChar();
    right_imagedata->AllocateScalars();

    vtkImageData *disp_imagedata = vtkImageData::New();
    disp_imagedata->SetDimensions(cols, rows, 1);
    disp_imagedata->SetNumberOfScalarComponents(3);
    disp_imagedata->SetScalarTypeToUnsignedChar();
    disp_imagedata->AllocateScalars();

    vtkImageData *depth_imagedata = vtkImageData::New();
    depth_imagedata->SetDimensions(cols, rows, 1);
    depth_imagedata->SetNumberOfScalarComponents(3);
    depth_imagedata->SetScalarTypeToFloat();
    depth_imagedata->AllocateScalars();

    unsigned char color[3];
    float position[3];
    unsigned char *pixel;
    float* pixel_float;

    for (uint32_t i = 0; i < rows; i++) {
      for (uint32_t j = 0; j < cols; j++) {

        // Left image
        color[0] = image_left.at<cv::Vec3b>(i, j)[0];
        color[1] = image_left.at<cv::Vec3b>(i, j)[1];
        color[2] = image_left.at<cv::Vec3b>(i, j)[2];
        // Puts our color in the left imagedata. We need to invert the y
        // axis since the (0,0) is at different positions for VTK and OpenCV
        pixel = static_cast<unsigned char *>(
            left_imagedata->GetScalarPointer(j, rows - i - 1, 0));
        memcpy(&pixel[0], &color[0], 3);

        // Right image
        color[0] = image_right.at<cv::Vec3b>(i, j)[0];
        color[1] = image_right.at<cv::Vec3b>(i, j)[1];
        color[2] = image_right.at<cv::Vec3b>(i, j)[2];
        pixel = static_cast<unsigned char *>(
            right_imagedata->GetScalarPointer(j, rows - i - 1, 0));
        memcpy(&pixel[0], &color[0], 3);

        // Disp image
        color[0] = image_disp_r.at<unsigned char>(i, j);
        color[1] = color[0];
        color[2] = color[0];
        pixel = static_cast<unsigned char *>(
            disp_imagedata->GetScalarPointer(j, rows - i - 1, 0));
        memcpy(&pixel[0], &color[0], 3);

        // Depth image
        position[0] = image_depth.at<cv::Vec3f>(i, j)[0];
        position[1] = image_depth.at<cv::Vec3f>(i, j)[1];
        position[2] = image_depth.at<cv::Vec3f>(i, j)[2];
        pixel_float = static_cast<float *>(
            depth_imagedata->GetScalarPointer(j, rows - i - 1, 0));
        memcpy(&pixel_float[0], &position[0], 3*sizeof(float));
      }
    }

    std::vector<vtkImageData *> images;
    images.push_back(left_imagedata);
    images.push_back(right_imagedata);
    images.push_back(disp_imagedata);
    images.push_back(depth_imagedata);

    Q_EMIT newStereoImages(images);
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

  // Read our vector from cache
  std::vector<float> depth_vector;
  m_crossbar->readVector(STEREO_DEPTH_CACHE_PATH, depth_vector);

  // Convert the vector to a ROS image message
  cv::Mat depth_mat;
  m_crossbar->floatVectorToCvMat(depth_vector, depth_mat);

  // CvImagePtr is a typedef of boost::shared_ptr<CvImage>
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

void QNode::startOverlay() {
  m_overlaying = true;
  m_form_is_ready = true;

  ROS_INFO("Loading visualization mesh");

  while (m_overlaying) {
    // Cameras run at 30 fps
    static ros::Rate loop_rate(30);
    loop_rate.sleep();

    // Checks our subscriptions for image callbacks
    ros::spinOnce();

    // Check for signals
    QCoreApplication::processEvents();
  }
}

void QNode::stopOverlay() { m_overlaying = false; }

void QNode::readyForStereoImages() { m_form_is_ready = true; }
