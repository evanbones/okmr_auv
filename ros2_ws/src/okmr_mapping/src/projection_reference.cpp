pcl::PointCloud<pcl::PointXYZ>::Ptr
pcl::io::OpenNI2Grabber::convertToXYZPointCloud (const DepthImage::Ptr& depth_image)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);

  cloud->header.seq = depth_image->getFrameID (); //base_link
  cloud->header.stamp = depth_image->getTimestamp (); // node->now, then use pcl_conversion to convert into pcl time type
  cloud->height = depth_height_; 
  cloud->width = depth_width_; //no height, only width, since we arenty inserting all points anymore
  cloud->is_dense = false; //true

  cloud->points.resize (cloud->height * cloud->width); //wont be the same, we only insert masked points (i.e points where mask != 0)

  float constant_x = 1.0f / device_->getDepthFocalLength ();
  float constant_y = 1.0f / device_->getDepthFocalLength ();
  float centerX = ((float)cloud->width - 1.f) / 2.f;
  float centerY = ((float)cloud->height - 1.f) / 2.f;

  //following section requires some mmodification / cannot be used, since we are not using OpenNI2

  if (std::isfinite (depth_parameters_.focal_length_x))
    constant_x =  1.0f / static_cast<float> (depth_parameters_.focal_length_x);

  if (std::isfinite (depth_parameters_.focal_length_y))
    constant_y =  1.0f / static_cast<float> (depth_parameters_.focal_length_y);

  if (std::isfinite (depth_parameters_.principal_point_x))
    centerX =  static_cast<float> (depth_parameters_.principal_point_x);

  if (std::isfinite (depth_parameters_.principal_point_y))
    centerY =  static_cast<float> (depth_parameters_.principal_point_y);

  if ( device_->isDepthRegistered() )
    cloud->header.frame_id = rgb_frame_id_;
  else
    cloud->header.frame_id = depth_frame_id_;


  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const std::uint16_t* depth_map = (const std::uint16_t*) depth_image->getData ();
  if (depth_image->getWidth () != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    // Resize the image if nessacery
    depth_resize_buffer_.resize(depth_width_ * depth_height_);

    depth_image->fillDepthImageRaw (depth_width_, depth_height_, (std::uint16_t*) depth_resize_buffer_.data() );
    depth_map = depth_resize_buffer_.data();
  }

  unsigned depth_idx = 0;
  for (unsigned v = 0; v < depth_height_; ++v)
  {
    for (unsigned u = 0; u < depth_width_; ++u, ++depth_idx)
    {
      pcl::PointXYZ& pt = (*cloud)[depth_idx]; //XYZRGBL
      // Check for invalid measurements
      if (depth_map[depth_idx] == 0 ||
        depth_map[depth_idx] == depth_image->getNoSampleValue () || //cant do this since we arent using pcl depthimage 
        depth_map[depth_idx] == depth_image->getShadowValue ())
      {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }
      pt.z = depth_map[depth_idx] * 0.001f; // same logic
      pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
      pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
    }
  }
  cloud->sensor_origin_.setZero (); //same logic
  cloud->sensor_orientation_.w () = 1.0f;
  cloud->sensor_orientation_.x () = 0.0f;
  cloud->sensor_orientation_.y () = 0.0f;
  cloud->sensor_orientation_.z () = 0.0f;
  return (cloud);
}
