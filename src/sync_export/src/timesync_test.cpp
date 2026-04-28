#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>

using ImageMsg = sensor_msgs::msg::CompressedImage;
using PCMsg = sensor_msgs::msg::PointCloud2;
using namespace message_filters;

using MySyncPolicy = sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, PCMsg>;

class TimeSyncTestNode : public rclcpp::Node
{
public:
  TimeSyncTestNode() : Node("sync_node"), count_(0)
  {
    // 1.订阅 imu 话题并注册回调并打印时间戳
    lidar_sub_ = std::make_shared<Subscriber<PCMsg>>(this, "/rslidar_points");
    lidar_sub_->registerCallback<PCMsg::SharedPtr>(
        [&](const PCMsg::SharedPtr &lidar_msg)
        {
          RCLCPP_INFO(get_logger(), "pcd(%u,%u)", lidar_msg->header.stamp.sec,
                      lidar_msg->header.stamp.nanosec);
        });
    // 2.订阅 camera 话题并注册回调函数打印时间戳
    image0_sub_ = std::make_shared<Subscriber<ImageMsg>>(this, "/camera/image0");
    image0_sub_->registerCallback<ImageMsg::SharedPtr>(
        [&](const ImageMsg::SharedPtr &camera0_msg)
        {
          RCLCPP_INFO(get_logger(), "camera0(%u,%u)", camera0_msg->header.stamp.sec,
                      camera0_msg->header.stamp.nanosec);
        });

    image1_sub_ = std::make_shared<Subscriber<ImageMsg>>(this, "/camera/image1");
    image1_sub_->registerCallback<ImageMsg::SharedPtr>(
        [&](const ImageMsg::SharedPtr &camera1_msg)
        {
          RCLCPP_INFO(get_logger(), "camera1(%u,%u)", camera1_msg->header.stamp.sec,
                      camera1_msg->header.stamp.nanosec);
        });

    // image2_sub_ = std::make_shared<Subscriber<ImageMsg>>(this, "/camera/image2");
    // image2_sub_->registerCallback<ImageMsg::SharedPtr>(
    //     [&](const ImageMsg::SharedPtr &camera2_msg)
    //     {
    //       RCLCPP_INFO(get_logger(), "camera2(%u,%u)", camera2_msg->header.stamp.sec,
    //                   camera2_msg->header.stamp.nanosec);
    //     });

    image3_sub_ = std::make_shared<Subscriber<ImageMsg>>(this, "/camera/image3");
    image3_sub_->registerCallback<ImageMsg::SharedPtr>(
        [&](const ImageMsg::SharedPtr &camera3_msg)
        {
          RCLCPP_INFO(get_logger(), "camera3(%u,%u)", camera3_msg->header.stamp.sec,
                      camera3_msg->header.stamp.nanosec);
        });

    image4_sub_ = std::make_shared<Subscriber<ImageMsg>>(this, "/camera/image4");
    image4_sub_->registerCallback<ImageMsg::SharedPtr>(
        [&](const ImageMsg::SharedPtr &camera4_msg)
        {
          RCLCPP_INFO(get_logger(), "camera4(%u,%u)", camera4_msg->header.stamp.sec,
                      camera4_msg->header.stamp.nanosec);
        });

    image5_sub_ = std::make_shared<Subscriber<ImageMsg>>(this, "/camera/image5");
    image5_sub_->registerCallback<ImageMsg::SharedPtr>(
        [&](const ImageMsg::SharedPtr &camera5_msg)
        {
          RCLCPP_INFO(get_logger(), "camera5(%u,%u)", camera5_msg->header.stamp.sec,
                      camera5_msg->header.stamp.nanosec);
        });

    // 3.创建对应策略的同步器同步7个话题，并注册回调函数打印数据
    synchronizer_ = std::make_shared<Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), *image0_sub_, *image1_sub_, *image3_sub_, *image4_sub_, *image5_sub_, *lidar_sub_);
    synchronizer_->registerCallback(
        std::bind(&TimeSyncTestNode::result_callback, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
  }

private:
  void result_callback(const ImageMsg::ConstSharedPtr camera0_msg,
                       const ImageMsg::ConstSharedPtr camera1_msg,
                      //  const ImageMsg::ConstSharedPtr camera2_msg,
                       const ImageMsg::ConstSharedPtr camera3_msg,
                       const ImageMsg::ConstSharedPtr camera4_msg,
                       const ImageMsg::ConstSharedPtr camera5_msg,
                       const PCMsg::ConstSharedPtr lidar_msg)
  {
    // RCLCPP_INFO(get_logger(), "pcd(%u,%u),camera0(%u,%u),camera1(%u,%u),camera2(%u,%u),camera3(%u,%u),camera4(%u,%u),camera5(%u,%u))",
    //             lidar_msg->header.stamp.sec, lidar_msg->header.stamp.nanosec,
    //             camera0_msg->header.stamp.sec, camera0_msg->header.stamp.nanosec,
    //             camera1_msg->header.stamp.sec, camera1_msg->header.stamp.nanosec,
    //             camera2_msg->header.stamp.sec, camera2_msg->header.stamp.nanosec,
    //             camera3_msg->header.stamp.sec, camera3_msg->header.stamp.nanosec,
    //             camera4_msg->header.stamp.sec, camera4_msg->header.stamp.nanosec,
    //             camera5_msg->header.stamp.sec, camera5_msg->header.stamp.nanosec);
    count_++;
    cv_bridge::CvImagePtr cv_ptr0 = cv_bridge::toCvCopy(camera0_msg, "bgr8");
    std::string filename_img0 = "/home/chery-chen/time_synchroize_ws/data/img0/image" +
                                std::to_string(count_) + ".png";
    cv::imwrite(filename_img0, cv_ptr0->image);

    cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(camera1_msg, "bgr8");
    std::string filename_img1 = "/home/chery-chen/time_synchroize_ws/data/img1/image" +
                                std::to_string(count_) + ".png";
    cv::imwrite(filename_img1, cv_ptr1->image);

    // cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(camera2_msg, "bgr8");
    // std::string filename_img2 = "/home/chery-chen/time_synchroize_ws/data/img2/image" +
    //                             std::to_string(count_) + ".png";
    // cv::imwrite(filename_img2, cv_ptr2->image);

    cv_bridge::CvImagePtr cv_ptr3 = cv_bridge::toCvCopy(camera3_msg, "bgr8");
    std::string filename_img3 = "/home/chery-chen/time_synchroize_ws/data/img3/image" +
                                std::to_string(count_) + ".png";
    cv::imwrite(filename_img3, cv_ptr3->image);

    cv_bridge::CvImagePtr cv_ptr4 = cv_bridge::toCvCopy(camera4_msg, "bgr8");
    std::string filename_img4 = "/home/chery-chen/time_synchroize_ws/data/img4/image" +
                                std::to_string(count_) + ".png";
    cv::imwrite(filename_img4, cv_ptr4->image);

    cv_bridge::CvImagePtr cv_ptr5 = cv_bridge::toCvCopy(camera5_msg, "bgr8");
    std::string filename_img5 = "/home/chery-chen/time_synchroize_ws/data/img5/image" +
                                std::to_string(count_) + ".png";
    cv::imwrite(filename_img5, cv_ptr5->image);

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*lidar_msg, cloud);
    std::string filename_pcd = "/home/chery-chen/time_synchroize_ws/data/pcd/pcd" +
                                std::to_string(count_) + ".pcd";
    pcl::io::savePCDFileBinary(filename_pcd, cloud);
  }

  std::shared_ptr<Subscriber<PCMsg>> lidar_sub_;
  std::shared_ptr<Subscriber<ImageMsg>> image0_sub_;
  std::shared_ptr<Subscriber<ImageMsg>> image1_sub_;
  // std::shared_ptr<Subscriber<ImageMsg>> image2_sub_;
  std::shared_ptr<Subscriber<ImageMsg>> image3_sub_;
  std::shared_ptr<Subscriber<ImageMsg>> image4_sub_;
  std::shared_ptr<Subscriber<ImageMsg>> image5_sub_;
  std::shared_ptr<Synchronizer<MySyncPolicy>> synchronizer_;
  int count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeSyncTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
