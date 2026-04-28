#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_localization_msgs/msg/kinematic_state.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace fs = std::filesystem;

class SyncExportNode : public rclcpp::Node
{
public:
    SyncExportNode() : Node("sync_export_node")
    {
        this->declare_parameter<std::string>("output_dir", "export_sync");
        this->get_parameter("output_dir", output_dir_);

        int queue_size = 20;

        sub_cam_back_.subscribe(this, "/cam4/compressed");
        sub_cam_back_left_.subscribe(this, "/cam3/compressed");
        sub_cam_front_.subscribe(this, "/cam5/compressed");
        sub_cam_front_left_.subscribe(this, "/cam2/compressed");
        sub_cam_front_right_.subscribe(this, "/cam1/compressed");
        sub_cam_back_right_.subscribe(this, "/cam0/compressed");
        sub_lidar_.subscribe(this, "/rslidar_points");
        sub_kinematic_state_.subscribe(this, "/localization/kinematicstate");

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::PointCloud2,
            autoware_localization_msgs::msg::KinematicState>
            SyncPolicy;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size),
                                                                            sub_cam_back_, sub_cam_back_left_, sub_cam_front_, sub_cam_front_left_,
                                                                            sub_cam_front_right_, sub_cam_back_right_, sub_lidar_, sub_kinematic_state_);

        sync_->registerCallback(std::bind(&SyncExportNode::callback, this,
                                          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                          std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
                                          std::placeholders::_7, std::placeholders::_8));

        RCLCPP_INFO(this->get_logger(), "SyncExportNode started, output_dir=%s", output_dir_.c_str());
    }

private:
    std::string generate_filename(const rclcpp::Time &lidar_time, const std::string &ext)
    {
        int64_t ts_10ms = static_cast<int64_t>(lidar_time.seconds() * 100);
        return std::to_string(ts_10ms) + ext;
    }

    void save_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg,
                    const std::string &camera_name,
                    const rclcpp::Time &lidar_time)
    {
        fs::path dir = fs::path(output_dir_) / camera_name;
        fs::create_directories(dir);

        std::string filename = (dir / generate_filename(lidar_time, ".jpg")).string();

        std::ofstream ofs(filename, std::ios::binary);
        ofs.write(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());
        ofs.close();

        RCLCPP_INFO(this->get_logger(), "Saved image %s", filename.c_str());
    }

    void save_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        fs::path dir = fs::path(output_dir_) / "lidar";
        fs::create_directories(dir);

        rclcpp::Time t = msg->header.stamp;
        std::string filename = (dir / generate_filename(t, ".pcd")).string();

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        if (pcl_pc2.fields.size() >= 4 && pcl_pc2.fields[3].name == "intensity") {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
            pcl::io::savePCDFileBinary(filename, *cloud);
            RCLCPP_INFO(this->get_logger(), "Saved pointcloud %s", filename.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No intensity field found in pointcloud, saving as XYZ");
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
            pcl::io::savePCDFileBinary(filename, *cloud);
            RCLCPP_INFO(this->get_logger(), "Saved pointcloud %s", filename.c_str());
        }
    }

    void save_pose(const autoware_localization_msgs::msg::KinematicState::ConstSharedPtr &msg,
                   const rclcpp::Time &lidar_time)
    {
        fs::path dir = fs::path(output_dir_) / "localization";
        fs::create_directories(dir);

        std::string filename = (dir / generate_filename(lidar_time, ".yaml")).string();

        double timestamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        std::ofstream ofs(filename);
        ofs << std::fixed << std::setprecision(17);

        ofs << "header:\n";
        ofs << "  frameId: " << (msg->header.frame_id.empty() ? "Chassis" : msg->header.frame_id) << "\n";
        ofs << "  timestampSec: " << std::setprecision(1) << timestamp_sec << "\n";
        ofs << "worldFrame: WGS84\n";
        ofs << "poseConfidence: 1.0\n";
        ofs << "status: 2\n";
        ofs << "mode: 1\n";
        ofs << "consistencyToMap: 1.0\n";

        const auto &pose = msg->pose_with_covariance.pose;
        ofs << "pose:\n";
        ofs << "  orientation:\n";
        ofs << "    w: " << std::setprecision(17) << pose.orientation.w << "\n";
        ofs << "    x: " << pose.orientation.x << "\n";
        ofs << "    y: " << pose.orientation.y << "\n";
        ofs << "    z: " << pose.orientation.z << "\n";
        ofs << "  position:\n";
        ofs << "    x: " << pose.position.x << "\n";
        ofs << "    y: " << pose.position.y << "\n";
        ofs << "    z: " << pose.position.z << "\n";

        ofs << "posCov:\n";
        for (size_t i = 0; i < 36; ++i) {
            ofs << "- " << msg->pose_with_covariance.covariance[i] << "\n";
        }

        const auto &twist = msg->twist_with_covariance.twist;
        ofs << "vel:\n";
        ofs << "  x: " << twist.linear.x << "\n";
        ofs << "  y: " << twist.linear.y << "\n";
        ofs << "  z: " << twist.linear.z << "\n";
        ofs << "angularV:\n";
        ofs << "  x: " << twist.angular.x << "\n";
        ofs << "  y: " << twist.angular.y << "\n";
        ofs << "  z: " << twist.angular.z << "\n";

        ofs << "velCov:\n";
        for (size_t i = 0; i < 36; ++i) {
            ofs << "- " << msg->twist_with_covariance.covariance[i] << "\n";
        }

        const auto &accel = msg->accel_with_covariance.accel;
        ofs << "acc:\n";
        ofs << "  x: " << accel.linear.x << "\n";
        ofs << "  y: " << accel.linear.y << "\n";
        ofs << "  z: " << accel.linear.z << "\n";

        ofs.close();
        RCLCPP_INFO(this->get_logger(), "Saved pose %s", filename.c_str());
    }

    void callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam_back,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam_back_left,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam_front,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam_front_left,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam_front_right,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam_back_right,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar,
                  const autoware_localization_msgs::msg::KinematicState::ConstSharedPtr &kinematic_state)
    {
        rclcpp::Time lidar_time = lidar->header.stamp;

        save_image(cam_back, "cam_back", lidar_time);
        save_image(cam_back_left, "cam_back_left", lidar_time);
        save_image(cam_front, "cam_front", lidar_time);
        save_image(cam_front_left, "cam_front_left", lidar_time);
        save_image(cam_front_right, "cam_front_right", lidar_time);
        save_image(cam_back_right, "cam_back_right", lidar_time);
        save_cloud(lidar);
        save_pose(kinematic_state, lidar_time);
    }

    std::string output_dir_;

    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cam_back_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cam_back_left_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cam_front_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cam_front_left_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cam_front_right_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cam_back_right_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;
    message_filters::Subscriber<autoware_localization_msgs::msg::KinematicState> sub_kinematic_state_;

    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::CompressedImage,
            sensor_msgs::msg::PointCloud2,
            autoware_localization_msgs::msg::KinematicState>>>
        sync_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncExportNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
