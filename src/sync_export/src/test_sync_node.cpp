#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_localization_msgs/msg/kinematic_state.hpp>
#include <beidou_ins_driver/msg/inspva.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <mutex>

constexpr size_t kNumCameras = 6;

struct CameraEntry {
    std::string name;
    std::string topic;
};

class TestSyncNode : public rclcpp::Node
{
public:
    TestSyncNode() : Node("test_sync_node")
    {
        this->declare_parameter<std::string>("cameras_config", "");
        this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
        this->declare_parameter<std::string>("kinematic_state_topic", "/localization/kinematicstate");
        this->declare_parameter<std::string>("inspva_topic", "/beidou/inspva");
        this->declare_parameter<int>("queue_size", 20);

        std::string cameras_config = this->get_parameter("cameras_config").as_string();
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        std::string ks_topic = this->get_parameter("kinematic_state_topic").as_string();
        int queue_size = this->get_parameter("queue_size").as_int();

        if (cameras_config.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Missing cameras_config parameter");
            throw std::runtime_error("missing cameras_config");
        }

        load_cameras_config(cameras_config);

        for (size_t i = 0; i < kNumCameras; ++i) {
            cam_subs_[i].subscribe(this, cameras_[i].topic);
            RCLCPP_INFO(this->get_logger(), "[subscribe] camera %zu: %s", i, cameras_[i].topic.c_str());
        }
        sub_lidar_.subscribe(this, lidar_topic);
        RCLCPP_INFO(this->get_logger(), "[subscribe] lidar: %s", lidar_topic.c_str());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(queue_size),
            cam_subs_[0], cam_subs_[1], cam_subs_[2], cam_subs_[3], cam_subs_[4], cam_subs_[5],
            sub_lidar_);

        sync_->registerCallback(std::bind(&TestSyncNode::callback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
            std::placeholders::_7));

        ks_sub_ = this->create_subscription<autoware_localization_msgs::msg::KinematicState>(
            ks_topic, rclcpp::SensorDataQoS().keep_last(200),
            std::bind(&TestSyncNode::on_kinematic_state, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "[subscribe] kinematicstate (independent): %s", ks_topic.c_str());

        std::string inspva_topic = this->get_parameter("inspva_topic").as_string();
        inspva_sub_ = this->create_subscription<beidou_ins_driver::msg::Inspva>(
            inspva_topic, rclcpp::SensorDataQoS().keep_last(200),
            std::bind(&TestSyncNode::on_inspva, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "[subscribe] inspva (independent): %s", inspva_topic.c_str());

        RCLCPP_INFO(this->get_logger(), "TestSyncNode started, queue_size=%d", queue_size);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::PointCloud2>;

    void load_cameras_config(const std::string &path)
    {
        YAML::Node root = YAML::LoadFile(path);
        for (const auto &c : root["cameras"]) {
            CameraEntry e;
            e.name = c["name"].as<std::string>();
            e.topic = c["topic"].as<std::string>();
            cameras_.push_back(e);
        }
    }

    void callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam0,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam1,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam2,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam3,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam4,
                  const sensor_msgs::msg::CompressedImage::ConstSharedPtr &cam5,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar)
    {
        rclcpp::Time lidar_time = lidar->header.stamp;
        auto ks = find_nearest_kinematic_state(lidar_time);
        auto inspva = find_nearest_inspva(lidar_time);

        std::array<rclcpp::Time, kNumCameras> cam_times{
            cam0->header.stamp, cam1->header.stamp, cam2->header.stamp,
            cam3->header.stamp, cam4->header.stamp, cam5->header.stamp};

        double cam_diff_ms[kNumCameras];
        for (size_t i = 0; i < kNumCameras; ++i) {
            cam_diff_ms[i] = (cam_times[i] - lidar_time).nanoseconds() / 1e6;
        }

        double ks_diff_ms = ks ? (rclcpp::Time(ks->header.stamp) - lidar_time).nanoseconds() / 1e6 : -999.0;
        double inspva_diff_ms = inspva ? (rclcpp::Time(inspva->header.stamp) - lidar_time).nanoseconds() / 1e6 : -999.0;

        RCLCPP_INFO(this->get_logger(),
            "[SYNC #%d] lidar=%.3fs | cam0=%.1fms cam1=%.1fms cam2=%.1fms cam3=%.1fms cam4=%.1fms cam5=%.1fms | ks=%.1fms inspva=%.1fms",
            sync_count_, lidar_time.seconds(),
            cam_diff_ms[0], cam_diff_ms[1], cam_diff_ms[2],
            cam_diff_ms[3], cam_diff_ms[4], cam_diff_ms[5],
            ks_diff_ms, inspva_diff_ms);

        ++sync_count_;
    }

    void on_kinematic_state(const autoware_localization_msgs::msg::KinematicState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(ks_mutex_);
        ks_buffer_.push_back(msg);
        if (ks_buffer_.size() > 200) {
            ks_buffer_.pop_front();
        }
    }

    autoware_localization_msgs::msg::KinematicState::ConstSharedPtr find_nearest_kinematic_state(const rclcpp::Time &target)
    {
        std::lock_guard<std::mutex> lock(ks_mutex_);
        autoware_localization_msgs::msg::KinematicState::ConstSharedPtr best;
        int64_t best_diff = std::numeric_limits<int64_t>::max();
        for (const auto &msg : ks_buffer_) {
            int64_t diff = std::abs((rclcpp::Time(msg->header.stamp) - target).nanoseconds());
            if (diff < best_diff) {
                best_diff = diff;
                best = msg;
            }
        }
        return best;
    }

    void on_inspva(const beidou_ins_driver::msg::Inspva::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(inspva_mutex_);
        inspva_buffer_.push_back(msg);
        if (inspva_buffer_.size() > 200) {
            inspva_buffer_.pop_front();
        }
    }

    beidou_ins_driver::msg::Inspva::ConstSharedPtr find_nearest_inspva(const rclcpp::Time &target)
    {
        std::lock_guard<std::mutex> lock(inspva_mutex_);
        beidou_ins_driver::msg::Inspva::ConstSharedPtr best;
        int64_t best_diff = std::numeric_limits<int64_t>::max();
        for (const auto &msg : inspva_buffer_) {
            int64_t diff = std::abs((rclcpp::Time(msg->header.stamp) - target).nanoseconds());
            if (diff < best_diff) {
                best_diff = diff;
                best = msg;
            }
        }
        return best;
    }

    std::vector<CameraEntry> cameras_;
    std::array<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>, kNumCameras> cam_subs_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Subscription<autoware_localization_msgs::msg::KinematicState>::SharedPtr ks_sub_;
    std::deque<autoware_localization_msgs::msg::KinematicState::ConstSharedPtr> ks_buffer_;
    std::mutex ks_mutex_;

    rclcpp::Subscription<beidou_ins_driver::msg::Inspva>::SharedPtr inspva_sub_;
    std::deque<beidou_ins_driver::msg::Inspva::ConstSharedPtr> inspva_buffer_;
    std::mutex inspva_mutex_;

    int sync_count_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<TestSyncNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("test_sync"), "Exception: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
