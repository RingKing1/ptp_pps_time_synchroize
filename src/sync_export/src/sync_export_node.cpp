#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_localization_msgs/msg/kinematic_state.hpp>
#include <beidou_ins_driver/msg/inspva.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <sqlite3.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <array>
#include <deque>
#include <mutex>
#include <limits>

namespace fs = std::filesystem;

namespace {

constexpr size_t kNumCameras = 6;

struct CameraEntry {
    std::string name;
    std::string topic;
};

std::vector<double> read_json_number_array(const std::string &path, const std::string &key)
{
    std::ifstream ifs(path);
    if (!ifs) {
        throw std::runtime_error("Cannot open calib JSON: " + path);
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    std::string text = ss.str();

    std::string needle = "\"" + key + "\"";
    size_t kpos = text.find(needle);
    if (kpos == std::string::npos) {
        throw std::runtime_error("Key '" + key + "' not found in " + path);
    }
    size_t lb = text.find('[', kpos);
    size_t rb = text.find(']', lb);
    if (lb == std::string::npos || rb == std::string::npos) {
        throw std::runtime_error("Malformed array for '" + key + "' in " + path);
    }
    std::string body = text.substr(lb + 1, rb - lb - 1);
    for (char &c : body) {
        if (c == ',') c = ' ';
    }
    std::istringstream iss(body);
    std::vector<double> out;
    double v;
    while (iss >> v) out.push_back(v);
    return out;
}

}  // namespace

class SyncExportNode : public rclcpp::Node
{
public:
    SyncExportNode() : Node("sync_export_node")
    {
        this->declare_parameter<std::string>("output_dir", "export_sync");
        this->declare_parameter<std::string>("cameras_config", "");
        this->declare_parameter<std::string>("calib_dir", "");
        this->declare_parameter<int>("queue_size", 20);
        this->declare_parameter<int>("max_sync_frames", 0);
        this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
        this->declare_parameter<std::string>("kinematic_state_topic", "/localization/kinematicstate");
        this->declare_parameter<std::string>("inspva_topic", "/beidou/inspva");
        this->declare_parameter<bool>("enable_undistort", true);

        this->get_parameter("output_dir", output_dir_);
        enable_undistort_ = this->get_parameter("enable_undistort").as_bool();
        std::string cameras_config;
        this->get_parameter("cameras_config", cameras_config);
        this->get_parameter("calib_dir", calib_dir_);
        int queue_size = this->get_parameter("queue_size").as_int();
        max_sync_frames_ = this->get_parameter("max_sync_frames").as_int();
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        std::string ks_topic = this->get_parameter("kinematic_state_topic").as_string();
        std::string inspva_topic = this->get_parameter("inspva_topic").as_string();

        if (cameras_config.empty() || calib_dir_.empty()) {
            RCLCPP_FATAL(this->get_logger(),
                "Missing required parameters cameras_config / calib_dir.");
            throw std::runtime_error("missing required params");
        }

        load_cameras_config(cameras_config);
        if (cameras_.size() != kNumCameras) {
            RCLCPP_FATAL(this->get_logger(),
                "cameras_config must define exactly %zu cameras, got %zu",
                kNumCameras, cameras_.size());
            throw std::runtime_error("camera count mismatch");
        }

        load_undistort_maps();
        ensure_output_dirs();
        open_sqlite();

        for (size_t i = 0; i < kNumCameras; ++i) {
            cam_subs_[i].subscribe(this, cameras_[i].topic);
        }
        sub_lidar_.subscribe(this, lidar_topic);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(queue_size),
            cam_subs_[0], cam_subs_[1], cam_subs_[2], cam_subs_[3], cam_subs_[4], cam_subs_[5],
            sub_lidar_);

        sync_->registerCallback(std::bind(&SyncExportNode::callback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
            std::placeholders::_7));

        // kinematicstate 独立高频订阅，避免加入 8 输入同步导致 ApproximateTime 死锁
        ks_sub_ = this->create_subscription<autoware_localization_msgs::msg::KinematicState>(
            ks_topic, rclcpp::SensorDataQoS().keep_last(200),
            std::bind(&SyncExportNode::on_kinematic_state, this, std::placeholders::_1));

        // inspva 独立订阅，用于提供 WGS84 经纬高
        inspva_sub_ = this->create_subscription<beidou_ins_driver::msg::Inspva>(
            inspva_topic, rclcpp::SensorDataQoS().keep_last(200),
            std::bind(&SyncExportNode::on_inspva, this, std::placeholders::_1));

        const char* undistort_str = enable_undistort_ ? "enabled" : "disabled";
        if (max_sync_frames_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                "SyncExportNode started, output_dir=%s, cameras=%zu, max_sync_frames=%d, undistort=%s",
                output_dir_.c_str(), cameras_.size(), max_sync_frames_, undistort_str);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "SyncExportNode started, output_dir=%s, cameras=%zu (no frame limit), undistort=%s",
                output_dir_.c_str(), cameras_.size(), undistort_str);
        }
    }

    ~SyncExportNode() override
    {
        if (ins_db_) {
            if (in_transaction_) {
                sqlite3_exec(ins_db_, "COMMIT;", nullptr, nullptr, nullptr);
                in_transaction_ = false;
            }
            sqlite3_finalize(odom_stmt_);
            sqlite3_finalize(accel_stmt_);
            sqlite3_close(ins_db_);
        }
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
        if (!root["cameras"] || !root["cameras"].IsSequence()) {
            throw std::runtime_error("cameras_config missing top-level 'cameras' list: " + path);
        }
        for (const auto &c : root["cameras"]) {
            CameraEntry e;
            e.name = c["name"].as<std::string>();
            e.topic = c["topic"].as<std::string>();
            cameras_.push_back(e);
        }
    }

    void load_undistort_maps()
    {
        for (size_t i = 0; i < cameras_.size(); ++i) {
            const auto &cam = cameras_[i];
            std::string calib_path = (fs::path(calib_dir_) / (cam.name + ".json")).string();

            auto K_vec = read_json_number_array(calib_path, "intrinsic");
            auto D_vec = read_json_number_array(calib_path, "distortion");
            if (K_vec.size() != 9 || D_vec.empty()) {
                throw std::runtime_error("Bad K/D in " + calib_path);
            }
            cv::Mat K = (cv::Mat_<double>(3, 3) <<
                K_vec[0], K_vec[1], K_vec[2],
                K_vec[3], K_vec[4], K_vec[5],
                K_vec[6], K_vec[7], K_vec[8]);
            cv::Mat D = cv::Mat(1, static_cast<int>(D_vec.size()), CV_64F, D_vec.data()).clone();
            K_[i] = K;
            D_[i] = D;
            maps_initialized_[i] = false;

            RCLCPP_INFO(this->get_logger(),
                "[calib] %s: K loaded (fx=%.2f fy=%.2f), distortion=%zu coeffs",
                cam.name.c_str(), K_vec[0], K_vec[4], D_vec.size());
        }
    }

    void ensure_output_dirs()
    {
        fs::create_directories(fs::path(output_dir_) / "lidar");
        fs::create_directories(fs::path(output_dir_) / "localization");
        fs::create_directories(fs::path(output_dir_) / "INS");
        for (const auto &cam : cameras_) {
            fs::create_directories(fs::path(output_dir_) / "camera" / cam.name);
        }
    }

    void open_sqlite()
    {
        std::string db_path = (fs::path(output_dir_) / "INS" / "ins_export.sqlite3").string();
        if (sqlite3_open(db_path.c_str(), &ins_db_) != SQLITE_OK) {
            throw std::runtime_error("Failed to open sqlite3 at " + db_path + ": " + sqlite3_errmsg(ins_db_));
        }

        const char *pragmas =
            "PRAGMA journal_mode=WAL;"
            "PRAGMA synchronous=NORMAL;";
        sqlite3_exec(ins_db_, pragmas, nullptr, nullptr, nullptr);

        const char *create_odom =
            "CREATE TABLE IF NOT EXISTS t_export_odometry ("
            "bag_ts INTEGER PRIMARY KEY,"
            "msg_ts REAL NOT NULL,"
            "status TEXT NOT NULL,"
            "pos_x REAL NOT NULL, pos_y REAL NOT NULL, pos_z REAL NOT NULL,"
            "ori_x REAL NOT NULL, ori_y REAL NOT NULL, ori_z REAL NOT NULL, ori_w REAL NOT NULL,"
            "linear_x REAL NOT NULL, linear_y REAL NOT NULL, linear_z REAL NOT NULL,"
            "angular_x REAL NOT NULL, angular_y REAL NOT NULL, angular_z REAL NOT NULL);";

        const char *create_accel =
            "CREATE TABLE IF NOT EXISTS t_export_accel ("
            "bag_ts INTEGER PRIMARY KEY,"
            "msg_ts REAL NOT NULL,"
            "linear_x REAL NOT NULL, linear_y REAL NOT NULL, linear_z REAL NOT NULL,"
            "angular_x REAL NOT NULL, angular_y REAL NOT NULL, angular_z REAL NOT NULL);";

        char *err = nullptr;
        if (sqlite3_exec(ins_db_, create_odom, nullptr, nullptr, &err) != SQLITE_OK) {
            std::string m = err ? err : "(null)";
            sqlite3_free(err);
            throw std::runtime_error("create t_export_odometry failed: " + m);
        }
        if (sqlite3_exec(ins_db_, create_accel, nullptr, nullptr, &err) != SQLITE_OK) {
            std::string m = err ? err : "(null)";
            sqlite3_free(err);
            throw std::runtime_error("create t_export_accel failed: " + m);
        }

        const char *insert_odom =
            "INSERT OR IGNORE INTO t_export_odometry VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?);";
        const char *insert_accel =
            "INSERT OR IGNORE INTO t_export_accel VALUES (?,?,?,?,?,?,?,?);";
        sqlite3_prepare_v2(ins_db_, insert_odom, -1, &odom_stmt_, nullptr);
        sqlite3_prepare_v2(ins_db_, insert_accel, -1, &accel_stmt_, nullptr);

        sqlite3_exec(ins_db_, "BEGIN;", nullptr, nullptr, nullptr);
        in_transaction_ = true;

        RCLCPP_INFO(this->get_logger(), "INS sqlite3 opened at %s", db_path.c_str());
    }

    std::string generate_filename(const rclcpp::Time &lidar_time, const std::string &ext)
    {
        int64_t ts_10ms = static_cast<int64_t>(lidar_time.seconds() * 100);
        return std::to_string(ts_10ms) + ext;
    }

    void on_inspva(beidou_ins_driver::msg::Inspva::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(inspva_mutex_);
        inspva_buffer_.push_back(msg);
        if (inspva_buffer_.size() > 200) {
            inspva_buffer_.pop_front();
        }
    }

    beidou_ins_driver::msg::Inspva::ConstSharedPtr find_nearest_inspva(const rclcpp::Time &target_time)
    {
        std::lock_guard<std::mutex> lock(inspva_mutex_);
        if (inspva_buffer_.empty()) return nullptr;

        double target_sec = target_time.seconds();
        beidou_ins_driver::msg::Inspva::ConstSharedPtr nearest = nullptr;
        double min_diff = std::numeric_limits<double>::max();

        for (const auto &msg : inspva_buffer_) {
            double t = static_cast<double>(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9;
            double diff = std::abs(t - target_sec);
            if (diff < min_diff) {
                min_diff = diff;
                nearest = msg;
            }
        }
        return nearest;
    }

    autoware_localization_msgs::msg::KinematicState::ConstSharedPtr find_nearest_kinematic_state(const rclcpp::Time &target_time)
    {
        std::lock_guard<std::mutex> lock(ks_mutex_);
        if (ks_buffer_.empty()) return nullptr;

        double target_sec = target_time.seconds();
        autoware_localization_msgs::msg::KinematicState::ConstSharedPtr nearest = nullptr;
        double min_diff = std::numeric_limits<double>::max();

        for (const auto &msg : ks_buffer_) {
            double t = static_cast<double>(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9;
            double diff = std::abs(t - target_sec);
            if (diff < min_diff) {
                min_diff = diff;
                nearest = msg;
            }
        }
        return nearest;
    }

    void save_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg,
                    size_t cam_idx, const rclcpp::Time &lidar_time)
    {
        cv::Mat raw = cv::imdecode(cv::Mat(1, static_cast<int>(msg->data.size()), CV_8U,
                                           const_cast<unsigned char *>(msg->data.data())),
                                   cv::IMREAD_COLOR);
        if (raw.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to decode JPEG for %s",
                        cameras_[cam_idx].name.c_str());
            return;
        }

        cv::Mat out_img = raw;
        if (enable_undistort_) {
            if (!maps_initialized_[cam_idx]) {
                cv::initUndistortRectifyMap(
                    K_[cam_idx], D_[cam_idx],
                    cv::Mat(), K_[cam_idx],
                    raw.size(), CV_16SC2,
                    map1_[cam_idx], map2_[cam_idx]);
                maps_initialized_[cam_idx] = true;
                RCLCPP_INFO(this->get_logger(), "[%s] undistort map initialized for %dx%d",
                            cameras_[cam_idx].name.c_str(), raw.cols, raw.rows);
            }
            cv::remap(raw, out_img, map1_[cam_idx], map2_[cam_idx], cv::INTER_LINEAR);
        }

        std::vector<uchar> buf;
        cv::imencode(".jpg", out_img, buf);

        fs::path dir = fs::path(output_dir_) / "camera" / cameras_[cam_idx].name;
        std::string filename = (dir / generate_filename(lidar_time, ".jpg")).string();
        std::ofstream ofs(filename, std::ios::binary);
        ofs.write(reinterpret_cast<const char *>(buf.data()), buf.size());
        ofs.close();

        RCLCPP_DEBUG(this->get_logger(), "Saved image %s", filename.c_str());
    }

    void save_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        fs::path dir = fs::path(output_dir_) / "lidar";
        rclcpp::Time t = msg->header.stamp;
        std::string filename = (dir / generate_filename(t, ".pcd")).string();

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        bool has_intensity = false;
        for (const auto &f : pcl_pc2.fields) {
            if (f.name == "intensity") { has_intensity = true; break; }
        }
        if (has_intensity) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
            pcl::io::savePCDFileBinary(filename, *cloud);
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "No intensity field in pointcloud; saving as XYZ (spec recommends intensity).");
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
            pcl::io::savePCDFileBinary(filename, *cloud);
        }
        RCLCPP_DEBUG(this->get_logger(), "Saved pointcloud %s", filename.c_str());
    }

    void save_pose(const autoware_localization_msgs::msg::KinematicState::ConstSharedPtr &msg,
                   const beidou_ins_driver::msg::Inspva::ConstSharedPtr &inspva,
                   const rclcpp::Time &lidar_time)
    {
        fs::path dir = fs::path(output_dir_) / "localization";
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
        if (inspva) {
            ofs << "    x: " << std::setprecision(17) << inspva->longitude << "\n";
            ofs << "    y: " << std::setprecision(17) << inspva->latitude << "\n";
            ofs << "    z: " << std::setprecision(17) << inspva->height << "\n";
        } else {
            ofs << "    x: " << pose.position.x << "\n";
            ofs << "    y: " << pose.position.y << "\n";
            ofs << "    z: " << pose.position.z << "\n";
        }

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
    }

    void on_kinematic_state(autoware_localization_msgs::msg::KinematicState::ConstSharedPtr msg)
    {
        // 缓存到 ring buffer，供 save_pose 时间最近查找
        {
            std::lock_guard<std::mutex> lock(ks_mutex_);
            ks_buffer_.push_back(msg);
            if (ks_buffer_.size() > 200) {
                ks_buffer_.pop_front();
            }
        }

        // INS sqlite3 写入（保持原始高频）
        if (finished_ || !ins_db_) return;

        auto inspva = find_nearest_inspva(msg->header.stamp);

        int64_t bag_ts_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
                            static_cast<int64_t>(msg->header.stamp.nanosec);
        double msg_ts_sec = static_cast<double>(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9;

        const auto &pose = msg->pose_with_covariance.pose;
        const auto &twist = msg->twist_with_covariance.twist;
        const auto &accel = msg->accel_with_covariance.accel;

        // t_export_odometry
        sqlite3_reset(odom_stmt_);
        int i = 1;
        sqlite3_bind_int64(odom_stmt_, i++, bag_ts_ns);
        sqlite3_bind_double(odom_stmt_, i++, msg_ts_sec);
        sqlite3_bind_text(odom_stmt_, i++, "$", -1, SQLITE_STATIC);
        if (inspva) {
            sqlite3_bind_double(odom_stmt_, i++, inspva->longitude);
            sqlite3_bind_double(odom_stmt_, i++, inspva->latitude);
            sqlite3_bind_double(odom_stmt_, i++, inspva->height);
        } else {
            sqlite3_bind_double(odom_stmt_, i++, pose.position.x);
            sqlite3_bind_double(odom_stmt_, i++, pose.position.y);
            sqlite3_bind_double(odom_stmt_, i++, pose.position.z);
        }
        sqlite3_bind_double(odom_stmt_, i++, pose.orientation.x);
        sqlite3_bind_double(odom_stmt_, i++, pose.orientation.y);
        sqlite3_bind_double(odom_stmt_, i++, pose.orientation.z);
        sqlite3_bind_double(odom_stmt_, i++, pose.orientation.w);
        sqlite3_bind_double(odom_stmt_, i++, twist.linear.x);
        sqlite3_bind_double(odom_stmt_, i++, twist.linear.y);
        sqlite3_bind_double(odom_stmt_, i++, twist.linear.z);
        sqlite3_bind_double(odom_stmt_, i++, twist.angular.x);
        sqlite3_bind_double(odom_stmt_, i++, twist.angular.y);
        sqlite3_bind_double(odom_stmt_, i++, twist.angular.z);
        sqlite3_step(odom_stmt_);

        // t_export_accel
        sqlite3_reset(accel_stmt_);
        i = 1;
        sqlite3_bind_int64(accel_stmt_, i++, bag_ts_ns);
        sqlite3_bind_double(accel_stmt_, i++, msg_ts_sec);
        sqlite3_bind_double(accel_stmt_, i++, accel.linear.x);
        sqlite3_bind_double(accel_stmt_, i++, accel.linear.y);
        sqlite3_bind_double(accel_stmt_, i++, accel.linear.z);
        sqlite3_bind_double(accel_stmt_, i++, twist.angular.x);
        sqlite3_bind_double(accel_stmt_, i++, twist.angular.y);
        sqlite3_bind_double(accel_stmt_, i++, twist.angular.z);
        sqlite3_step(accel_stmt_);

        ++ins_pending_;
        if (ins_pending_ >= 100) {
            sqlite3_exec(ins_db_, "COMMIT;", nullptr, nullptr, nullptr);
            sqlite3_exec(ins_db_, "BEGIN;", nullptr, nullptr, nullptr);
            ins_pending_ = 0;
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
        if (finished_) return;
        if (max_sync_frames_ > 0 && sync_frame_count_ >= max_sync_frames_) {
            finished_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Reached max_sync_frames=%d, stopping export.", max_sync_frames_);
            rclcpp::shutdown();
            return;
        }

        rclcpp::Time lidar_time = lidar->header.stamp;
        auto inspva = find_nearest_inspva(lidar_time);
        auto ks = find_nearest_kinematic_state(lidar_time);
        if (!inspva) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No inspva message available for synced frame yet.");
        }
        if (!ks) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No kinematic_state message available for synced frame yet.");
        }
        std::array<sensor_msgs::msg::CompressedImage::ConstSharedPtr, kNumCameras> imgs{
            cam0, cam1, cam2, cam3, cam4, cam5};
        for (size_t i = 0; i < kNumCameras; ++i) {
            save_image(imgs[i], i, lidar_time);
        }
        save_cloud(lidar);
        if (ks) {
            save_pose(ks, inspva, lidar_time);
        }
        ++sync_frame_count_;
        RCLCPP_INFO(this->get_logger(), "Saved synced frame %d/%d at ts=%s",
                    sync_frame_count_, max_sync_frames_ > 0 ? max_sync_frames_ : sync_frame_count_,
                    generate_filename(lidar_time, "").c_str());
    }

    std::string output_dir_;
    std::string calib_dir_;
    std::vector<CameraEntry> cameras_;
    int max_sync_frames_{0};
    int sync_frame_count_{0};
    bool finished_{false};
    bool enable_undistort_{true};

    std::array<cv::Mat, kNumCameras> K_;
    std::array<cv::Mat, kNumCameras> D_;
    std::array<cv::Mat, kNumCameras> map1_;
    std::array<cv::Mat, kNumCameras> map2_;
    std::array<bool, kNumCameras> maps_initialized_{};

    std::array<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>, kNumCameras> cam_subs_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Subscription<autoware_localization_msgs::msg::KinematicState>::SharedPtr ks_sub_;
    rclcpp::Subscription<beidou_ins_driver::msg::Inspva>::SharedPtr inspva_sub_;

    std::deque<autoware_localization_msgs::msg::KinematicState::ConstSharedPtr> ks_buffer_;
    std::mutex ks_mutex_;
    std::deque<beidou_ins_driver::msg::Inspva::ConstSharedPtr> inspva_buffer_;
    std::mutex inspva_mutex_;

    sqlite3 *ins_db_{nullptr};
    sqlite3_stmt *odom_stmt_{nullptr};
    sqlite3_stmt *accel_stmt_{nullptr};
    bool in_transaction_{false};
    int ins_pending_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<SyncExportNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        fprintf(stderr, "sync_export_node fatal: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
