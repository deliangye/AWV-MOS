#include "AWV_MOS.h"
#include <filesystem>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

bool ReadCalibFile(const std::string& i_file_path, Eigen::Affine3f& o_tf_lidar_to_base) {
    std::ifstream calib_file(i_file_path);
    if (!calib_file.is_open()) {
        std::cerr << "Could not open calib file: " << i_file_path << std::endl;
        return false;
    }

    Eigen::Affine3f tf_lidar_to_base;
    std::string line;
    while (std::getline(calib_file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (!(iss >> key)) continue;

        std::vector<float> values((std::istream_iterator<float>(iss)), std::istream_iterator<float>());
        if (values.size() != 12) continue;

        Eigen::Matrix4f matrix;
        matrix << values[0], values[1], values[2], values[3],
                  values[4], values[5], values[6], values[7],
                  values[8], values[9], values[10], values[11],
                  0, 0, 0, 1;

        tf_lidar_to_base = Eigen::Affine3f(matrix);
    }
    o_tf_lidar_to_base = tf_lidar_to_base;

    calib_file.close();

    std::cout << "Read calibration file done. \n";
    return true;
}

bool ReadPosesFile(const std::string& i_file_path, const Eigen::Affine3f& i_tf_lidar_to_base, std::vector<Eigen::Affine3f>& o_poses) 
{
    std::ifstream pose_file(i_file_path);
    if (!pose_file.is_open()) {
        std::cerr << "Could not open poses file: " << i_file_path << std::endl;
        return false;
    }

    std::vector<Eigen::Affine3f> poses;
    Eigen::Affine3f tf_cam_coordi_system_to_world_coordi_system = i_tf_lidar_to_base.inverse();
    std::string line;
    while (std::getline(pose_file, line)) {
        std::istringstream iss(line);
        std::vector<float> values;
        float value;
        while (iss >> value) {
            values.push_back(value);
        }

        // 각 포즈 데이터는 12개의 값으로 구성되어 있다고 가정합니다 (3x4 행렬).
        if (values.size() != 12) {
            std::cerr << "Incorrect format in line: " << line << std::endl;
            continue;
        }

        Eigen::Matrix4f matrix;
        matrix << values[0], values[1], values[2], values[3],
                  values[4], values[5], values[6], values[7],
                  values[8], values[9], values[10], values[11],
                  0, 0, 0, 1;

        Eigen::Affine3f tf_base_to_origin = Eigen::Affine3f(matrix);

        poses.push_back(tf_cam_coordi_system_to_world_coordi_system * tf_base_to_origin * i_tf_lidar_to_base);
    }
    pose_file.close();
    o_poses = poses;

    std::cout << "Read poses file done. poses size: " << o_poses.size() << "\n";

    return true;
}

int extract_number(const std::string& file_path) {
    // Get file name
    size_t last_slash_idx = file_path.find_last_of("/\\");
    size_t expender_idx = file_path.find_last_of(".");
    std::string file_name;
    if (last_slash_idx != std::string::npos && expender_idx != std::string::npos)
    {
        file_name = file_path.substr(last_slash_idx + 1, expender_idx - last_slash_idx - 1);
        return std::stoi(file_name);
    }
    else
        return -1;
}

bool GetScanFileList(const std::string& i_folder_path, std::vector<std::string>& o_file_list)
{
    std::vector<std::string> file_list;

    if (std::filesystem::exists(i_folder_path) == false || 
        std::filesystem::is_directory(i_folder_path) == false)
    {
        std::cout << "Scan file folder does not exist or is not a directory: " << i_folder_path << std::endl;
        return false;
    }
        
    for (const auto& entry : std::filesystem::directory_iterator(i_folder_path)) 
    {
        if (entry.is_regular_file()) 
        {  // 파일만 선택 (디렉토리 제외)
            file_list.push_back(entry.path().string());
        }
    }
 
    // 파일 리스트 정렬
    std::sort(file_list.begin(), file_list.end(), 
            [](const std::string& a, const std::string& b) 
            {return extract_number(a) < extract_number(b);});
    
    o_file_list = file_list;

    std::cout << "Get scan file list done. list size: " << o_file_list.size() << "\n";
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "start\n";
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    ros::init(argc, argv, "run_static_mapping");
    ros::NodeHandle nh;
    std::string m_cfg_s_output_pc_namespace;
    std::string scan_folder_path, poses_file_path, calib_file_path;
    nh.param<std::string>("awv_mos/m_cfg_s_output_pc_namespace", m_cfg_s_output_pc_namespace, "/awv_mos/mos_pc");
    if (!nh.getParam("scan_folder_path", scan_folder_path)){ROS_WARN("Fail to get param - scan_folder_path");}
    if (!nh.getParam("poses_file_path", poses_file_path)){ROS_WARN("Fail to get param - poses_file_path");}
    if (!nh.getParam("calib_file_path", calib_file_path)){ROS_WARN("Fail to get param - calib_file_path");}

    ros::Publisher pub_segmented_new_scan_all = nh.advertise<sensor_msgs::PointCloud2>(m_cfg_s_output_pc_namespace + std::string("/segmented_new_scan_all"), 100);
    ros::Publisher pub_segmented_new_scan_static = nh.advertise<sensor_msgs::PointCloud2>(m_cfg_s_output_pc_namespace + std::string("/segmented_new_scan_static"), 100);
    ros::Publisher pub_segmented_new_scan_dynamic = nh.advertise<sensor_msgs::PointCloud2>(m_cfg_s_output_pc_namespace + std::string("/segmented_new_scan_dynamic"), 100);
    tf::TransformBroadcaster br;

    pcl::PointCloud<PointTypeMOS>::Ptr scan(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
    pcl::PointCloud<PointTypeMOS>::Ptr segmented_scan;

    AWV_MOS awv_mos;
    awv_mos.SaveConfigParams();
    Eigen::Affine3f tf_lidar_to_base;
    if(calib_file_path != "")
        ReadCalibFile(calib_file_path, tf_lidar_to_base);
    else
        tf_lidar_to_base = Eigen::Affine3f::Identity();

    // Read poses.txt
    std::vector<Eigen::Affine3f> poses;
    ReadPosesFile(poses_file_path, tf_lidar_to_base, poses);
    
    // Get scan file list
    std::vector<std::string> scan_files;
    GetScanFileList(scan_folder_path, scan_files);

    // // Compute keyframes
    // // Todo
    // // 그냥 poses, scan_files 입력해서 실행하는 걸로
    // // mapping 옵션에 use only keyframe 설정 추가
    // // frame_idx 입력받아서 reference frame 설정
    // static int prev_close_backward_keyframe_id = 0;
    // if(i_pres_frame_id == m_keyframe_idx_list[prev_close_backward_keyframe_id])
    // {
    //     start_id = prev_close_backward_keyframe_id + 1;
    //     last_id = std::max(prev_close_backward_keyframe_id + m_ref_size, m_keyframe_idx_list.size())
    //     if(backward_ref_buff.size() == 0)
    //     {
    //         for(int keyframe_id = start_id; keyframe_id <= last_id; keyframe_id++)
    //         {

    //         }
    //     }
    //     backward_ref_buff.pop_front();

    // }
    // if(backward_ref_buff[m_cfg_ref_size])



    // // Runtime check variables
    // static std::chrono::duration<double, std::milli> duration;
    // static std::chrono::high_resolution_clock clock;
    // static std::chrono::time_point<std::chrono::high_resolution_clock> total_start_time;
    // static std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    // static int tmp_count = 0;
    // tmp_count++;
    // static double sum_total_time_ms = 0;
    // double total_time_ms = 0;

    // // Scene time file

    // ros::Rate loop_rate(100);
    // while (ros::ok())
    // {
    //     loop_rate.sleep();
    //     ros::spinOnce();

    //     std::lock_guard<std::mutex> lock(mtx);
    //     // std::cout << "[debug] scan size: " << scan_msg_queue.size() << ", odom size: " << odom_msg_queue.size() << "\n";
    //     while (!scan_msg_queue.empty() && !odom_msg_queue.empty()) {
    //         sensor_msgs::PointCloud2 scan_msg = scan_msg_queue.front();
    //         nav_msgs::Odometry odom_msg = odom_msg_queue.front();

    //         double scan_time_s = scan_msg.header.stamp.toSec();
    //         double odom_time_s = odom_msg.header.stamp.toSec();

    //         static std::string prediction_write_folder_path = "";
    //         if(prediction_write_folder_path == "")
    //         {
    //             int scene_idx = findClosestIndex(start_timestamps, scan_time_s);
    //             std::string scene_name = scene_names[scene_idx];

    //             prediction_write_folder_path = awv_mos.m_cfg_s_prediction_write_path + scene_name + "/";
    //             std::cout << "Set prediction write path: " << prediction_write_folder_path << "\n";

    //             if (!std::filesystem::exists(prediction_write_folder_path)) 
    //             {
    //                 std::filesystem::create_directories(prediction_write_folder_path);
    //                 std::cout << "Create prediction write path folder" << std::endl;
    //             }
    //         }

    //         double time_diff_s = scan_time_s - odom_time_s;
    //         if (fabs(time_diff_s) <= 0.025) 
    //         {
    //             Eigen::Affine3f tf_frame_to_map = Eigen::Affine3f::Identity();
    //             tf_frame_to_map.translation() << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
    //             Eigen::Quaternionf quat(odom_msg.pose.pose.orientation.w, 
    //                                     odom_msg.pose.pose.orientation.x, 
    //                                     odom_msg.pose.pose.orientation.y, 
    //                                     odom_msg.pose.pose.orientation.z);
    //             tf_frame_to_map.rotate(quat);

    //             float tmp_x, tmp_y, tmp_z, tmp_roll, tmp_pitch, tmp_yaw;
    //             pcl::getTranslationAndEulerAngles (tf_frame_to_map, tmp_x, tmp_y, tmp_z, tmp_roll, tmp_pitch, tmp_yaw);
    //             auto q = tf::createQuaternionMsgFromRollPitchYaw(tmp_roll, tmp_pitch, tmp_yaw);

    //             geometry_msgs::TransformStamped transformStamped;
    //             transformStamped.header.stamp = odom_msg.header.stamp;
    //             transformStamped.header.frame_id = "map";
    //             transformStamped.child_frame_id = scan_msg.header.frame_id;
    //             transformStamped.transform.translation.x = tf_frame_to_map.translation().x();
    //             transformStamped.transform.translation.y = tf_frame_to_map.translation().y();
    //             transformStamped.transform.translation.z = tf_frame_to_map.translation().z();
    //             transformStamped.transform.rotation.x = q.x;
    //             transformStamped.transform.rotation.y = q.y;
    //             transformStamped.transform.rotation.z = q.z;
    //             transformStamped.transform.rotation.w = q.w;
    //             br.sendTransform(transformStamped);

    //             pcl::fromROSMsg(scan_msg, *scan);
                
    //             total_start_time = clock.now();

    //             double time_s = scan_msg.header.stamp.toSec();
    //             bool is_keyframe = awv_mos.KeyframeSelection(tf_frame_to_map, time_s);
    //             static int frame_count = 0;
    //             awv_mos.RunOnlineMOS(scan, tf_frame_to_map, ++frame_count-1, time_s, is_keyframe);
    //             awv_mos.GetSegmentedScan(segmented_scan);

    //             if(awv_mos.m_cfg_b_write_prediction == true)
    //             {
    //                 std::ostringstream oss;
    //                 oss << std::setw(6) << std::setfill('0') << frame_count-1 << ".label";
    //                 std::string prediction_file_name = oss.str();
    //                 std::string prediction_write_file_name = prediction_write_folder_path + prediction_file_name;
    //                 awv_mos.WritePrediction(segmented_scan, prediction_write_file_name);
    //             }

    //             duration = std::chrono::high_resolution_clock::now() - total_start_time;
    //             total_time_ms = duration.count();
    //             sum_total_time_ms += total_time_ms;

    //             std::cout << frame_count - 1 << " [MOS Module Ave Time] Total average time ms: " << sum_total_time_ms / frame_count << "\n"; 

    //             sensor_msgs::PointCloud2 segmented_new_scan_all_all;
    //             pcl::toROSMsg(*segmented_scan, segmented_new_scan_all_all);
    //             segmented_new_scan_all_all.header = scan_msg.header;
    //             pub_segmented_new_scan_all.publish(segmented_new_scan_all_all);

    //             pcl::PointCloud<PointTypeMOS>::Ptr segmented_new_scan_static(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
    //             pcl::PointCloud<PointTypeMOS>::Ptr segmented_new_scan_dynamic(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
    //             for (const auto& point : segmented_scan->points) 
    //             {
    //                 if(point.r >= point.g || point.b >= point.g) // Filtering non-dynamic points
    //                     segmented_new_scan_static->points.push_back(point);
    //                 else
    //                     segmented_new_scan_dynamic->points.push_back(point);
    //             }
    //             sensor_msgs::PointCloud2 segmented_new_scan_static_msg, segmented_new_scan_dynamic_msg;
    //             pcl::toROSMsg(*segmented_new_scan_static, segmented_new_scan_static_msg);
    //             pcl::toROSMsg(*segmented_new_scan_dynamic, segmented_new_scan_dynamic_msg);
    //             segmented_new_scan_static_msg.header = scan_msg.header;
    //             segmented_new_scan_dynamic_msg.header = scan_msg.header;
    //             pub_segmented_new_scan_static.publish(segmented_new_scan_static_msg);
    //             pub_segmented_new_scan_dynamic.publish(segmented_new_scan_dynamic_msg);
                
    //             scan_msg_queue.pop();
    //             odom_msg_queue.pop();
    //         } 
    //         else 
    //         {
    //             if (scan_time_s < odom_time_s) 
    //             {
    //                 ROS_WARN("older scan data: %f", time_diff_s);
    //                 scan_msg_queue.pop();
    //             } 
    //             else 
    //             {
    //                 ROS_WARN("older odom data: %f", time_diff_s);
    //                 odom_msg_queue.pop();
    //             }
    //         }
    //     }
    // }

    

    return 0;
}