#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

struct EIGEN_ALIGN16 PointXYZIRTRGBL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    std::uint16_t ring;
    float time;
    PCL_ADD_RGB
    std::uint16_t label;
    PCL_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} ;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTRGBL,
                                (float, x, x) (float, y, y) (float, z, z) 
                                (float, intensity, intensity)
                                (std::uint16_t, ring, ring)
                                (float, time, time)
                                (std::uint32_t, rgb, rgb)
                                (std::uint16_t, label, label))


bool m_cfg_b_exclude_ground;
int m_cfg_i_start_frame;
int m_cfg_i_end_frame;

int total_TP = 0;
int total_TN = 0;
int total_FP = 0;
int total_FN = 0;

std::ofstream m_file_output;

std::vector<std::pair<int,double>> m_scan_IoU_storage;
std::vector<std::pair<int,double>> m_scan_accuracy_storage;
std::vector<std::pair<int,double>> m_scan_precision_storage;
std::vector<std::pair<int,double>> m_scan_recall_storage;
std::vector<std::pair<int,double>> m_scan_PR_storage;
std::vector<std::pair<int,double>> m_scan_RR_storage;
std::vector<std::pair<int,double>> m_scan_score_storage;
std::vector<std::pair<int,int>> m_scan_TP_storage;
std::vector<std::pair<int,int>> m_scan_TN_storage;
std::vector<std::pair<int,int>> m_scan_FP_storage; 
std::vector<std::pair<int,int>> m_scan_FN_storage;


bool m_cfg_b_use_bagfile;
std::string m_cfg_s_bag_file_name_path, m_cfg_s_pc2_topic, m_cfg_s_result_file_path;

void SaveEvaluationResult()
{
    double total_IoU = (double) total_TP / (double) (total_TP + total_FP + total_FN + FLT_MIN);
    double total_accuracy = (double) (total_TP + total_TN) / (double) (total_TP + total_TN + total_FP + total_FN + FLT_MIN);
    double total_precision = (double) (total_TP) / (double) (total_TP + total_FP + FLT_MIN);
    double total_recall = (double) (total_TP) / (double) (total_TP + total_FN + FLT_MIN);
    double total_PR = (double) total_TN / (double) (total_TN + total_FP + FLT_MIN);
    double total_RR = (double) total_TP / (double) (total_TP + total_FN + FLT_MIN);
    double total_score = (total_PR + total_RR) / 2.0; 

    m_file_output.setf(std::ios::fixed);
    m_file_output.precision(6);
    m_file_output << "frame_count - 1,scan_IoU,scan_accuracy,scan_precision,scan_recall,scan_PR,scan_RR,scan_score,scan_TP,scan_TN,scan_FP,scan_FN,total_IoU,total_accuracy,total_precision,total_recall,total_PR,total_RR,total_score,total_TP,total_TN,total_FP,total_FN\n";
    for(int i = 0; i < (int)m_scan_IoU_storage.size(); i++)
    {
        if(i == 0)
        {
            m_file_output << m_scan_IoU_storage[i].first << "," << m_scan_IoU_storage[i].second << "," << m_scan_accuracy_storage[i].second << "," 
                << m_scan_precision_storage[i].second << "," << m_scan_recall_storage[i].second << "," << m_scan_PR_storage[i].second << "," 
                << m_scan_RR_storage[i].second << "," << m_scan_score_storage[i].second << "," << m_scan_TP_storage[i].second << "," 
                << m_scan_TN_storage[i].second << "," << m_scan_FP_storage[i].second << "," << m_scan_FN_storage[i].second << "," 
                << total_IoU << "," << total_accuracy << "," << total_precision << "," 
                << total_recall << "," << total_PR << "," << total_RR << "," 
                << total_score << "," << total_TP << "," << total_TN << "," 
                << total_FP << "," << total_FN << "\n";
        }
        else
        {
            m_file_output << m_scan_IoU_storage[i].first << "," << m_scan_IoU_storage[i].second << "," << m_scan_accuracy_storage[i].second << "," 
                << m_scan_precision_storage[i].second << "," << m_scan_recall_storage[i].second << "," << m_scan_PR_storage[i].second << "," 
                << m_scan_RR_storage[i].second << "," << m_scan_score_storage[i].second << "," << m_scan_TP_storage[i].second << "," 
                << m_scan_TN_storage[i].second << "," << m_scan_FP_storage[i].second << "," << m_scan_FN_storage[i].second << "\n"; 
        }
    }
    
    std::cout << std::fixed << std::setprecision(4);
    std::cout <<"total_IoU: " << total_IoU <<", total_accuracy: " << total_accuracy << ", total_precision: " << total_precision << ", total_recall: " << total_recall 
                << ", total_PR: " << total_PR << ", total_RR: " << total_RR << ", total_score: " << total_score 
                << ", \ntotal_TP: " << total_TP << ", total_TN: " << total_TN << ", total_FP: " << total_FP << ", total_FN: " << total_FN << "\n";
}

void EvaluateMOS(sensor_msgs::PointCloud2 msg)
{
    /* 
    Index List
    - 0~1 : not evaluate
    - 9~99 : static
    - 251~259 : moving
    */

    pcl::PointCloud<PointXYZIRTRGBL>::Ptr mos_result_pcptr;
    mos_result_pcptr.reset(new pcl::PointCloud<PointXYZIRTRGBL>);
    pcl::fromROSMsg(msg, *mos_result_pcptr);

    int num_of_point = mos_result_pcptr->points.size();

    if(num_of_point <= 0)
    {
        std::cout << "point cloud size 0\n";
        return;
    }

    static int frame_count = 0;
    frame_count++;

    int scan_TP = 0;
    int scan_TN = 0;
    int scan_FP = 0;
    int scan_FN = 0;
    
    for(int idx = 0; idx < (int)mos_result_pcptr->points.size(); idx++)
    {
		// Motion belief update
		int static_belief = mos_result_pcptr->points[idx].r;
		int moving_belief = mos_result_pcptr->points[idx].g;
		int unknown_belief = mos_result_pcptr->points[idx].b;

        bool predict_is_dynamic;
		if (moving_belief > static_belief && moving_belief > unknown_belief) // dynamic filter
		{
			predict_is_dynamic = true;
		}
        else
        {
            predict_is_dynamic = false;
        }

        int gt_label = mos_result_pcptr->points[idx].label;
        bool gt_is_dynamic;
        if(gt_label >= 0 && gt_label <= 1) // unlabeled
        {
            continue;
        }
        else if(gt_label == 40 && m_cfg_b_exclude_ground == true)
        {
            continue;
        }
        else if(gt_label >= 9 && gt_label <= 99) // static
        {
            gt_is_dynamic = false;
        }
        else if(gt_label >= 251 && gt_label <= 259)
        {
            gt_is_dynamic = true;
        }
        else
        {
            continue;
        }

        if(gt_is_dynamic == true && predict_is_dynamic == true)
        {
            scan_TP++;
        }
        else if(gt_is_dynamic == false && predict_is_dynamic == false)
        {
            scan_TN++;
        }
        else if(gt_is_dynamic == true && predict_is_dynamic == false)
        {
            scan_FN++;
        }
        else if(gt_is_dynamic == false && predict_is_dynamic == true)
        {
            scan_FP++;
        }
    }    

    total_TP += scan_TP;
    total_TN += scan_TN;
    total_FP += scan_FP;
    total_FN += scan_FN;

    double scan_IoU = (double) (scan_TP + 1) / (double) (scan_TP + scan_FP + scan_FN + 1);
    double scan_accuracy = (double) (scan_TP + scan_TN + 1) / (double) (scan_TP + scan_TN + scan_FP + scan_FN + 1);
    double scan_precision = (double) (scan_TP + 1) / (double) (scan_TP + scan_FP + 1);
    double scan_recall = (double) (scan_TP + 1) / (double) (scan_TP + scan_FN + 1);
    double scan_PR = (double) (scan_TN + 1) / (double) (scan_TN + scan_FP + 1);
    double scan_RR = (double) (scan_TP + 1) / (double) (scan_TP + scan_FN + 1);
    double scan_score = (scan_PR + scan_RR) / 2.0; 


    m_scan_IoU_storage.push_back(std::make_pair(frame_count - 1, scan_IoU));
    m_scan_accuracy_storage.push_back(std::make_pair(frame_count - 1, scan_accuracy));
    m_scan_precision_storage.push_back(std::make_pair(frame_count - 1, scan_precision));
    m_scan_recall_storage.push_back(std::make_pair(frame_count - 1, scan_recall));
    m_scan_PR_storage.push_back(std::make_pair(frame_count - 1, scan_PR));
    m_scan_RR_storage.push_back(std::make_pair(frame_count - 1, scan_RR));
    m_scan_score_storage.push_back(std::make_pair(frame_count - 1, scan_score));

    m_scan_TP_storage.push_back(std::make_pair(frame_count - 1, scan_TP));
    m_scan_TN_storage.push_back(std::make_pair(frame_count - 1, scan_TN));
    m_scan_FP_storage.push_back(std::make_pair(frame_count - 1, scan_FP));
    m_scan_FN_storage.push_back(std::make_pair(frame_count - 1, scan_FN));

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[EvaluateMOS] frame_count: " << frame_count << ", frame_count - 1: " << frame_count - 1  << ", scan_IoU: " << scan_IoU << ", PR: " << scan_PR << ", RR: " <<  scan_RR 
                << ", score: " << scan_score << ", accuracy: " << scan_accuracy << ", precision: " << scan_precision << ", recall: " << scan_recall << "\n";


}

// void SemanticKittiPCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {

// }

void MosResultPCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    EvaluateMOS(*msg);
}


bool parse_bag(std::string i_s_bag_file_path, std::string i_s_pc2_topic)
{

    rosbag::Bag bag(i_s_bag_file_path);
    if(bag.isOpen() == false)
    {
        std::cout << "Fail to open bag file : " << i_s_bag_file_path << "\n";
        return false;        
    }

    rosbag::View view(bag, rosbag::TopicQuery(i_s_pc2_topic));
    int count = -1;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view)
    {
        sensor_msgs::PointCloud2::ConstPtr instance = msg.instantiate<sensor_msgs::PointCloud2>();
        if (instance != NULL)
        {
            count++;
            if(count < m_cfg_i_start_frame)
            {
                continue;
            }
            else if(m_cfg_i_end_frame > 0 && count > m_cfg_i_end_frame)
            {
                break;
            }
            EvaluateMOS(*instance);
        }
    }
    bag.close();
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mos_evaluation");
    std::cout << "evaluation start!\n";

    ros::NodeHandle nhPrivate("~");

    if(!nhPrivate.getParam("/mos_evaluation/m_cfg_b_exclude_ground", m_cfg_b_exclude_ground)){{ROS_WARN("Fail to get param - m_cfg_b_exclude_ground");}};
    if(!nhPrivate.getParam("/mos_evaluation/m_cfg_b_use_bagfile", m_cfg_b_use_bagfile)){{ROS_WARN("Fail to get param - m_cfg_b_use_bagfile");}};
    if(!nhPrivate.getParam("/mos_evaluation/m_cfg_i_start_frame", m_cfg_i_start_frame)){{ROS_WARN("Fail to get param - m_cfg_i_start_frame");}};
    if(!nhPrivate.getParam("/mos_evaluation/m_cfg_i_end_frame", m_cfg_i_end_frame)){{ROS_WARN("Fail to get param - m_cfg_i_end_frame");}};
    if(!nhPrivate.getParam("/mos_evaluation/m_cfg_s_bag_file_name_path", m_cfg_s_bag_file_name_path)){{ROS_WARN("Fail to get param - m_cfg_s_bag_file_name_path");}};
    if(!nhPrivate.getParam("/mos_evaluation/m_cfg_s_pc2_topic", m_cfg_s_pc2_topic)){{ROS_WARN("Fail to get param - m_cfg_s_pc2_topic");}};
    if(!nhPrivate.getParam("/mos_evaluation/m_cfg_s_result_file_path", m_cfg_s_result_file_path)){{ROS_WARN("Fail to get param - m_cfg_s_result_file_path");}};

    ros::NodeHandle nh;

    ros::Subscriber mos_result_pc_sub;

    if(m_cfg_b_use_bagfile == true)
    {
        parse_bag(m_cfg_s_bag_file_name_path, m_cfg_s_pc2_topic);
    }
    else
    {
        mos_result_pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(m_cfg_s_pc2_topic, 1, &MosResultPCCallback);
        ros::spin();
    }

    std::string bag_file_name_full, bag_file_name;

    const char pathSeparator = '/';
    size_t separatorPos = m_cfg_s_bag_file_name_path.find_last_of(pathSeparator);
    if(separatorPos == std::string::npos)
        bag_file_name_full = m_cfg_s_bag_file_name_path;
    else
        bag_file_name_full = m_cfg_s_bag_file_name_path.substr(separatorPos + 1);

    const char extensionSeparator = '.';
    size_t extensionPos = bag_file_name_full.find_last_of(extensionSeparator);
    if (extensionPos == std::string::npos)
        bag_file_name = bag_file_name_full;
    else
        bag_file_name = bag_file_name_full.substr(0, extensionPos);

    std::string file_name = bag_file_name + "_mos_evaluation.csv";
    m_file_output.open(m_cfg_s_result_file_path + file_name);

    if (m_file_output.is_open() == false)
    {
        std::cout << "fail to make result file. Path : " << m_cfg_s_result_file_path + file_name << "\n";
        return 0;
    }

    SaveEvaluationResult();
    
    m_file_output.close();
    std::cout << "file name: " << file_name << "\n";

    return 0;
}
