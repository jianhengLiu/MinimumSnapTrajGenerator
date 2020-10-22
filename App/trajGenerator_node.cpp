#include "trajGenerator/trajectory_generator_waypoint.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <vector>
#include <fstream>

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajGenerator_node");
    ros::NodeHandle nh;
    ros::Rate rate(1000);

    std::vector<TrajectoryGeneratorWaypoint> vect_Traj;

    visualization_msgs::Marker _traj_vis;
    {
        _traj_vis.header.stamp = ros::Time::now();
        _traj_vis.header.frame_id = "map";

        _traj_vis.ns = "traj_node/trajectory_waypoints";
        _traj_vis.id = 0;
        _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        _traj_vis.action = visualization_msgs::Marker::ADD;
        _traj_vis.scale.x = 0.2;
        _traj_vis.scale.y = 0.2;
        _traj_vis.scale.z = 0.2;
        _traj_vis.pose.orientation.x = 0.0;
        _traj_vis.pose.orientation.y = 0.0;
        _traj_vis.pose.orientation.z = 0.0;
        _traj_vis.pose.orientation.w = 1.0;

        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 1.0;
        _traj_vis.color.g = 0.0;
        _traj_vis.color.b = 0.0;

        _traj_vis.points.clear();
    }

    visualization_msgs::Marker line_list;
    {
        line_list.header.frame_id = "map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "wp_path";
        line_list.id = 0;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.pose.orientation.x = 0.0;
        line_list.pose.orientation.y = 0.0;
        line_list.pose.orientation.z = 0.0;

        line_list.type = visualization_msgs::Marker::LINE_STRIP;

        line_list.scale.x = 0.15;
        line_list.scale.y = 0.15;
        line_list.scale.z = 0.15;
        line_list.color.a = 1.0;

        line_list.color.r = 0.0;
        line_list.color.g = 1.0;
        line_list.color.b = 0.0;

        line_list.points.clear();
    }

    {
        std::string config_file = "/home/chrisliu/src/Controller/main_controller/config/default.yaml";

        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        cv::Mat readMat;
        fs["waypointMatrix"] >> readMat;
        Eigen::MatrixXd readMat_e(9, 4);
        cv::cv2eigen(readMat, readMat_e);
        for (int i = 0; i < readMat_e.rows() - 1; i++) {
            TrajectoryGeneratorWaypoint TrajectoryGenerator(0.5,0.8);

            nav_msgs::Path waypoints;
            waypoints.header.frame_id = "map";
            waypoints.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped temp_wpt;
            temp_wpt.header.frame_id = "map";
            temp_wpt.pose.orientation.w = 1;
            temp_wpt.pose.orientation.z = 0;
            temp_wpt.pose.orientation.y = 0;
            temp_wpt.pose.orientation.x = 0;

            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = readMat_e(i, 0);
            temp_wpt.pose.position.y = readMat_e(i, 1);
            temp_wpt.pose.position.z = readMat_e(i, 2);
            waypoints.poses.push_back(temp_wpt);

            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = readMat_e(i + 1, 0);
            temp_wpt.pose.position.y = readMat_e(i + 1, 1);
            temp_wpt.pose.position.z = readMat_e(i + 1, 2);
            waypoints.poses.push_back(temp_wpt);

            Eigen::MatrixXd margin_constraint(2,6);
            margin_constraint = readMat_e.block(i,3,2,6);

            TrajectoryGenerator.isTraj = TrajectoryGenerator.trajGeneration(waypoints,margin_constraint);
            std::cout << "_waypoints" << std::endl;
            std::cout << TrajectoryGenerator._waypoints << std::endl;
            if (TrajectoryGenerator.isTraj) {
                for(int i =0;i<TrajectoryGenerator.visWayPointPath().points.size();i++)
                {
                    line_list.points.push_back(TrajectoryGenerator.visWayPointPath().points.at(i));
                }
                for(int i =0;i<TrajectoryGenerator.visWayPointTraj().points.size();i++)
                {
                    _traj_vis.points.push_back(TrajectoryGenerator.visWayPointTraj().points.at(i));
                }
                vect_Traj.push_back(TrajectoryGenerator);
            }
        }
        if(!vect_Traj.empty())
        {
            vect_Traj[0]._wp_path_vis_pub.publish(line_list);
            vect_Traj[0]._wp_traj_vis_pub.publish(_traj_vis);
        }
    }


    ofstream fout_pos("/home/chrisliu/GitRespository/MinimumSnapTrajGenerator/traj_data/trajectory_pos.txt");
    ofstream fout_vel("/home/chrisliu/GitRespository/MinimumSnapTrajGenerator/traj_data/trajectory_vel.txt");

    double startTime = 0;
    int cnt_traj = 0;
    while(ros::ok()){
        if(vect_Traj.empty())
        {
        } else
        {
            if(startTime == 0)
            {
                startTime = ros::Time::now().toSec();
                fout_pos << "double traj_pos_"<<cnt_traj+1<<"[] = {" << endl;
                fout_vel << "double traj_vel_"<<cnt_traj+1<<"[] = {" << endl;
            }
            double t = ros::Time::now().toSec()-startTime;
            if(vect_Traj[cnt_traj]._totalTime>t)
            {
                Eigen::Vector3d inputDesiredPos = vect_Traj[cnt_traj].getTrajectoryStates(t,0);
                Eigen::Vector3d inputDesiredVel = vect_Traj[cnt_traj].getTrajectoryStates(t,1);
                Eigen::Vector3d inputDesiredAcc = vect_Traj[cnt_traj].getTrajectoryStates(t,2);
                Eigen::Vector3d inputDesiredJerk = vect_Traj[cnt_traj].getTrajectoryStates(t,3);
                Eigen::Vector3d inputDesiredSnap = vect_Traj[cnt_traj].getTrajectoryStates(t,4);
                fout_pos << inputDesiredPos.x() << "," << endl;
                fout_vel << inputDesiredVel.x() << "," << endl;
            }
            else
            {
                if(cnt_traj+1<vect_Traj.size())
                {
                    startTime += vect_Traj[cnt_traj]._totalTime;
                    cnt_traj++;
                    fout_pos << "};"<< endl;
                    fout_pos << "double traj_pos_"<<cnt_traj+1<<"[] = {" << endl;
                    fout_vel << "};"<< endl;
                    fout_vel << "double traj_vel_"<<cnt_traj+1<<"[] = {" << endl;
                }else
                {
                    cout<<"保存完毕!"<<endl;
                    break;
                }

            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

