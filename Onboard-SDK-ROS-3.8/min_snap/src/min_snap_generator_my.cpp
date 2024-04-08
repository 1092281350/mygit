#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "min_snap/min_snap_closeform.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include "dji_sdk/dji_sdk.h"
#include <std_msgs/UInt8.h>

ros::Publisher goal_list_pub;
ros::Publisher poly_coef_pub;
ros::Subscriber rviz_goal_sub;
ros::Subscriber odom_sub;
ros::Subscriber state_sub;
ros::Subscriber local_pos_sub;
ros::Subscriber next_pharse_sub;
ros::Publisher rviz_goal_pub;
ros::Publisher last_point_pub;
ros::Publisher cmdStatusPub;

bool mys=false;
int id = 0;
double mean_vel = 1;
const int GOAL_HEIGHT = 2;
int b=-1;
bool complete=false;
bool w =false;
const int row=204;
const int rw=123;
const int rt=85;
const int a[7]={0,3,8,3,4,5,2};
const int ac[7]={0,3,11,14,18,23,25};
const float gh[25][2]={{-4.6354,-1.1727},{-4.3903,1.2768},{-1.8848,1.3558},{-1.1497,-0.5307},{-0.4204,-0.6146},{0.1980,-1.4452},{0.6225,-0.4964},{0.9417,-0.0921},{0.7014,0.3674},{0.4272,0.6326},{0.0769,0.7363},{5.4777,-1.3707},{5.1236,1.0470},{1.8386,1.1371},{1.5207,4.7301},{-1.1182,5.0345},{-3.2949,3.6038},{-5.7683,1.5824},{-0.5704,-0.0049},{4.7090,-3.1625},{2.1907,-4.3879},{0.0315,-3.7493},{-0.5479,-1.2541},{-2.3595,-0.6330},{-0.6963,0.0407}};
// const float s[row][2]={{0.1948,0.1277},{0.0026,0.1116},{-0.0883,0.3267},{-0.1896,0.0954},{-0.5687,0.2863},{-0.5661,0.3979},{-0.3713,0.5256},{-0.2700,0.7569},{-0.1688,0.9881},{0.0286,1.2274},{0.0546,2.3432},{0.1507,2.3513},{0.1429,2.0165},{0.3298,1.8095},{0.4129,1.2597},{0.4051,0.9249},{0.3999,0.7018},{0.5895,0.6063},{0.7817,0.6225},{1.0673,0.5351},{0.9712,0.5271},{0.7817,0.6225},{0.3947,0.4786},{0.3012,0.5821},{0.2078,0.6856},{0.1221,1.1239},{0.0156,0.6695},{-0.1818,0.4302},{-0.3739,0.4140},{-0.5687,0.2863},{-0.5713,0.1747},{-0.5791,-0.1600},{-0.3869,-0.1439},{-0.2934,-0.2474},{-0.3012,-0.5821},{-0.5999,-1.0527},{-1.0024,-1.8660},{-0.7116,-1.7302},{-0.6077,-1.3874},{-0.2986,-0.4705},{-0.2960,-0.3590},{-0.2882,-0.0242},{-0.3817,0.0793},{-0.7583,0.3817},{-0.2856,0.0874},{-0.1948,-0.1277},{-0.1948,-0.1277},{-0.1117,-0.6776},{-0.2208,-1.2435},{-0.0104,-0.4463},{0.0883,-0.3267},{0.2830,-0.1989},{0.4726,-0.2944},{0.4752,-0.1828},{0.1870,-0.2070},{-0.0078,-0.3347},{-0.1351,-1.6818},{-0.0416,-1.7853},{0.2440,-1.8727},{-0.1039,-0.3428},{-0.0987,-0.1197},{-0.4856,-0.2635},{-0.6752,-0.1681},{-0.2934,-0.2474},{-0.1013,-0.2312},{-0.1065,-0.4544},{-0.0364,-1.5621},{-0.0026,-0.1116},{0.0935,-0.1035},{0.4804,0.0404},{0.7713,0.1762},{0.3843,0.0323},{0.0909,-0.2151},{-0.0104,-0.4463},{0.0805,-0.6614},{0.2700,-0.7569},{0.2752,-0.5337},{0.4674,-0.5175},{0.6648,-0.2782},{0.9713,0.1747},{-0.7635,0.5107},{-0.5661,0.3979},{-0.8101,2.2706},{0.0832,3.5706},{0.1507,2.3513},{0.1429,2.0165},{0.3298,1.8095},{0.8180,2.1846},{0.9894,1.3081},{0.7817,0.6225},{1.0699,0.2946},{-0.3403,-2.2558},{-0.7064,-1.5071},{-0.5194,-1.7141},{-0.2286,-1.5783},{-0.1221,-1.1239},{-0.0208,-0.8927},{-0.0338,-1.4506},{-0.0312,-1.3390},{0.2622,-1.0916},{0.5531,-0.9558},{0.3739,-0.4140},{-0.2052,-0.5741},{-0.6934,-0.9492},{-0.9686,-0.4155},{-0.8749,-0.1666},{3.7081,2.4091},{0.8751,0.5190},{0.7791,0.5109},{1.2777,1.3323},{0.6986,1.1723},{0.6285,2.2801},{0.0468,2.0085},{-0.7193,2.0555},{-2.2153,3.3456},{-1.1296,0.9074},{-0.9530,0.2540},{1.7246,-1.5915},{0.8673,6.1843},{1.5559,13.2715},{2.7526,-1.2164},{2.1034,-0.2687},{0.4570,-0.9639},{-0.5090,-1.2678},{-0.9530,0.2540},{-0.4752,0.1828},{-0.3792,0.1908},{-0.4700,0.4059},{-0.7504,0.7164},{-1.3087,1.4491},{-0.4674,0.5175},{-0.6310,1.7288},{-0.2700,0.7569},{-0.0883,0.3267},{-0.0909,0.2151},{-1.1219,1.2420},{-0.6622,0.3898},{-0.4778,0.0712},{-1.0674,-0.5351},{-0.8051,-1.6267},{-0.5012,-0.9330},{-0.2986,-0.4705},{-1.0906,-1.5393},{-0.6882,-0.7260},{-0.3922,-0.3671},{-0.3584,1.0835},{-0.3610,0.9720},{-0.8519,0.4851},{-0.2883,-0.0243},{-0.0416,-1.7854},{-0.0182,-0.7811},{0.0805,-0.6614},{-0.0312,-1.3389},{-0.0546,-2.3432},{-0.0208,-0.8927},{0.0597,-1.5541},{0.1688,-0.9881},{0.2701,-0.7569},{0.2779,-0.4221},{0.6545,-0.7245},{0.8544,-0.3736},{1.7216,-0.1895},{1.0568,0.0888},{0.2908,0.1358},{0.7712,0.1761},{0.1948,0.1277},{0.8804,0.7421},{0.9842,1.0850},{0.2934,0.2474},{0.6960,1.0608},{0.2052,0.5741},{0.1403,1.9049},{-0.6387,1.3941},{-0.1818,0.4302},{0.0364,1.5623},{0.2052,0.5741},{0.2909,0.1359},{0.3532,-1.3067},{0.5584,-0.7326},{0.2804,-0.3105},{0.5583,-0.7326},{0.2830,-0.1989},{0.8519,-0.4852},{0.1922,0.0161},{0.7686,0.0645},{1.0620,0.3119},{0.1922,0.0161},{0.4883,0.3751},{0.2987,0.4707},{0.0987,0.1197},{-0.1870,0.2070},{-7.4423,5.7224},{-1.5347,-0.0176},{-0.6440,1.1709},{0.8361,-1.1547},{-5.2820,-4.4529},{0.4440,-1.5218},{1.6204,-0.4207},{2.1034,-0.2687},{-0.5194,-1.7141},{-3.7811,-5.8858},{0.9322,-1.1467},{1.5321,-0.0940},{0.8933,1.3001}};
//乌龟----123
const float wg[rw][2]={{0.1948,0.1277},{0.0026,0.1116},{-0.0883,0.3267},{-0.1896,0.0954},{-0.5687,0.2863},{-0.5661,0.3979},{-0.3713,0.5256},{-0.2700,0.7569},{-0.1688,0.9881},{0.0286,1.2274},{0.0546,2.3432},{0.1507,2.3513},{0.1429,2.0165},{0.3298,1.8095},{0.4129,1.2597},{0.4051,0.9249},{0.3999,0.7018},{0.5895,0.6063},{0.7817,0.6225},{1.0673,0.5351},{0.9712,0.5271},{0.7817,0.6225},{0.3947,0.4786},{0.3012,0.5821},{0.2078,0.6856},{0.1221,1.1239},{0.0156,0.6695},{-0.1818,0.4302},{-0.3739,0.4140},{-0.5687,0.2863},{-0.5713,0.1747},{-0.5791,-0.1600},{-0.3869,-0.1439},{-0.2934,-0.2474},{-0.3012,-0.5821},{-0.5999,-1.0527},{-1.0024,-1.8660},{-0.7116,-1.7302},{-0.6077,-1.3874},{-0.2986,-0.4705},{-0.2960,-0.3590},{-0.2882,-0.0242},{-0.3817,0.0793},{-0.7583,0.3817},{-0.2856,0.0874},{-0.1948,-0.1277},{-0.1948,-0.1277},{-0.1117,-0.6776},{-0.2208,-1.2435},{-0.0104,-0.4463},{0.0883,-0.3267},{0.2830,-0.1989},{0.4726,-0.2944},{0.4752,-0.1828},{0.1870,-0.2070},{-0.0078,-0.3347},{-0.1351,-1.6818},{-0.0416,-1.7853},{0.2440,-1.8727},{-0.1039,-0.3428},{-0.0987,-0.1197},{-0.4856,-0.2635},{-0.6752,-0.1681},{-0.2934,-0.2474},{-0.1013,-0.2312},{-0.1065,-0.4544},{-0.0364,-1.5621},{-0.0026,-0.1116},{0.0935,-0.1035},{0.4804,0.0404},{0.7713,0.1762},{0.3843,0.0323},{0.0909,-0.2151},{-0.0104,-0.4463},{0.0805,-0.6614},{0.2700,-0.7569},{0.2752,-0.5337},{0.4674,-0.5175},{0.6648,-0.2782},{0.9713,0.1747},{-0.7635,0.5107},{-0.5661,0.3979},{-0.8101,2.2706},{0.0832,3.5706},{0.1507,2.3513},{0.1429,2.0165},{0.3298,1.8095},{0.8180,2.1846},{0.9894,1.3081},{0.7817,0.6225},{1.0699,0.2946},{-0.3403,-2.2558},{-0.7064,-1.5071},{-0.5194,-1.7141},{-0.2286,-1.5783},{-0.1221,-1.1239},{-0.0208,-0.8927},{-0.0338,-1.4506},{-0.0312,-1.3390},{0.2622,-1.0916},{0.5531,-0.9558},{0.3739,-0.4140},{-0.2052,-0.5741},{-0.6934,-0.9492},{-0.9686,-0.4155},{-0.8749,-0.1666},{3.7081,2.4091},{0.8751,0.5190},{0.7791,0.5109},{1.2777,1.3323},{0.6986,1.1723},{0.6285,2.2801},{0.0468,2.0085},{-0.7193,2.0555},{-1.2153,1.3456},{-1.1296,0.9074},{-0.9530,0.2540},{0.7246,6.4085},{0.4856,0.2636},{0.3817,-0.0793},{-0.3013,-0.5821},{-0.5660,0.3979},{0.5740,-0.0632}};

//兔子----85
// const float tz[rt][2]={{2.7526,-1.2164},{2.1034,-0.2687},{0.4570,-0.9639},{-0.5090,-1.2678},{-0.9530,0.2540},{-0.4752,0.1828},{-0.3792,0.1908},{-0.4700,0.4059},{-0.7504,0.7164},{-1.3087,1.4491},{-0.4674,0.5175},{-0.6310,1.7288},{-0.2700,0.7569},{-0.0883,0.3267},{-0.0909,0.2151},{-1.1219,1.2420},{-0.6622,0.3898},{-0.4778,0.0712},{-1.0674,-0.5351},{-0.8051,-1.6267},{-0.5012,-0.9330},{-0.2986,-0.4705},{-1.0906,-1.5393},{-0.6882,-0.7260},{-0.3922,-0.3671},{-0.3584,1.0835},{-0.3610,0.9720},{-0.8519,0.4851},{-0.2883,-0.0243},{-0.0416,-1.7854},{-0.0182,-0.7811},{0.0805,-0.6614},{-0.0312,-1.3389},{-0.0546,-2.3432},{-0.0208,-0.8927},{0.0597,-1.5541},{0.1688,-0.9881},{0.2701,-0.7569},{0.2779,-0.4221},{0.6545,-0.7245},{0.8544,-0.3736},{1.7216,-0.1895},{1.0568,0.0888},{0.2908,0.1358},{0.7712,0.1761},{0.1948,0.1277},{0.8804,0.7421},{0.9842,1.0850},{0.2934,0.2474},{0.6960,1.0608},{0.2052,0.5741},{0.1403,1.9049},{-0.6387,1.3941},{-0.1818,0.4302},{0.0364,1.5623},{0.2052,0.5741},{0.2909,0.1359},{0.3532,-1.3067},{0.5584,-0.7326},{0.2804,-0.3105},{0.5583,-0.7326},{0.2830,-0.1989},{0.8519,-0.4852},{0.1922,0.0161},{0.7686,0.0645},{1.0620,0.3119},{0.1922,0.0161},{0.4883,0.3751},{0.2987,0.4707},{0.0987,0.1197},{-0.1870,0.2070},{-7.4423,5.7224},{-1.5347,-0.0176},{-0.6440,1.1709},{0.8361,-1.1547},{-5.2820,-4.4529},{0.4440,-1.5218},{1.6204,-0.4207},{2.1034,-0.2687},{-0.5194,-1.7141},{-3.7811,-5.8858},{0.9322,-1.1467},{1.5321,-0.0940},{0.8933,1.3001}};

geometry_msgs::Point last_xy;
nav_msgs::Odometry odom;
geometry_msgs::Point current_local_pos;

geometry_msgs::Pose goal_pt;
geometry_msgs::PoseArray goal_list;
my_planner::minsnapCloseform minsnap_solver;
std::vector<Eigen::Vector3d> waypoints;
quadrotor_msgs::PolynomialTrajectory poly_pub_topic;

double rsum=0;
ros::Time js;
Eigen::VectorXd seg_ts(20),res_ts(20);

// //回调函数
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }
void gps_goal_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped t;
    current_local_pos = msg->point;
    if(mys && (ros::Time::now() - js).toSec()>rsum){
        t.header.frame_id="world";
        t.header.stamp=ros::Time::now();
        t.header.seq=0;
        
        rviz_goal_pub.publish(t);
    }
}
void pub_poly_coefs()
{
    Eigen::MatrixXd poly_coef = minsnap_solver.getPolyCoef();
    Eigen::MatrixXd dec_vel = minsnap_solver.getDecVel();
    Eigen::VectorXd time = minsnap_solver.getTime();

    poly_pub_topic.num_segment = goal_list.poses.size() - 1;
    poly_pub_topic.coef_x.clear();
    poly_pub_topic.coef_y.clear();
    poly_pub_topic.coef_z.clear();
    poly_pub_topic.time.clear();
    poly_pub_topic.trajectory_id = id;

    // display decision variable
    ROS_WARN("decision variable:");
    for (int i = 0; i < goal_list.poses.size(); i++)
    {
        cout << "Point number = " << i + 1 << endl
             << dec_vel.middleRows(i * 4, 4) << endl;
    }

    for (int i = 0; i < time.size(); i++)
    {
        for (int j = (i + 1) * 8 - 1; j >= i * 8; j--)
        {
            poly_pub_topic.coef_x.push_back(poly_coef(j, 0));
            poly_pub_topic.coef_y.push_back(poly_coef(j, 1));
            poly_pub_topic.coef_z.push_back(poly_coef(j, 2));
        }
        poly_pub_topic.time.push_back(time(i));
    }

    poly_pub_topic.header.frame_id = "world";
    poly_pub_topic.header.stamp = ros::Time::now();

    poly_coef_pub.publish(poly_pub_topic);
}

void solve_min_snap()
{
    Eigen::Vector3d wp;
    waypoints.clear();
    for (int i = 0; i < int(goal_list.poses.size()); i++)
    {
        wp << goal_list.poses[i].position.x, goal_list.poses[i].position.y, goal_list.poses[i].position.z;
        waypoints.push_back(wp);
    }
    minsnap_solver.Init(waypoints, mean_vel);
    ROS_INFO("Init success");
    minsnap_solver.calMinsnap_polycoef();
    pub_poly_coefs();

}
//乌龟兔子
void rviz_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double tm=0;
    b++;
    goal_pt.position = current_local_pos;
    goal_list.poses.push_back(goal_pt);
    // goal_pt = msg->pose;
    // if (goal_pt.position.z < 0)
    // {
    //     goal_pt.position.z = GOAL_HEIGHT;
    //     goal_list.poses.push_back(goal_pt);
    //     goal_pt.position = odom.pose.pose.position;
    //     goal_list.poses.insert(goal_list.poses.begin(), goal_pt);
    //     goal_list.header.stamp = ros::Time::now();
    //     goal_list.header.frame_id = "world";
    //     goal_list.header.seq = id++;
    //     goal_list_pub.publish(goal_list);
    //     solve_min_snap();
    //     ROS_INFO("solver finished");
    //     goal_list.poses.clear();
    // }
    // else
    // {
    //     goal_pt.position.z = GOAL_HEIGHT;
    //     goal_list.poses.push_back(goal_pt);
    // }

    int tmp=20*(b+1)<rw ? 20*(b+1) : rw;
    int tmp1=20*(b+1)<rt ? 20*(b+1) : rt;
    // if(w==false){
    //     tmp=tmp1;
    // }
    for (int i = 20*b; i <tmp; i++){
            // if(w==true){
                goal_pt.position.x=goal_pt.position.x+wg[i][0];
            goal_pt.position.y=goal_pt.position.y+wg[i][1];
            // }else{
            //     goal_pt.position.x=goal_pt.position.x+tz[i][0];
            // goal_pt.position.y=goal_pt.position.y+tz[i][1];
            // }
            goal_pt.position.z = GOAL_HEIGHT;
            goal_list.poses.push_back(goal_pt);
            goal_list.header.stamp = ros::Time::now();
            goal_list.header.frame_id = "world";
            goal_list.header.seq = (id++);
    }

     goal_list_pub.publish(goal_list);
     last_point_pub.publish(goal_pt.position);//发布最后一个目标点
     solve_min_snap();
    ROS_INFO("solver finished");
    js= ros::Time::now();
    seg_ts=minsnap_solver.getTime();
     for(int k=0;k<seg_ts.size();k++){
        tm+=seg_ts(k);
     }
     rsum=tm;
    goal_list.poses.clear();
    id=0;
    std_msgs::UInt8 ms;
    ms.data=2;
    cmdStatusPub.publish(ms);
    mys=true;
}

//  //党徽
// void rviz_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     b++;
//     goal_pt.position = current_local_pos;
//     goal_list.poses.push_back(goal_pt);
//     if(b==0){
//     last_xy.x=current_local_pos.y;
//     last_xy.y=current_local_pos.x;
//     }


//     // goal_pt = msg->pose;
//     // if (goal_pt.position.z < 0)
//     // {
//     //     goal_pt.position.z = GOAL_HEIGHT;
//     //     goal_list.poses.push_back(goal_pt);
//     //     goal_pt.position = odom.pose.pose.position;
//     //     goal_list.poses.insert(goal_list.poses.begin(), goal_pt);
//     //     goal_list.header.stamp = ros::Time::now();
//     //     goal_list.header.frame_id = "world";
//     //     goal_list.header.seq = id++;
//     //     goal_list_pub.publish(goal_list);
//     //     solve_min_snap();
//     //     ROS_INFO("solver finished");
//     //     goal_list.poses.clear();
//     // }
//     // else
//     // {
//     //     goal_pt.position.z = GOAL_HEIGHT;
//     //     goal_list.poses.push_back(goal_pt);
//     // }

//     // int tmp=20*(b+1)<row ? 20*(b+1) : row;
//     for (int i = ac[b]; i <ac[b+1]; i++){
//         last_xy.x+=gh[i][1];
//         last_xy.y+=gh[i][0];
//         goal_pt.position.x=goal_pt.position.x+gh[i][0];
//             goal_pt.position.y=goal_pt.position.y+gh[i][1];
//             // ROS_INFO("X IS %f,    Y is  %f",gh[i][0],gh[i][1]);
//             goal_pt.position.z = GOAL_HEIGHT;
//             goal_list.poses.push_back(goal_pt);
//             goal_list.header.stamp = ros::Time::now();
//             goal_list.header.frame_id = "world";
//             goal_list.header.seq = (id++);
//     }

//      goal_list_pub.publish(goal_list);
//      last_point_pub.publish(last_xy);//发布最后一个目标点
//      solve_min_snap();
//     ROS_INFO("solver finished");
//     goal_list.poses.clear();
//     id=0;
//     std_msgs::UInt8 ms;
//     ms.data=2;
//     cmdStatusPub.publish(ms);
// }

// void rviz_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
    
//     goal_pt = msg->pose;
    
//     if (goal_pt.position.z < 0)
//     {
//         goal_pt.position.z = GOAL_HEIGHT;
//         goal_list.poses.push_back(goal_pt);
//         // goal_pt.position = odom.pose.pose.position;
//         goal_pt.position = current_local_pos;
//         goal_list.poses.insert(goal_list.poses.begin(), goal_pt);
//         goal_list.header.stamp = ros::Time::now();
//         goal_list.header.frame_id = "world";
//         goal_list.header.seq = id++;
//         goal_list_pub.publish(goal_list);
//         solve_min_snap();
//         ROS_INFO("solver finished");
//         goal_list.poses.clear();
//         id=0;
//         std_msgs::UInt8 ms;
//         ms.data=2;
//         cmdStatusPub.publish(ms);
//     }
//     else
//     {
//         goal_pt.position.z = GOAL_HEIGHT;
//         goal_list.poses.push_back(goal_pt);
//     }
// }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_snap_generator");
    ros::NodeHandle nh("~");

    ros::Rate rate(10);
    ros::param::get("wugui",w);
    // 【订阅】里程计（此处注意话题名称替换）
    // odom_sub = nh.subscribe("/odom_topic", 10, odom_goal_cb);
    local_pos_sub=nh.subscribe("/local_position", 10, gps_goal_cb);

    rviz_goal_pub=nh.advertise<geometry_msgs::PoseStamped>("/rviz_goal", 10);
    // 【订阅】RVIZ目标点
    rviz_goal_sub = nh.subscribe("/rviz_goal", 10, rviz_goal_cb);
    // next_pharse_sub = nh.subscribe("/next_pharse", 1, next_pharse);
    cmdStatusPub = nh.advertise<std_msgs::UInt8> ("/control_status",1);

    // 【发布】目标点，用于目标点显示
    goal_list_pub = nh.advertise<geometry_msgs::PoseArray>("/goal_list", 10);
    // 发布多项式轨迹
    poly_coef_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_coefs", 10);

    // 发布最后一次目标点
    last_point_pub=nh.advertise<geometry_msgs::Point>("/last_point",1);

    poly_pub_topic.num_order = 7;
    poly_pub_topic.start_yaw = 0;
    poly_pub_topic.final_yaw = 0;
    poly_pub_topic.mag_coeff = 0;
    poly_pub_topic.order.push_back(0);

    //  ros::Timer mainloop_timer = nh.createTimer(ros::Duration(0.5), mainloop);
    
    ros::param::get("/min_snap_generator/meanvel", mean_vel);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

