#include <iostream>
#include "homotopy_class_planner.h"
#include "obstacles.h"
#include "pose_se2.h"
#include "teb_config.h"

using namespace teb_local_planner;
inline cv::Point scalePoint(double x, double y, double scale, double offset_x, double offset_y)
{
    int px = static_cast<int>(x * scale + offset_x);
    int py = static_cast<int>(y * scale + offset_y);
    // Nếu muốn trục y ngược, có thể: py = height - py;
    return cv::Point(px, py);
}

int main()
{
    // (1) Kịch bản lập kế hoạch như bạn đã có:
    TebConfig cfg;
    ObstContainer my_obstacles;
    my_obstacles.push_back(ObstaclePtr(new PointObstacle(2.0, 2.0))); 
    my_obstacles.push_back(ObstaclePtr(new PointObstacle(3.0, 1.0)));
    my_obstacles.push_back(ObstaclePtr(new PointObstacle(3.0, 3.0)));
    // my_obstacles.push_back(ObstaclePtr(new LineObstacle(Eigen::Vector2d(0.0,0.0), Eigen::Vector2d(5.0, 5.0))));

    HomotopyClassPlanner hcp(cfg, &my_obstacles, nullptr);
    PoseSE2 start(0,0,0), goal(6,3,0);
    Twist start_vel{1.0, 0.2};
    hcp.plan(start, goal, &start_vel, false);

    // (2) Tạo ảnh trắng 500x500
    cv::Mat map(1500, 1500, CV_8UC3, cv::Scalar(255,255,255));

    double scale = 50.0;  // ví dụ 1m -> 50 pixel
    double offset_x = 50; // dời sang phải 50 px
    double offset_y = 400;// dời origin xuống (tuỳ bạn)

    // (3) Vẽ obstacle
    for(const auto& obst : my_obstacles)
    {
        // Lấy vị trí obstacle (PointObstacle => getCentroid)
        Eigen::Vector2d c = obst->getCentroid();
        cv::Point pc = scalePoint(c.x(), c.y(), scale, offset_x, offset_y);
        cv::circle(map, pc, 5, cv::Scalar(0,0,255), -1); 
        // vẽ chấm đỏ
    }

    // (4) Vẽ path TEB
    const auto& all_tebs = hcp.getTrajectoryContainer();
    // std::cout<<all_tebs.size()<<std::endl;
    // std::cout<<hcp.getVelocityCommand(start_vel, all_tebs.size())<<std::endl;
    //print out predicted velocity command based on start_vel
    Twist velocity;
    hcp.getVelocityCommand(velocity, all_tebs.size());

    std::cout<<"Predicted velocity command based on start_vel: "<<velocity.linear<<" "<<velocity.angular<<std::endl;
    int idx=0;
    for (auto& planner : all_tebs)
    {
        auto &teb = planner->teb();
        // Tạo vector<Point> để vẽ polyline
        std::vector<cv::Point> path_points;
        path_points.reserve(teb.sizePoses());
        for(int p=0; p<teb.sizePoses(); ++p)
        {
            auto &pose = teb.Pose(p);
            double x = pose.x();
            double y = pose.y();
            path_points.push_back(scalePoint(x, y, scale, offset_x, offset_y));
        }
        // random màu theo idx
        cv::Scalar color( (50*idx) % 255, (100*idx)%255, (150*idx)%255 );
        idx++;

        // Vẽ polyline
        if(path_points.size()>1)
        {
            cv::polylines(map, path_points, false, color, 2);
        }
        // Vẽ start -> chấm to
        if(!path_points.empty())
        {
            cv::circle(map, path_points.front(), 6, color, -1);
            cv::putText(map, "Start", path_points.front(), cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
            cv::circle(map, path_points.back(), 6, color, -1);
            cv::putText(map, "Goal", path_points.back(), cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
        }
    }

    // (5) Hiển thị
    cv::imshow("TEB Paths", map);
    cv::waitKey(0);

    return 0;
}