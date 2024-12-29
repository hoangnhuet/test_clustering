#include <iostream>
#include "homotopy_class_planner.h"
#include "obstacles.h"
#include "pose_se2.h"
#include "teb_config.h"

using namespace teb_local_planner;

int main()
{
    // 1) Tạo TebConfig và tùy chỉnh
    TebConfig cfg;
    
    // 2) Tạo container obstacles
    ObstContainer my_obstacles;
    my_obstacles.push_back(ObstaclePtr(new PointObstacle(2.0, 2.0))); 
    my_obstacles.push_back(ObstaclePtr(new PointObstacle(3.0, 1.0)));
    
    // 3) Tạo HomotopyClassPlanner
    HomotopyClassPlanner hcp(cfg, &my_obstacles, NULL); // via_points = NULL
    
    // 4) Gọi hcp.plan() với start, goal
    PoseSE2 start(0,0,0), goal(6,3,0);
    Twist start_vel;
    start_vel.linear = 0.0;
    start_vel.angular = 0.0;
    
    // plan(...) nội bộ sẽ gọi exploreEquivalenceClassesAndInitTebs(), 
    // -> graph_search_->createGraph() -> DFS -> addAndInitNewTeb()...
    hcp.plan(start, goal, &start_vel, false);
    
    // 5) Kiểm tra kết quả
    // - hcp.getTrajectoryContainer() cho bạn vector TebOptimalPlannerPtr
    const auto& all_tebs = hcp.getTrajectoryContainer();
    std::cout << "Number of TEBs after plan: " << all_tebs.size() << std::endl;
    
    // - In ra đường (pose) của từng TEB
    for (size_t i=0; i<all_tebs.size(); ++i)
    {
        TebOptimalPlannerPtr teb_ptr = all_tebs[i];
        std::cout << " TEB " << i << " n_poses = " << teb_ptr->teb().sizePoses() << std::endl;
        
        // In chi tiết các pose
        for (int p=0; p<teb_ptr->teb().sizePoses(); ++p)
        {
            auto &pose_vtx = teb_ptr->teb().Pose(p);
            std::cout << "   (" << pose_vtx.x() << ", " << pose_vtx.y() << ")\n";
        }
    }

    return 0;
}
