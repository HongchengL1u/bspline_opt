#include "bspline/non_uniform_bspline.h"
#include "matplotlibcpp.h"
#include "bspline_opt/include/bspline_optimizer.h"
#include "path_searching/include/kinodynamic_astar.h"
#include "sdf/include/signed_distance_field_2d.h"
#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>

namespace plt = matplotlibcpp;

std::vector<fast_planner::BsplineOptimizer::Ptr> bspline_optimizers_;
std::unique_ptr<fast_planner::KinodynamicAstar> kino_path_finder_;


// void getTrajByDuration(double start_t, double duration, int seg_num,
//                          vector<Eigen::Vector3d>& point_set,
//                          vector<Eigen::Vector3d>& start_end_derivative, double& dt) 
// {
//     dt = duration / seg_num;
//     // Eigen::Vector3d cur_pt;
//     // for (double tp = 0.0; tp <= duration + 1e-4; tp += dt) {
//     //   cur_pt = getPosition(start_t + tp);
//     //   point_set.push_back(cur_pt);
//     // }
//     Eigen::Vector3d a = Eigen::Vector3d({1,1,0});
//     for (double tp = 0.0; tp <= duration+ 1e-4; tp += dt)
//     {   
//         point_set.push_back(0.5*a*tp*tp);
//     }
    
//     start_end_derivative.push_back(a*0);
//     start_end_derivative.push_back(a*duration);
//     start_end_derivative.push_back(a);
//     start_end_derivative.push_back(a);
// }

// void reparamBspline(NonUniformBspline& bspline, double ratio,
//                                         Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
//   int    prev_num    = bspline.getControlPoint().rows();
//   double time_origin = bspline.getTimeSum();
//   int    seg_num     = bspline.getControlPoint().rows() - 3;
//   // double length = bspline.getLength(0.1);
//   // int seg_num = ceil(length / pp_.ctrl_pt_dist);

//   ratio = min(1.01, ratio);
//   bspline.lengthenTime(ratio);
//   double duration = bspline.getTimeSum();
//   dt              = duration / double(seg_num);
//   time_inc        = duration - time_origin;

//   vector<Eigen::Vector3d> point_set;
//   for (double time = 0.0; time <= duration + 1e-4; time += dt) {
//     point_set.push_back(bspline.evaluateDeBoorT(time));
//   }
//   bspline.parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
//                                            ctrl_pts);
//   // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
// }

// void refineTraj(fast_planner::NonUniformBspline& best_traj, double& time_inc) {
//     //   ros::Time t1 = ros::Time::now();
//     time_inc = 0.0;

//     // TODO dt??
//     double    dt = 0.05, t_inc;
//     // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
//     Eigen::MatrixXd ctrl_pts = best_traj.getControlPoint();
    
//     int cost_function = fast_planner::BsplineOptimizer::NORMAL_PHASE;


//     // TODO: 
//     // max_vel_ = 10;
//     // max_acc_ = 3;
//     // best_traj.setPhysicalLimits(max_vel_, max_acc_);
//     // double ratio = best_traj.checkRatio();
//     // std::cout << "ratio: " << ratio << std::endl;
//     // reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
//     // time_inc += t_inc;


//     ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
//     best_traj = fast_planner::NonUniformBspline(ctrl_pts, 3, dt);
// }


// Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration) {
//     /* get the sample points local traj within radius */

//     vector<Eigen::Vector3d> point_set;
//     vector<Eigen::Vector3d> start_end_derivative;
//     int seg_num = 20;

//     getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
//     // getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
//     //                             start_end_derivative, dt, duration);
//     /* parameterization of B-spline */
//     Eigen::MatrixXd ctrl_pts;
//     // std::cout << point_set.size() << std::endl;
//     fast_planner::NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
//     // plan_data_.local_start_end_derivative_ = start_end_derivative;
//     // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

//     return ctrl_pts;
// }



int main()
{
    planning::GridMap2D<uint8_t> grid_map;
    std::array<int, 2> size{800, 800};
    grid_map.set_cell_number(size);
    grid_map.set_origin({0.0, 0.0});
    grid_map.set_resolution({0.1, 0.1});

    // grid_map.FillCircle(Eigen::Vector2d(10, 10), 5);
    vector_Eigen<Eigen::Vector2d> rectangular({Eigen::Vector2d(30, 20), Eigen::Vector2d(40, 20), Eigen::Vector2d(40, 35), Eigen::Vector2d(30, 35)});
    grid_map.FillConvexPoly(rectangular);


    std::shared_ptr<planning::SignedDistanceField2D> sdf_map_ptr = std::make_shared<planning::SignedDistanceField2D>(std::move(grid_map));
    sdf_map_ptr->UpdateSDF();
    auto esdf = sdf_map_ptr->esdf();
    // cout << esdf.Matrix() << endl;

    
    bool use_kinodynamic_path = true;
    if (use_kinodynamic_path) {
        kino_path_finder_.reset(new fast_planner::KinodynamicAstar);
        kino_path_finder_->setParam();
        kino_path_finder_->setSdfmap(sdf_map_ptr);
        kino_path_finder_->init();
    }
    kino_path_finder_->reset();
    Eigen::Vector3d start_pt = {1,1,0};
    Eigen::Vector3d start_vel = {0,0,0};
    Eigen::Vector3d start_acc = {0,0,0};
    Eigen::Vector3d end_pt = {60,60,0};
    Eigen::Vector3d end_vel = {0,0,0};
    int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == fast_planner::KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == fast_planner::KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

    } else {
        cout << "[kino replan]: kinodynamic search success." << endl;
    }

    std::vector<Eigen::Vector3d> kino_result = kino_path_finder_->getKinoTraj(0.1);
    
    double ctrl_pt_dist = 1;
    double max_vel_ = 10;
    double ts = ctrl_pt_dist / max_vel_;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);
    Eigen::MatrixXd ctrl_pts;
    // for(auto el:kino_result) std::cout << el << std::endl;
    // for(auto el:point_set) std::cout << el << std::endl;

    // 这一步相当于对A*的结果进行拟合，A*结果包括一个总共时间长度，比如12
    // 首先会确定每一段的时间跨度，比如这里是0.1，那么会有120个段
    // 但是这121个点只有120个有acc和vel
    // 所以一共有120个点可用，这样就有119个段，有119个knots加上头尾，各一个p的knots，这样就会的到125个knots
    // 则有122个控制点
    // 对于这120个点，有120个位置约束和头尾点的速度和加速度约束


    // 位置约束：
    // 根据下文Bspline的矩阵表达式，此处仅仅使用了一阶的位置，也就是M4的第一行
    // 速度约束和加速度约束也只是位置约束的完全表达式，也就是用了第二行和第三行再差分得到

    // K+2个控制点
    // https://blog.csdn.net/weixin_42284263/article/details/119204964
    fast_planner::NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    
    // double dt, duration=1;
    // Eigen::MatrixXd ctrl_pts = reparamLocalTraj(0.0, dt, duration);
    // fast_planner::NonUniformBspline bspline(ctrl_pts, 3, dt);
    // std::cout << ctrl_pts.rows() << std::endl;
    
    fast_planner::NonUniformBspline init(ctrl_pts, 3, ts);
    bool use_optimization = true;
    if (use_optimization) 
    {
        bspline_optimizers_.resize(1);
        for (int i = 0; i < bspline_optimizers_.size(); ++i) 
        {
            bspline_optimizers_[i].reset(new fast_planner::BsplineOptimizer);
            bspline_optimizers_[i]->setParam();
            bspline_optimizers_[i]->setSdfptr(sdf_map_ptr);
        }
    }

    int cost_function = fast_planner::BsplineOptimizer::NORMAL_PHASE;

    if (status != fast_planner::KinodynamicAstar::REACH_END) {
        cost_function |= fast_planner::BsplineOptimizer::ENDPOINT;
    }

    ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

    fast_planner::NonUniformBspline res = fast_planner::NonUniformBspline(ctrl_pts, 3, ts);
    double to = res.getTimeSum();
    res.setPhysicalLimits(10, 5);
    bool feasible = res.checkFeasibility(true);

    int iter_num = 0;
    while (!feasible) {
        feasible = res.reallocateTime();
        if (++iter_num >= 3)
        {
            std::cerr<<"no feasible" << std::endl;
            break;
        }
    }

    cout << "[Main]: iter num: " << iter_num << endl;

    double tn = res.getTimeSum();

    cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
    if (tn / to > 3.0) std::cerr<<"reallocate error."<<std::endl;

    // double time_inc = 0;
    // refineTraj(bspline, time_inc);
    
    std::vector<Eigen::VectorXd> control_points;
    // std::cout << "control_points: " << std::endl;
    for(int i=0;i<ctrl_pts.rows();i++)
    {
        // std::cout << ctrl_pts.row(i).transpose() << std::endl;
        control_points.push_back(ctrl_pts.row(i));
    }

    std::vector<Eigen::VectorXd> init_points;
    double du = 0.01;
    for(double u=0;u<=8+1e-4;u+=du)
    {
        init_points.push_back(init.evaluateDeBoor(u));
    }

    std::vector<Eigen::VectorXd> result_points;
    for(double u=0;u<=8+1e-4;u+=du)
    {
        result_points.push_back(res.evaluateDeBoor(u));
    }

    std::vector<double> control_x;
    std::vector<double> control_y;
    for(auto el:control_points)
    {
        control_x.push_back(el(0));
        control_y.push_back(el(1));
    }
    std::vector<double> init_x;
    std::vector<double> init_y;
    for(auto el:init_points)
    {
        init_x.push_back(el(0));
        init_y.push_back(el(1));
    }

    std::vector<double> result_x;
    std::vector<double> result_y;
    for(auto el:result_points)
    {
        result_x.push_back(el(0));
        result_y.push_back(el(1));
    }
    std::vector<double> kino_result_x;
    std::vector<double> kino_result_y;
    for(auto el:kino_result)
    {
        kino_result_x.push_back(el(0));
        kino_result_y.push_back(el(1));
    }
    

    plt::set_aspect(1);
    plt::scatter(control_x,control_y);
    plt::plot(kino_result_x,kino_result_y,{{"color", "orange"}});
    plt::plot(init_x,init_y,{{"color", "green"}});
    plt::plot(result_x,result_y,{{"color", "red"}});

    std::vector<double> rec_x;
    std::vector<double> rec_y;
    for(auto el:rectangular)
    {   
        rec_x.push_back(el(0));
        rec_y.push_back(el(1));
    }
    rec_x.push_back(rectangular[0](0));
    rec_y.push_back(rectangular[0](1));
    plt::plot(rec_x,rec_y,{{"color", "purple"}});
    plt::save("result.png",300);
    plt::show();
    
}