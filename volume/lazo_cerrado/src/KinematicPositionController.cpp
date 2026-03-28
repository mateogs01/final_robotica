#include <angles/angles.h>
#include "KinematicPositionController.h"

#include <cmath>

/**
 * NOTA: Para un sistema estable mantener:
 * - 0 < K_RHO
 * - K_RHO < K_ALPHA
 * - K_BETA < 0
 */

/*
K_RHO = 0.2, 0.4, 0.8
K_alpha = 0.5, 1, 1.5
LOOKAHEAD = 0.2, 0.5, 1

exp1 = .5,1,.5

exp2 = .2,1,.5
exp3 = .9,1,.5

exp4 = .5,.6,.5
exp5 = .5,1.4,.5

exp6 = .5,1,1
exp7 = .5,1,.2
*/

#define K_RHO .5
#define K_ALPHA 1
#define LOOKAHEAD 1

KinematicPositionController::KinematicPositionController() :
  TrajectoryFollower(), tfBuffer_(this->get_clock()),transform_listener_( tfBuffer_ )
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(50));
    qos_profile.reliable();
    qos_profile.durability_volatile();

    expected_position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10));
    
    std::string odometry_selection = this->declare_parameter("odometry_selection", "ODOMETRY"); // "EKF", "ODOMETRY"
    
    if (odometry_selection == "ODOMETRY")
        current_pos_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10), std::bind(&KinematicPositionController::getCurrentPoseFromOdometry, this, std::placeholders::_1));
    else if (odometry_selection == "EKF")
        current_pos_ekf_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", rclcpp::QoS(10), std::bind(&KinematicPositionController::getCurrentPoseFromEKF, this, std::placeholders::_1));
          
    std::string goal_selection = this->declare_parameter("goal_selection", "TIME_BASED");
    fixed_goal_x_ = this->declare_parameter("fixed_goal_x", 3.0);
    fixed_goal_y_ = this->declare_parameter("fixed_goal_y", 0.0);
    fixed_goal_a_ = this->declare_parameter("fixed_goal_a", -M_PI_2);
    
    if(goal_selection == "TIME_BASED")
      goal_selection_ = TIME_BASED;
    else if(goal_selection == "PURSUIT_BASED")
      goal_selection_ = PURSUIT_BASED;
    else if(goal_selection == "FIXED_GOAL")
      goal_selection_ = FIXED_GOAL;
    else
      goal_selection_ = TIME_BASED; // default
}

double lineal_interp(const rclcpp::Time& t0, const rclcpp::Time& t1, double y0, double y1, const rclcpp::Time& t)
{
  return y0 + (t - t0).seconds() * (y1 - y0) / (t1 - t0).seconds();
}

void KinematicPositionController::getCurrentPoseFromOdometry(const nav_msgs::msg::Odometry& odometry_msg)
{
  x = odometry_msg.pose.pose.position.x;
  y = odometry_msg.pose.pose.position.y;
  tf2::Quaternion q(odometry_msg.pose.pose.orientation.x,
                    odometry_msg.pose.pose.orientation.y,
                    odometry_msg.pose.pose.orientation.z,
                    odometry_msg.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  a = yaw;
}

void KinematicPositionController::getCurrentPoseFromEKF(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
{
  x = pose_msg.pose.pose.position.x;
  y = pose_msg.pose.pose.position.y;
  tf2::Quaternion q(pose_msg.pose.pose.orientation.x,
                    pose_msg.pose.pose.orientation.y,
                    pose_msg.pose.pose.orientation.z,
                    pose_msg.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  a = yaw;
}


bool KinematicPositionController::control(const rclcpp::Time& t, double& vx, double& vy, double& w)
{
  // Se obtiene la pose actual publicada por la odometria
  double current_x, current_y, current_a;
  current_x = this->x; current_y = this->y; current_a = this->a;

  // Se obtiene la pose objetivo actual a seguir
  double goal_x, goal_y, goal_a;
  if( not getCurrentGoal(t, goal_x, goal_y, goal_a) )
    return false;
  publishCurrentGoal(t, goal_x, goal_y, goal_a); // publicación de la pose objetivo para visualizar en RViz


  double dxi = goal_x - current_x;
  double dyi = goal_y - current_y;

  double dx =  cos(current_a)*dxi + sin(current_a)*dyi;
  double dy = -sin(current_a)*dxi + cos(current_a)*dyi;
  double theta = angles::normalize_angle(goal_a - current_a);

  
  // Computar variables del sistema de control
  double rho = sqrt((dx*dx) + (dy*dy));
  double alpha = angles::normalize_angle(atan2(dy, dx) - theta); // Normalizes the angle to be -M_PI circle to +M_PI circle It takes and returns radians. 
  
  if (rho>0.01) {
    vx = K_RHO * dx;
    vy = K_RHO * dy;
    w = K_ALPHA * theta;
  } else {
    vx = 0;
    vy = 0;
    w = K_ALPHA * theta;
  }
  

  RCLCPP_INFO(this->get_logger(), "atan2: %.2f, theta siegwart: %.2f, expected_atheta: %.2f, rho: %.2f, vx: %.2f, vy: %.2f, w: %.2f",
            atan2(dy, dx), theta, current_a, rho, vx, vy, w);

  RCLCPP_INFO(this->get_logger(), "goal_x: %.2f, goal_y: %.2f, goal_a: %.2f, current_x: %.2f, current_y: %.2f, current_a: %.2f",
            goal_x, goal_y, goal_a, current_x, current_y, current_a);

  return true;
}

/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1)
{ return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));}

bool KinematicPositionController::getPursuitBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a)
{
  // Se obtienen los valores de la posicion y orientacion actual.
  double current_x, current_y, current_a;
  current_x = this->x; current_y = this->y; current_a = this->a;
    
  // Se obtiene la trayectoria requerida.
  const robmovil_msgs::msg::Trajectory& trajectory = getTrajectory();
  
  double lookahead = LOOKAHEAD;
  double wpoint_x = 0;
  double wpoint_y = 0;
  double wpoint_a = 0;
  double min_dist = -1;
  int id_wpoint_actual = -1;

  /* NOTA: De esta manera les es posible recorrer la trayectoria requerida */  
  for(int i = 0; i < trajectory.points.size(); i++)
  {
    // Recorren cada waypoint definido
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[i];
    
    // Y de esta manera puede acceder a la informacion de la posicion y orientacion requerida en el waypoint
    wpoint_x = wpoint.transform.translation.x;
    wpoint_y = wpoint.transform.translation.y;
    wpoint_a = tf2::getYaw(wpoint.transform.rotation);

    double dist = dist2(current_x, current_y, wpoint_x, wpoint_y);
    if (dist < min_dist || min_dist == -1) {
      min_dist = dist;
      id_wpoint_actual = i;
    }
  }
  
  if (id_wpoint_actual == trajectory.points.size()-1) {
    const robmovil_msgs::msg::TrajectoryPoint& last_wpoint = trajectory.points.back();
    x = last_wpoint.transform.translation.x;
    y = last_wpoint.transform.translation.y;
    a = tf2::getYaw(last_wpoint.transform.rotation);
    return false;
  }

  for(int i = id_wpoint_actual+1; i < trajectory.points.size(); i++)
  {
    // Recorren cada waypoint definido
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[i];
    
    // Y de esta manera puede acceder a la informacion de la posicion y orientacion requerida en el waypoint
    wpoint_x = wpoint.transform.translation.x;
    wpoint_y = wpoint.transform.translation.y;
    wpoint_a = tf2::getYaw(wpoint.transform.rotation);

    double dist = dist2(current_x, current_y, wpoint_x, wpoint_y);
    if (dist > lookahead) {
      break;
    }
  }
    
  x = wpoint_x;
  y = wpoint_y;
  a = wpoint_a;
  
  /* retorna true si es posible definir un goal, false si se termino la trayectoria y no quedan goals. */
  return true;
}

bool KinematicPositionController::getTimeBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a)
{
  size_t next_point_idx;

  if( not nextPointIndex(t, next_point_idx ) )
    return false;
    
  RCLCPP_INFO(this->get_logger(), "processing index: %zu", next_point_idx);

  const robmovil_msgs::msg::TrajectoryPoint& prev_point = getTrajectory().points[ next_point_idx-1 ];
  const robmovil_msgs::msg::TrajectoryPoint& next_point = getTrajectory().points[ next_point_idx ];

  const rclcpp::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const rclcpp::Time& t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);

  double x0 = prev_point.transform.translation.x;
  double x1 = next_point.transform.translation.x;

  double y0 = prev_point.transform.translation.y;
  double y1 = next_point.transform.translation.y;

  double a0 = tf2::getYaw(prev_point.transform.rotation);
  double a1 = tf2::getYaw(next_point.transform.rotation);

  x = lineal_interp(t0, t1, x0, x1, t);
  y = lineal_interp(t0, t1, y0, y1, t);
  a = lineal_interp(t0, t1, a0, a1, t);

  return true;
}
