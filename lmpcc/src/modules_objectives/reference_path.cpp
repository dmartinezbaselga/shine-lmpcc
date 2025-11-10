#include "modules_objectives/reference_path.h"

#include <lmpcc_solver/SolverInclude.h>
#include <lmpcc/lmpcc_configuration.h>

ReferencePath::ReferencePath(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle)
    : ControllerModule(nh, config, vehicle)
{
  LMPCC_WARN_ALWAYS("Initializing Reference Path");

  type_ = ModuleType::OBJECTIVE;

  // Reference Path
  ros_markers_reference_path_.reset(new RosTools::ROSMarkerPublisher(nh, config_->reference_path_topic_.c_str(), config_->target_frame_, 45));
  ros_markers_reference_arrows_.reset(new RosTools::ROSMarkerPublisher(nh, config_->reference_arrows_topic_.c_str(), config_->target_frame_, 30));
  ros_markers_splineindex.reset(new RosTools::ROSMarkerPublisher(nh, config_->spline_index_topic_.c_str(), config_->target_frame_, 5));
  ros_markers_linearboundaries.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/linear_road_constraints", config_->target_frame_, 100));
  ros_markers_road_limits.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/road_limits", config_->target_frame_, 300));

  // MPCC path variables
  x_.resize(config_->ref_x_.size());
  y_.resize(config_->ref_y_.size());
  psi_.resize(config_->ref_psi_.size());
  waypoints_size_ = config_->ref_x_.size();

  // Read the reference from the config
  ReadReferencePath();

  // Initialize the path from file
  InitPath();

  LMPCC_WARN_ALWAYS("Reference Path Initialized");
}

bool ReferencePath::ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data)
{
  return waypoints_size_ > 0;
}

bool ReferencePath::ObjectiveReached(SolverInterface *solver_interface, const RealTimeData &data)
{
  return goal_reached_ && !first_run_;
}

void ReferencePath::Update(SolverInterface *solver_interface, RealTimeData &data)
{
  PROFILE_AND_LOG(config_->debug_output_, "ReferencePath::Update");
  first_run_ = false;
  segments = solver_interface->n_segments_;

  /** Update the point on the reference to track */
  minimal_s_ = solver_interface->spline(0);
  UpdateClosestPoint(&(*solver_interface), minimal_s_, window_size_, n_search_points_);

  // Check if the end of the path was reached
  if (ReachedEnd())
  {
    if (!goal_reached_)
    {
      LMPCC_INFO("Reached the end of the reference path!");
      goal_reached_ = true;
    }
  }

  if (EndOfCurrentSpline(minimal_s_) && !goal_reached_)
    spline_index_++;
  /* --------------------------------
  Adjust solver inputs:
      - xinit: initial state and spline
      - x0: only first entries corresponding to current vehicle state */
  solver_interface->setInitialSpline(minimal_s_);
  solver_interface->spline(0) = minimal_s_;
  /* ----------------------------- */

  // Construct linearized constraints from the spline if enabled
  if (config_->halfspaces_from_spline_)
    ConstructRoadConstraints(&(*solver_interface), data.halfspaces_);
}

void ReferencePath::OnReset(SolverInterface *solver_interface)
{
  // Reinitializes the spline (Note: closest point on the spline will be found in the state callback when state_received
  // == false)
  InitPath();
  solver_interface->setInitialSpline(minimal_s_);

  /** @todo FIND CLOSEST POINT HERE */
  UpdateClosestPoint(&(*solver_interface), minimal_s_, window_size_, n_search_points_);
  goal_reached_ = false;

  first_run_ = true;
}

void ReferencePath::OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name)
{
  if (data_name == "Waypoints")
  {
    LMPCC_WARN("Reference Path: Received Reference Path");

    if (config_->enable_clothoid_interpolation_)
      InitPath(data.path_.Get().x_, data.path_.Get().y_, data.path_.Get().psi_); // With clothoid, we also use the orientation
    else
      InitPath(data.path_.Get().x_, data.path_.Get().y_);
  }
  else if (data_name == "State")
    InitializeClosestPoint(solver_interface);
}

void ReferencePath::Visualize()
{
  // PROFILE_AND_LOG(config_->debug_output_,"Updating Visuals");
  VisualizeRoad();

  PublishReferencePath();
}

/* Initialize a path from x,y,psi in the class */
void ReferencePath::InitPath()
{
  spline_index_ = 0;

  // Construct a spline through these points
  ConstructReferencePath(x_, y_, psi_);

  // Visualizes the given reference points (for debug mostly)
  PublishReferencePath();
}

/* Initialize a path from x,y,psi given */
void ReferencePath::InitPath(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &psi)
{

  // Save x,y, psi
  psi_ = psi;

  // Initialize using these
  InitPath(x, y);
}

void ReferencePath::InitPath(const std::vector<double> &x, const std::vector<double> &y)
{
  x_ = x;
  y_ = y;

  waypoints_size_ = x_.size();

  InitPath();
}

void ReferencePath::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int N_iter,
                                  int &param_idx)
{
  // Set contour and lag weight
  // solver_interface->setParameter(N_iter, param_idx, solver_interface->weights_.contour_);
  // solver_interface->setParameter(N_iter, param_idx, solver_interface->weights_.lag_);

  /** @todo: Handling of parameters when the spline parameters go beyond the splines defined */
  for (int i = 0; i < solver_interface->n_segments_; i++)
  {
    // Spline x
    solver_interface->setParameter(N_iter, param_idx, ref_path_x_.m_a[spline_index_ + i]);
    solver_interface->setParameter(N_iter, param_idx, ref_path_x_.m_b[spline_index_ + i]);
    solver_interface->setParameter(N_iter, param_idx, ref_path_x_.m_c[spline_index_ + i]);
    solver_interface->setParameter(N_iter, param_idx, ref_path_x_.m_d[spline_index_ + i]);

    // Spline y
    solver_interface->setParameter(N_iter, param_idx, ref_path_y_.m_a[spline_index_ + i]);
    solver_interface->setParameter(N_iter, param_idx, ref_path_y_.m_b[spline_index_ + i]);
    solver_interface->setParameter(N_iter, param_idx, ref_path_y_.m_c[spline_index_ + i]);
    solver_interface->setParameter(N_iter, param_idx, ref_path_y_.m_d[spline_index_ + i]);

    // Distance where this spline starts
    solver_interface->setParameter(N_iter, param_idx, ss_[spline_index_ + i]); // s1
  }
}

void ReferencePath::ReadReferencePath()
{
  LMPCC_INFO_ALWAYS("Reading Reference Path");

  // Check if all reference vectors are of the same length
  assert(config_->ref_x_.size() == config_->ref_y_.size());
  assert(config_->ref_x_.size() == config_->ref_psi_.size());

  geometry_msgs::Pose pose;
  tf2::Quaternion myQuaternion;

  // Iterate over the reference points given
  for (size_t ref_point_it = 0; ref_point_it < config_->ref_x_.size(); ref_point_it++)
  {
    // Create a pose at each position
    pose.position.x = config_->ref_x_.at(ref_point_it);
    pose.position.y = config_->ref_y_.at(ref_point_it);

    // Find the orientation as quaternion
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, (double)config_->ref_psi_[ref_point_it]);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // // Convert from global_frame to planning frame
    // if (config_->global_path_frame_.compare(config_->target_frame_) != 0)
    //     transformPose(config_->global_path_frame_, config_->target_frame_, pose);

    x_[ref_point_it] = pose.position.x;
    y_[ref_point_it] = pose.position.y;
    psi_[ref_point_it] = RosTools::quaternionToAngle(pose.orientation);
  }
}

void ReferencePath::ConstructReferencePath(const std::vector<double> &x, const std::vector<double> &y,
                                           const std::vector<double> &psi)
{
  if (config_->enable_clothoid_interpolation_)
    ConstructReferencePathWithClothoid(x, y, psi);
  else
    ConstructReferencePathWithoutClothoid(x, y);
}

void ReferencePath::ConstructReferencePathWithClothoid(const std::vector<double> &x, const std::vector<double> &y,
                                                       const std::vector<double> &psi)
{
  LMPCC_INFO("Reference Path: Generating path with clothoid interpolation...");

  double k, dk, L;
  int n_clothoid = config_->n_points_clothoid_;
  int n_pts = config_->n_points_spline_;
  std::vector<double> X(n_clothoid), Y(n_clothoid);
  std::vector<double> X_all, Y_all, S_all;
  double total_length = 0;

  S_all.push_back(0);

  // For each set of waypoints in the trajectory
  for (size_t i = 0; i < x.size() - 1; i++)
  {
    // Build a clothoid and get the X and Y waypoints
    Clothoid::buildClothoid(x[i], y[i], psi[i], x[i + 1], y[i + 1], psi[i + 1], k, dk, L);
    Clothoid::pointsOnClothoid(x[i], y[i], psi[i], k, dk, L, n_clothoid, X, Y);

    if (i == 0)
    {
      // For the first one insert the full clothoid
      X_all.insert(X_all.end(), X.begin(), X.end());
      Y_all.insert(Y_all.end(), Y.begin(), Y.end());
    }
    else
    {
      // Afterwards, insert all except for the duplicate initial point
      X.erase(X.begin() + 0);
      Y.erase(Y.begin() + 0);
      X_all.insert(X_all.end(), X.begin(), X.end());
      Y_all.insert(Y_all.end(), Y.begin(), Y.end());
    }

    total_length += L;

    // Compute the distances along the clothoid
    for (int j = 1; j < n_clothoid; j++)
      S_all.push_back(S_all[j - 1 + i * (n_clothoid - 1)] + L / (n_clothoid - 1));
  }
  ref_path_x_.set_points(S_all, X_all);
  ref_path_y_.set_points(S_all, Y_all);

  dist_spline_pts_ = total_length / (n_pts);
  // ROS_DEBUG_STREAM("dist_spline_pts_: " << dist_spline_pts_);
  ss_.resize(n_pts);
  xx_.resize(n_pts);
  yy_.resize(n_pts);

  for (int i = 0; i < n_pts; i++)
  {
    ss_[i] = dist_spline_pts_ * (double)i;
    xx_[i] = ref_path_x_(ss_[i]);
    yy_[i] = ref_path_y_(ss_[i]);
  }

  ref_path_x_.set_points(ss_, xx_);
  ref_path_y_.set_points(ss_, yy_);
  reference_path_.reset(new RosTools::CubicSpline2D<tk::spline>(ref_path_x_, ref_path_y_));

  LMPCC_SUCCESS("Reference Path: Path generated");
}

void ReferencePath::ConstructReferencePathWithoutClothoid(const std::vector<double> &x, const std::vector<double> &y)
{
  LMPCC_INFO("Reference Path: Generating path without clothoid interpolation...");

  ROSTOOLS_ASSERT(x.size() == y.size(), "x and y vectors must have equal length");

  double total_length_ = 0.;

  std::vector<double> S_all;
  S_all.push_back(0.);

  LMPCC_INFO("Generating path...");

  for (size_t i = 1; i < x.size(); i++)
  {
    // Compute the distance between points
    total_length_ += std::sqrt(std::pow(x[i] - x[i - 1], 2) + std::pow(y[i] - y[i - 1], 2));

    S_all.push_back(total_length_);
  }
  config_->n_points_spline_ = S_all.size();

  // Same from here?
  ref_path_x_.set_points(S_all, x);
  ref_path_y_.set_points(S_all, y);
  reference_path_.reset(new RosTools::CubicSpline2D<tk::spline>(ref_path_x_, ref_path_y_));

  // Save the path with distances
  xx_ = x;
  yy_ = y;
  ss_ = S_all;

  LMPCC_SUCCESS("Reference Path: Path generated");
}

double ReferencePath::FindClosestSRecursively(const Eigen::Vector2d &pose, double low, double high, int num_recursions)
{
  // Stop after x recursions
  if (num_recursions > 10)
    return (low + high) / 2.;

  // Computes the distance between point "s" on the spline and a vehicle position
  auto dist_to_spline = [&](double s, const Eigen::Vector2d &pose)
  {
    return std::sqrt(std::pow(ref_path_x_(s) - pose(0), 2.) + std::pow(ref_path_y_(s) - pose(1), 2.));
  };

  // Compute a middle s value
  double mid = (low + high) / 2.;

  // Compute the distance to the spline for high/low
  double value_low = dist_to_spline(low, pose);
  double value_high = dist_to_spline(high, pose);

  // Check the next closest value
  if (value_low < value_high)
    return FindClosestSRecursively(pose, low, mid, num_recursions + 1);
  else
    return FindClosestSRecursively(pose, mid, high, num_recursions + 1);
}

int ReferencePath::RecursiveClosestPointSearch(SolverInterface *solver_interface_ptr, unsigned int cur_traj_i,
                                               double &s_guess, double window, int n_tries, int num_recursions)
{
  Eigen::Vector2d pose(solver_interface_ptr->State().x(), solver_interface_ptr->State().y());
  s_guess = FindClosestSRecursively(pose, 0., ss_.back(), 0);

  for (size_t i = 0; i < ss_.size() - 1; i++)
  {
    if (s_guess > ss_[i] && s_guess < ss_[i + 1])
      return i;
  }

  return ss_.size() - 1;

  // auto dist_to_spline = [&](double s, const Eigen::Vector2d& pose) {
  //   return std::sqrt(std::pow(ref_path_x_(s) - pose(0), 2.) + std::pow(ref_path_y_(s) - pose(1), 2.));
  // };

  // // What is ss?
  // double low = 0.;
  // double high = ss_.back();

  // if (ss_.size() > 0 && num_recursions < 12)
  // {
  //   double s_min = ss_[cur_traj_i] - MAX_STEP_BACK_TOLERANCE;
  //   double s_max = ss_[cur_traj_i + 1] + MAX_STEP_BACK_TOLERANCE;

  //   double lower = std::max(s_min, s_guess - window);
  //   double upper = std::min(s_max, s_guess + window);

  //   // if (lower >= upper)
  //   // {
  //   // LMPCC_WARN_ALWAYS("RecursiveClosestPointSearch lower >= upper");
  //   // return cur_traj_i;
  //   // }

  //   double s_i = upper;
  //   double spline_pos_x_i, spline_pos_y_i;
  //   double dist_i, min_dist;

  //   // First, try the furthest point in our search window. This is the reference that must be beat.
  //   double s_best = s_i;
  //   spline_pos_x_i = ref_path_x_(s_i);
  //   spline_pos_y_i = ref_path_y_(s_i);

  //   min_dist = std::sqrt(
  //       (spline_pos_x_i - solver_interface_ptr->State().x()) * (spline_pos_x_i - solver_interface_ptr->State().x()) +
  //       (spline_pos_y_i - solver_interface_ptr->State().y()) * (spline_pos_y_i - solver_interface_ptr->State().y()));

  //   // Compute the step size.
  //   // Divide by minus one. If you want to go from 1 to 3 (distance two) with three steps, step size must be
  //   (3 - 1) / 2 = 1
  //       // to go 1,2,3.
  //       double step_size = (upper - lower) / ((double)n_tries - 1.);

  //   for (s_i = lower; s_i < upper; s_i += step_size)
  //   {
  //     // Get the current spline position
  //     spline_pos_x_i = ref_path_x_(s_i);
  //     spline_pos_y_i = ref_path_y_(s_i);

  //     // Compute the distance
  //     dist_i = std::sqrt(
  //         (spline_pos_x_i - solver_interface_ptr->State().x()) * (spline_pos_x_i - solver_interface_ptr->State().x())
  //         + (spline_pos_y_i - solver_interface_ptr->State().y()) * (spline_pos_y_i -
  //         solver_interface_ptr->State().y()));

  //     // Save it if it is the smallest
  //     if (dist_i < min_dist)
  //     {
  //       min_dist = dist_i;
  //       s_best = s_i;
  //     }
  //   }

  //   // Save the previous best s
  //   double previous_guess = s_guess;
  //   s_guess = s_best;

  //   int next_traj = cur_traj_i;

  //   // If the smallest distance is the lower bound of the window
  //   if (s_best == lower && lower != previous_guess)
  //   {
  //     // If we hit the low point of the window, and that low point was the end of this spline segment, try one
  //     segment
  //         // higher!
  //         if (lower == s_min && cur_traj_i > 0)
  //     {
  //       next_traj--;
  //     }
  //     return RecursiveClosestPointSearch(solver_interface_ptr, next_traj, s_guess, window, n_tries, num_recursions +
  //     1);
  //   }

  //   if (s_best == upper && upper != previous_guess)
  //   {
  //     // If we hit the high point of the window, and that high point was the end of this spline segment, try one
  //     segment
  //         // higher!
  //         if (upper == s_max && cur_traj_i < ss_.size() - 2)
  //     {
  //       next_traj++;
  //     }
  //     return RecursiveClosestPointSearch(solver_interface_ptr, next_traj, s_guess, window, n_tries, num_recursions +
  //     1);
  //   }
  // }

  // return cur_traj_i;
}

void ReferencePath::UpdateClosestPoint(SolverInterface *solver_interface_ptr, double &s_guess, double window,
                                       int n_tries)
{
  spline_index_ = RecursiveClosestPointSearch(solver_interface_ptr, spline_index_, s_guess, window, n_tries, 0);
  PublishCurrentSplineIndex();
}

void ReferencePath::InitializeClosestPoint(SolverInterface *solver_interface_ptr)
{
  Eigen::Vector2d current_pose(solver_interface_ptr->State().x(), solver_interface_ptr->State().y());
  Eigen::Vector2d trajectory_pose;

  double smallest_dist = 9999999.0;
  double current_dist;
  int best_i = -1;
  for (int i = 0; i < (int)ss_.size(); i++)
  {
    trajectory_pose = Eigen::Vector2d(ref_path_x_(ss_[i]), ref_path_y_(ss_[i]));

    current_dist = RosTools::dist(current_pose, trajectory_pose);

    if (current_dist < smallest_dist)
    {
      smallest_dist = current_dist;
      best_i = i;
    }
  }

  if (best_i == -1)
    ROS_ERROR("Initial spline search failed: No point was found!");

  // If it succeeded return our best index
  if (best_i == -1)
    spline_index_ = std::max(0, int(ss_.size() - 1));
  else
    spline_index_ = best_i;

  // Visualizes the given reference points (for debug mostly)
  PublishReferencePath();

  // Visualize the road limits
  VisualizeRoad();
}

bool ReferencePath::EndOfCurrentSpline(double index)
{
  if (ss_.size() > spline_index_ + 1) // Checks if there are further splines
    return index > ss_[spline_index_ + 1];
  else
    return true;
}

bool ReferencePath::ReachedEnd()
{
  return (size_t)(spline_index_ + 2) >= ss_.size(); // 1 for size, 2 for extra spline parts
}

void ReferencePath::ConstructRoadConstraints(SolverInterface *solver_interface,
                                             std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out)
{
  return;
  LMPCC_INFO("Reference Path: Constructing linear road constraints.");

  RosTools::ROSPointMarker &cube = ros_markers_linearboundaries->getNewPointMarker("Cube");
  cube.setScale(0.5, 0.5, 0.5);

  geometry_msgs::Point p;
  p.z = 0.2;

  std::vector<double> x_path, y_path, dx_path, dy_path, lambdas;
  x_path.resize(segments);
  y_path.resize(segments);
  dx_path.resize(segments);
  dy_path.resize(segments);
  lambdas.resize(segments - 1);

  // For each stage
  for (size_t k = 0; k < solver_interface->FORCES_N; k++)
  {
    /** @brief automatic merging of n_segments_ segments */
    double cur_s = solver_interface->spline(k + 1); // + 1 because 0 is the initial

    x_path.clear();
    y_path.clear();
    dx_path.clear();
    dy_path.clear();
    lambdas.clear();

    for (int segment_id = 0; segment_id < segments; segment_id++)
    {
      // Compute the value and derivative of the path in (x, y)
      x_path[segment_id] = ref_path_x_(cur_s);
      y_path[segment_id] = ref_path_y_(cur_s);

      dx_path[segment_id] = ref_path_x_.deriv(1, cur_s); // - ss_[spline_index_ + segment_id]);
      dy_path[segment_id] = ref_path_y_.deriv(1, cur_s); // - ss_[spline_index_ + segment_id]);

      if (segment_id > 0)
        lambdas[segment_id - 1] = 1.0 / (1.0 + std::exp((cur_s - ss_[spline_index_ + segment_id] + 0.02) / 0.1));
    }

    double path_x, path_y, path_dx, path_dy;
    path_x = x_path[segments - 1];
    path_y = y_path[segments - 1];
    path_dx = dx_path[segments - 1];
    path_dy = dy_path[segments - 1];

    // Loop backwards through the segments, glueing them together
    for (int segment_id = segments - 1; segment_id > 0; segment_id--)
    {
      double lambda = lambdas[segment_id - 1];

      path_x = lambda * x_path[segment_id - 1] + (1. - lambda) * path_x; // Glue with the previous segment
      path_y = lambda * y_path[segment_id - 1] + (1. - lambda) * path_y;
      path_dx = lambda * dx_path[segment_id - 1] + (1. - lambda) * path_dx;
      path_dy = lambda * dy_path[segment_id - 1] + (1. - lambda) * path_dy;
    }

    // This is the final point and the normal vector of the path
    Eigen::Vector2d path_point = Eigen::Vector2d(path_x, path_y);
    Eigen::Vector2d dpath = Eigen::Vector2d(-path_dy, path_dx);

    // LEFT HALFSPACE
    // dx, -dy NOTE 3* FOR DOUBLE LANE, 1* FOR SINGLE LANE
    Eigen::Vector2d A(-path_dy, path_dx);
    double width_times = config_->two_way_road_ ? 3.0 : 1.0;
    // line is parallel to the spline
    Eigen::Vector2d boundary_left =
        path_point + dpath * (width_times * config_->road_width_left_ -
                              solver_interface->area_->DiscRadius()); // Incorporate the left road side
    double b = A.transpose() * boundary_left;                         // And lies on the boundary point
    halfspaces_out[k][0] = RosTools::Halfspace(A, b);                 // new_halfspace;

    // RIGHT HALFSPACE
    A = Eigen::Vector2d(-path_dy, path_dx); // line is parallel to the spline

    Eigen::Vector2d boundary_right =
        path_point - dpath * (config_->road_width_right_ - solver_interface->area_->DiscRadius());
    b = A.transpose() * boundary_right; // And lies on the boundary point

    halfspaces_out[k][1] = RosTools::Halfspace(-A, -b); // new_halfspace;
                                                        // p.x = boundary_right(0);
                                                        // p.y = boundary_right(1);
                                                        // cube.addPointMarker(p);
                                                        // p.x = boundary_left(0);
                                                        // p.y = boundary_left(1);
                                                        // cube.addPointMarker(p);
  }

  PublishLinearRoadBoundaries(solver_interface, halfspaces_out[0]);
}

void ReferencePath::PublishReferencePath()
{
  RosTools::ROSLine &line = ros_markers_reference_path_->getNewLine();
  line.setColorInt(10, 10);
  line.setScale(0.1, 0.1);
  line.setOrientation(0.0);

  RosTools::ROSPointMarker &arrow = ros_markers_reference_arrows_->getNewPointMarker("ARROW");
  arrow.setScale(1.5, 0.3, 0.3);

  geometry_msgs::Point prev_point;

  for (size_t i = 0; i < xx_.size(); i++)
  {
    // Draw the constraint as a line
    geometry_msgs::Point p;
    p.x = xx_[i];
    p.y = yy_[i];
    p.z = -0.5e-3;

    if (i > 0)
      line.addLine(prev_point, p);

    prev_point = p;

    arrow.setOrientation(config_->ref_psi_[i]);
    arrow.addPointMarker(p);
  }

  // Plot the part that we are tracking in the solver currently
  RosTools::ROSLine &active_line = ros_markers_reference_path_->getNewLine();
  active_line.setColorInt(8, 10, 0.5);
  active_line.setScale(0.25, 0.25);
  active_line.setOrientation(0.0);

  for (int segment_id = 0; segment_id < segments; segment_id++)
  {
    if (spline_index_ + segment_id + 1 > ss_.size()) // stop if we ran out of segments
      break;

    for (double t = 0.; t <= 1.; t += 0.1)
    {
      // Draw the constraint as a line
      geometry_msgs::Point p;
      p.x = ref_path_x_(ss_[spline_index_ + segment_id] * (1. - t) +
                        t * ss_[spline_index_ + segment_id + 1]); // Move from s[cur-segment] to s[next segment]
      p.y = ref_path_y_(ss_[spline_index_ + segment_id] * (1. - t) + t * ss_[spline_index_ + segment_id + 1]);
      p.z = -0.5e-3;

      if (!(segment_id == 0 && t == 0.))
        active_line.addLine(prev_point, p);

      prev_point = p;
    }
  }

  ros_markers_reference_arrows_->publish();
  ros_markers_reference_path_->publish();
}

void ReferencePath::PublishCurrentSplineIndex()
{
  // Use to debug spline init
  RosTools::ROSPointMarker &cube = ros_markers_splineindex->getNewPointMarker("Cube");
  cube.setScale(0.5, 0.5, 0.5);
  cube.setOrientation(config_->ref_psi_[spline_index_]);

  geometry_msgs::Point p;
  p.z = 0.2;

  if (xx_.size() > 0)
  {
    p.x = xx_[spline_index_];
    p.y = yy_[spline_index_];
    cube.addPointMarker(p);

    cube.setColor(0, 0.8, 0);
    p.x = xx_[spline_index_ + 1];
    p.y = yy_[spline_index_ + 1];
    cube.addPointMarker(p);

    ros_markers_splineindex->publish();
  }
}

void ReferencePath::PublishLinearRoadBoundaries(SolverInterface *solver_interface_ptr,
                                                const std::vector<RosTools::Halfspace> &halfspaces_out)
{
  LMPCC_INFO("Reference Path: Visualizing linear road constraints.");

  RosTools::DrawHalfspaces(*ros_markers_linearboundaries, halfspaces_out);
  ros_markers_linearboundaries->publish();
}

void ReferencePath::VisualizeRoad()
{
  RosTools::ROSLine &line = ros_markers_road_limits->getNewLine();
  line.setColor(0.5, 0.0, 0.0);
  line.setScale(0.1);
  line.setOrientation(0.0);

  geometry_msgs::Point prev_l, cur_l, prev_r, cur_r, prev_m, cur_m;
  cur_l.z = -0.5;
  prev_l.z = -0.5;
  cur_r.z = -0.5;
  prev_r.z = -0.5;

  for (size_t i = 0; i < ss_.size(); i++)
  {
    Eigen::Vector2d pose(xx_[i], yy_[i]);

    // Vector orthogonal to the derivative of the spline
    Eigen::Vector2d direction(-ref_path_y_.deriv(1, ss_[i]), ref_path_x_.deriv(1, ss_[i]));

    // Construct the road limits using the orthogonal vector
    Eigen::Vector2d right = pose - direction * config_->road_width_right_;
    Eigen::Vector2d mid = pose + direction * config_->road_width_left_;
    Eigen::Vector2d left = pose + direction * config_->road_width_left_ * 3.;

    cur_l.x = left(0);
    cur_l.y = left(1);

    cur_m.x = mid(0);
    cur_m.y = mid(1);

    cur_r.x = right(0);
    cur_r.y = right(1);

    if (i > 0)
    {
      line.setColor(0.1, 0.1, 0.1); // 64./256., 224. / 256., 208 / 256.);
      line.addLine(prev_l, cur_l);
      line.addLine(prev_r, cur_r);

      if (i % 2 == 1)
      {
        line.setColor(249. / 256., 215. / 256., 28 / 256.);
        line.addLine(prev_m, cur_m);
      }
    }

    prev_l = cur_l;
    prev_m = cur_m;
    prev_r = cur_r;
  }

  ros_markers_road_limits->publish();
}