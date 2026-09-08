// Copyright 2018 David V. Lu!!
#include <social_navigation_layers/proxemic_layer.h>
#include <math.h>
#include <cmath>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <list>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::ProxemicLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew)
{
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx),
         f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var)
{
  return sqrt(-2 * var * log(cutoff / A));
}

namespace social_navigation_layers
{
  double ProxemicLayer::calculateDensityAdjustedCost(double base_cost)
  {
    double w = density_weight_;
    return base_cost / exp(w * density_);
  }

void ProxemicLayer::onInitialize()
{
  SocialLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  server_ = new dynamic_reconfigure::Server<ProxemicLayerConfig>(nh);
  f_ = boost::bind(&ProxemicLayer::configure, this, _1, _2);
  server_->setCallback(f_);
}

void ProxemicLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::list<social_msgs::SocialPerson>::iterator p_it;

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
  {
    social_msgs::SocialPerson person = *p_it;

    double cx = person.position.position.x, cy = person.position.position.y;
    if (!std::isfinite(cx) || !std::isfinite(cy))
      continue;

    double vx = person.velocity.linear.x;
    double vy = person.velocity.linear.y;
    if (!std::isfinite(vx)) vx = 0.0;
    if (!std::isfinite(vy)) vy = 0.0;

    double mag = sqrt(vx * vx + vy * vy);
    if (!std::isfinite(mag)) mag = 0.0;
    double factor = (1.0 + mag * factor_);
    double point = std::max(get_radius(cutoff_, amplitude_, covar_ * factor), min_radius_);
    if (!std::isfinite(point) || point <= 0.0)
      continue;

    *min_x = std::min(*min_x, cx - point);
    *min_y = std::min(*min_y, cy - point);
    *max_x = std::max(*max_x, cx + point);
    *max_y = std::max(*max_y, cy + point);
  }
}

void ProxemicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_) return;

  if (social_people_list_.people.size() == 0)
    return;
  if (cutoff_ >= amplitude_)
    return;

  std::list<social_msgs::SocialPerson>::iterator p_it;
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();
  // Thêm log để kiểm tra danh sách người
  // ROS_INFO("Number of transformed people: %zu", transformed_people_.size());
   
  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
  {
    social_msgs::SocialPerson person = *p_it;

    double cx = person.position.position.x, cy = person.position.position.y;
    if (!std::isfinite(cx) || !std::isfinite(cy))
      continue;

    double vx = person.velocity.linear.x;
    double vy = person.velocity.linear.y;
    if (!std::isfinite(vx)) vx = 0.0;
    if (!std::isfinite(vy)) vy = 0.0;

    double angle = atan2(vy, vx);
    double mag = sqrt(vx * vx + vy * vy);
    if (!std::isfinite(mag)) mag = 0.0;
    double factor = (1.0 + mag * factor_);
    double base = std::max(get_radius(cutoff_, amplitude_, covar_), min_radius_);
    double point = std::max(get_radius(cutoff_, amplitude_, covar_ * factor), min_radius_);
    if (!std::isfinite(base) || !std::isfinite(point) || base <= 0.0 || point <= 0.0)
      continue;

    unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                 height = std::max(1, static_cast<int>((base + point) / res));

    double ox, oy;
    if (sin(angle) > 0)
      oy = cy - base;
    else
      oy = cy + (point - base) * sin(angle) - base;

    if (cos(angle) >= 0)
      ox = cx - base;
    else
      ox = cx + (point - base) * cos(angle) - base;


    int dx, dy;
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0, end_x = width, end_y = height;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + height > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j)
      start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)
      end_y = max_j - dy;

    double bx = ox + res / 2,
           by = oy + res / 2;
    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == costmap_2d::NO_INFORMATION)
          continue;

        double x = bx + i * res, y = by + j * res;
        double ma = atan2(y - cy, x - cx);
        double diff = angles::shortest_angular_distance(angle, ma);
        double dist_to_person = hypot(x - cx, y - cy);
        double a;
        if (fabs(diff) < M_PI / 2)
          a = gaussian(x, y, cx, cy, amplitude_, covar_ * factor, covar_, angle);
        else
          a = gaussian(x, y, cx, cy, amplitude_, covar_, covar_, 0);

        double density_cost = calculateDensityAdjustedCost(a);
        if (dist_to_person <= min_radius_)
        {
          density_cost = std::max(density_cost, cutoff_);
        }
        if (density_cost < cutoff_)
          continue;
        unsigned char cvalue = (unsigned char)density_cost;
        // cvalue = LETHAL_OBSTACLE;
        // cvalue = 100;
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }
}

void ProxemicLayer::configure(ProxemicLayerConfig &config, uint32_t level)
{
  cutoff_ = config.cutoff;
  amplitude_ = config.amplitude;
  covar_ = config.covariance;
  factor_ = config.factor;
  min_radius_ = config.min_radius;
  density_weight_ = config.proxemic_density_weight;
  people_keep_time_ = ros::Duration(config.keep_time);
  enabled_ = config.enabled;
}
};  // namespace social_navigation_layers
