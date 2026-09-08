// Copyright 2018 David V. Lu!!
#include <social_navigation_layers/proxemic_layer.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <algorithm>
#include <list>
#include <string>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{
class PassingLayer : public ProxemicLayer
{
public:
  PassingLayer() : ProxemicLayer()
  {
  }

  virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                            double* max_x, double* max_y)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

    std::string global_frame = layered_costmap_->getGlobalFrameID();
    transformed_people_.clear();

    for (unsigned int i = 0; i < social_people_list_.people.size(); i++)
    {
      social_msgs::SocialPerson& person = social_people_list_.people[i];
      social_msgs::SocialPerson tpt;
      geometry_msgs::PointStamped pt, opt;

      try
      {
        pt.point.x = person.position.position.x;
        pt.point.y = person.position.position.y;
        pt.point.z = person.position.position.z;
        pt.header.frame_id = social_people_list_.header.frame_id;
        pt.header.stamp = social_people_list_.header.stamp;
        tf_->transform(pt, opt, global_frame);
        tpt.position.position.x = opt.point.x;
        tpt.position.position.y = opt.point.y;
        tpt.position.position.z = opt.point.z;

        pt.point.x += person.velocity.linear.x;
        pt.point.y += person.velocity.linear.y;
        pt.point.z += person.velocity.linear.z;
        tf_->transform(pt, opt, global_frame);

        tpt.velocity.linear.x = tpt.position.position.x - opt.point.x;
        tpt.velocity.linear.y = tpt.position.position.y - opt.point.y;
        tpt.velocity.linear.z = tpt.position.position.z - opt.point.z;

        transformed_people_.push_back(tpt);

        double mag = sqrt(pow(tpt.velocity.linear.x, 2) + pow(tpt.velocity.linear.y, 2));
        
        double factor = (1.0 + mag * factor_);
        
        double point = get_radius(cutoff_, amplitude_, covar_ * factor);

        *min_x = std::min(*min_x, tpt.position.position.x - point);
        *min_y = std::min(*min_y, tpt.position.position.y - point);
        *max_x = std::max(*max_x, tpt.position.position.x + point);
        *max_y = std::max(*max_y, tpt.position.position.y + point);
      }
      catch (tf2::LookupException& ex)
      {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        continue;
      }
      catch (tf2::ConnectivityException& ex)
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        continue;
      }
      catch (tf2::ExtrapolationException& ex)
      {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        continue;
      }
    }
  }

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
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

    for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
    {
      social_msgs::SocialPerson person = *p_it;
      double angle = atan2(person.velocity.linear.y, person.velocity.linear.x) + 1.51;
      double mag = sqrt(pow(person.velocity.linear.x, 2) + pow(person.velocity.linear.y, 2));
      double factor = (1.0 + mag * factor_);
      double base = get_radius(cutoff_, amplitude_, covar_);
      double point = get_radius(cutoff_, amplitude_, covar_ * factor);

      unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                   height = std::max(1, static_cast<int>((base + point) / res));

      double cx = person.position.position.x, cy = person.position.position.y;

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
          double a;
          if (fabs(diff) < M_PI / 2)
            a = gaussian(x, y, cx, cy, amplitude_, covar_ * factor, covar_, angle);
          else
            continue;

          if (a < cutoff_)
            continue;
          // Tính toán chi phí dựa trên mật độ    
          double density_cost = calculateDensityAdjustedCost(a);
          unsigned char cvalue = (unsigned char) density_cost;
          // cvalue = LETHAL_OBSTACLE;
          costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
        }
      }
    }
  }

  double get_radius(double cutoff, double A, double var)
  {
    return sqrt(-2 * var * log(cutoff / A));
  }

  // Hàm helper để tính toán chi phí dựa trên mật độ
  double calculateDensityAdjustedCost(double base_cost) {
    // Sử dụng công thức hàm mũ để giảm chi phí khi mật độ tăng
    double w = 10.0;  // Hệ số điều chỉnh
    return base_cost / exp(w * density_);
  }
};
};  // namespace social_navigation_layers

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::PassingLayer, costmap_2d::Layer)