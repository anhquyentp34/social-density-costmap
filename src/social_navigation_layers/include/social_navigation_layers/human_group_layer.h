#ifndef HUMAN_GROUP_LAYER_H_
#define HUMAN_GROUP_LAYER_H_

#include <social_navigation_layers/social_layer.h>
#include <social_msgs/SocialGroups.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/HumanGroupLayerConfig.h>
#include <ros/ros.h>

namespace social_navigation_layers
{
  class HumanGroupLayer : public SocialLayer
  {
    public:
      HumanGroupLayer() : SocialLayer() {}

      virtual void onInitialize();
      virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected:
      // Dynamic reconfigure
      boost::shared_ptr<dynamic_reconfigure::Server<HumanGroupLayerConfig> > dsrv_;
      void reconfigureCB(HumanGroupLayerConfig &config, uint32_t level);

      // Gaussian distribution parameters
      double cutoff_;      // Threshold for cost calculation
      double amplitude_;   // Maximum cost at group center
      double covar_;       // Base covariance for Gaussian
      double factor_;      // Factor to adjust group radius
      double density_weight_;  // Hệ số suy giảm theo mật độ

      // Helper functions
      double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
      double get_radius(double cutoff, double amplitude, double covar);
      social_msgs::SocialGroup analyzeSocialGroup(const social_msgs::SocialGroup& group);
  };
};

#endif 