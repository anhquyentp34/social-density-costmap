#ifndef HUMAN_OBJECT_LAYER_H_
#define HUMAN_OBJECT_LAYER_H_

#include <social_navigation_layers/social_layer.h>
#include <social_msgs/SocialInteractions.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/HumanObjectLayerConfig.h>

namespace social_navigation_layers
{
  class HumanObjectLayer : public SocialLayer
  {
    public:
      HumanObjectLayer() : SocialLayer() {}

      virtual void onInitialize();
      virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected:
      // Dynamic reconfigure
      boost::shared_ptr<dynamic_reconfigure::Server<HumanObjectLayerConfig> > dsrv_;
      void reconfigureCB(HumanObjectLayerConfig &config, uint32_t level);

      // Gaussian distribution parameters
      double cutoff_;      // Threshold for cost calculation
      double amplitude_;   // Maximum cost at person's position
      double covar_;       // Base covariance for Gaussian
      double factor_;      // Factor to adjust covariance based on distance
      double density_weight_;  // Hệ số suy giảm theo mật độ

      // Helper functions
      double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
      double get_radius(double cutoff, double amplitude, double covar);
  };
};

#endif
