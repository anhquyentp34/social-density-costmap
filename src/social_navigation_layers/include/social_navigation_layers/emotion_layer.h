// Copyright 2023
#ifndef EMOTION_LAYER_H_
#define EMOTION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <social_navigation_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/EmotionLayerConfig.h>

namespace social_navigation_layers
{

class EmotionLayer : public SocialLayer
{
public:
    EmotionLayer() { layered_costmap_ = NULL; }
    virtual void onInitialize();
    virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
    void configure(EmotionLayerConfig &config, uint32_t level);
    
    // Các tham số cấu hình
    double cutoff_, amplitude_, covar_, density_weight_;
    // double density_;  // Biến theo dõi mật độ
    
    // Server cấu hình động
    dynamic_reconfigure::Server<EmotionLayerConfig>* server_;
    dynamic_reconfigure::Server<EmotionLayerConfig>::CallbackType f_;

};

}
#endif
