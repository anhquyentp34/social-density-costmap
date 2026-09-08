#include <social_navigation_layers/human_object_layer.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{
    namespace
    {
        double computeDensityGeometryScale(double density, double density_weight)
        {
            const double safe_density = std::max(0.0, density);
            return std::exp(-density_weight * safe_density);
        }
    }  // namespace

    void HumanObjectLayer::onInitialize() 
    {
        SocialLayer::onInitialize();
        current_ = true;
        enabled_ = true;
        
        ros::NodeHandle nh("~/" + name_);
        
        // Initialize Gaussian distribution parameters
        cutoff_ = 0.8;      // Base threshold for cost calculation
        amplitude_ = 254.0;  // Maximum cost at person's position
        covar_ = 4.0;       // Base covariance for Gaussian
        factor_ = 0.01;      // Factor to adjust covariance based on distance
        
        // Setup dynamic reconfigure
        dsrv_ = boost::make_shared<dynamic_reconfigure::Server<HumanObjectLayerConfig> >(nh);
        dynamic_reconfigure::Server<HumanObjectLayerConfig>::CallbackType cb = boost::bind(&HumanObjectLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        
        ROS_INFO("HumanObjectLayer initialized with Gaussian parameters - cutoff=%.2f, amplitude=%.2f, covar=%.2f, factor=%.2f", 
                 cutoff_, amplitude_, covar_, factor_);
    }

    void HumanObjectLayer::reconfigureCB(HumanObjectLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covar;
        factor_ = config.factor;
        density_weight_ = config.human_object_density_weight;
        people_keep_time_ = ros::Duration(config.keep_time);
    }

    // Helper function to calculate Gaussian distribution (matching proxemic_layer)
    double HumanObjectLayer::gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew) {
        double dx = x - x0, dy = y - y0;
        double h = sqrt(dx*dx + dy*dy);
        double angle = atan2(dy, dx);
        double mx = cos(angle - skew) * h;
        double my = sin(angle - skew) * h;
        double f1 = pow(mx, 2.0) / (2.0 * varx);
        double f2 = pow(my, 2.0) / (2.0 * vary);
        return A * exp(-(f1 + f2));
    }

    // Helper function to calculate radius based on Gaussian parameters
    double HumanObjectLayer::get_radius(double cutoff, double amplitude, double covar) {
        return sqrt(-2 * covar * log(cutoff/amplitude));
    }

    void HumanObjectLayer::updateBounds(double origin_x, double origin_y, double origin_z,
                                      double* min_x, double* min_y, double* max_x, double* max_y)
    {
        boost::recursive_mutex::scoped_lock lock(lock_);
        expireStaleSocialCaches();
        if (!enabled_ || social_interactions_list_.interactions.empty()) return;

        std::string global_frame = layered_costmap_->getGlobalFrameID();

        // Process each interaction from social_interactions_list_
        for (const auto& interaction : social_interactions_list_.interactions) {
            // Transform object position to global frame
            geometry_msgs::PointStamped obj_pt, obj_pt_global;
            obj_pt.point = interaction.object_position.position;
            obj_pt.header.frame_id = social_interactions_list_.header.frame_id;
            
            try {
                tf_->transform(obj_pt, obj_pt_global, global_frame);
            }
            catch(tf2::TransformException &ex) {
                ROS_ERROR("HumanObjectLayer: Transform error (object): %s", ex.what());
                continue;
            }

            // Process each participant in the interaction
            for (const auto& person : interaction.participants) {
                // Transform person position to global frame
                geometry_msgs::PointStamped person_pt, person_pt_global;
                person_pt.point = person.position.position;
                person_pt.header.frame_id = social_interactions_list_.header.frame_id;
                
                try {
                    tf_->transform(person_pt, person_pt_global, global_frame);
                }
                catch(tf2::TransformException &ex) {
                    ROS_ERROR("HumanObjectLayer: Transform error (person): %s", ex.what());
                    continue;
                }

                // Calculate interaction vector (from person to object)
                double dx_interaction = obj_pt_global.point.x - person_pt_global.point.x;
                double dy_interaction = obj_pt_global.point.y - person_pt_global.point.y;
                double angle = atan2(dy_interaction, dx_interaction);
                double dist = hypot(dx_interaction, dy_interaction);
                if (!std::isfinite(dist) || dist < 1e-3)
                    continue;
                const double density_scale = computeDensityGeometryScale(density_, density_weight_);
                const double effective_dist = dist * density_scale;
                if (!std::isfinite(effective_dist) || effective_dist <= 1e-6) {
                    continue;
                }

                // Calculate Gaussian parameters to ensure extension to object
                double base = get_radius(cutoff_, amplitude_, covar_);
                
                // Calculate factor to make the Gaussian reach exactly to the object
                // The factor is calculated to ensure the Gaussian value at object position equals cutoff
                double factor = (effective_dist * effective_dist) / (-2 * covar_ * log(cutoff_/amplitude_));
                double point = effective_dist;  // Density-scaled interaction length
                
                // Calculate bounding box
                double resolution = layered_costmap_->getCostmap()->getResolution();
                unsigned int width = std::max(1, int((base + point) / resolution));
                unsigned int height = std::max(1, int((base + point) / resolution));

                double cx = person_pt_global.point.x;
                double cy = person_pt_global.point.y;

                // Calculate origin point for the bounding box
                double ox, oy;
                if(sin(angle) > 0)
                    oy = cy - base;
                else
                    oy = cy + (point - base) * sin(angle) - base;

                if(cos(angle) >= 0)
                    ox = cx - base;
                else
                    ox = cx + (point - base) * cos(angle) - base;

                int map_x, map_y;
                layered_costmap_->getCostmap()->worldToMapNoBounds(ox, oy, map_x, map_y);

                // Calculate bounds for iteration
                int start_x = 0, start_y = 0, end_x = width, end_y = height;
                if(map_x < 0)
                    start_x = -map_x;
                else if(map_x + width > layered_costmap_->getCostmap()->getSizeInCellsX())
                    end_x = std::max(0, (int)layered_costmap_->getCostmap()->getSizeInCellsX() - map_x);

                if(map_y < 0)
                    start_y = -map_y;
                else if(map_y + height > layered_costmap_->getCostmap()->getSizeInCellsY())
                    end_y = std::max(0, (int)layered_costmap_->getCostmap()->getSizeInCellsY() - map_y);

                double bx = ox + resolution / 2;
                double by = oy + resolution / 2;

                // Update bounds
                *min_x = std::min(*min_x, ox);
                *min_y = std::min(*min_y, oy);
                *max_x = std::max(*max_x, ox + width * resolution);
                *max_y = std::max(*max_y, oy + height * resolution);
            }
        }
    }

    void HumanObjectLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        boost::recursive_mutex::scoped_lock lock(lock_);
        expireStaleSocialCaches();
        if (!enabled_ || social_interactions_list_.interactions.empty()) return;

        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();
        std::string global_frame = layered_costmap_->getGlobalFrameID();

        // Process each interaction from social_interactions_list_
        for (const auto& interaction : social_interactions_list_.interactions) {
            // Transform object position to global frame
            geometry_msgs::PointStamped obj_pt, obj_pt_global;
            obj_pt.point = interaction.object_position.position;
            obj_pt.header.frame_id = social_interactions_list_.header.frame_id;
            
            try {
                tf_->transform(obj_pt, obj_pt_global, global_frame);
            }
            catch(tf2::TransformException &ex) {
                ROS_ERROR("HumanObjectLayer: Transform error (object): %s", ex.what());
                continue;
            }

            // Process each participant in the interaction
            for (const auto& person : interaction.participants) {
                // Transform person position to global frame
                geometry_msgs::PointStamped person_pt, person_pt_global;
                person_pt.point = person.position.position;
                person_pt.header.frame_id = social_interactions_list_.header.frame_id;
                
                try {
                    tf_->transform(person_pt, person_pt_global, global_frame);
                }
                catch(tf2::TransformException &ex) {
                    ROS_ERROR("HumanObjectLayer: Transform error (person): %s", ex.what());
                    continue;
                }

                // Calculate interaction vector (from person to object)
                double dx_interaction = obj_pt_global.point.x - person_pt_global.point.x;
                double dy_interaction = obj_pt_global.point.y - person_pt_global.point.y;
                double angle = atan2(dy_interaction, dx_interaction);
                double dist = hypot(dx_interaction, dy_interaction);
                if (!std::isfinite(dist) || dist < 1e-3)
                    continue;
                const double density_scale = computeDensityGeometryScale(density_, density_weight_);
                const double effective_dist = dist * density_scale;
                if (!std::isfinite(effective_dist) || effective_dist <= 1e-6) {
                    continue;
                }

                // Calculate Gaussian parameters to ensure extension to object
                double base = get_radius(cutoff_, amplitude_, covar_);
                
                // Calculate factor to make the Gaussian reach exactly to the object
                // The factor is calculated to ensure the Gaussian value at object position equals cutoff
                double factor = (effective_dist * effective_dist) / (-2 * covar_ * log(cutoff_/amplitude_));
                double point = effective_dist;  // Density-scaled interaction length
                
                // Calculate bounding box
                double resolution = layered_costmap_->getCostmap()->getResolution();
                unsigned int width = std::max(1, int((base + point) / resolution));
                unsigned int height = std::max(1, int((base + point) / resolution));

                double cx = person_pt_global.point.x;
                double cy = person_pt_global.point.y;

                // Calculate origin point for the bounding box
                double ox, oy;
                if(sin(angle) > 0)
                    oy = cy - base;
                else
                    oy = cy + (point - base) * sin(angle) - base;

                if(cos(angle) >= 0)
                    ox = cx - base;
                else
                    ox = cx + (point - base) * cos(angle) - base;

                int map_x, map_y;
                costmap->worldToMapNoBounds(ox, oy, map_x, map_y);

                // Calculate bounds for iteration
                int start_x = 0, start_y = 0, end_x = width, end_y = height;
                if(map_x < 0)
                    start_x = -map_x;
                else if(map_x + width > costmap->getSizeInCellsX())
                    end_x = std::max(0, (int)costmap->getSizeInCellsX() - map_x);

                if((int)(start_x+map_x) < min_i)
                    start_x = min_i - map_x;
                if((int)(end_x+map_x) > max_i)
                    end_x = max_i - map_x;

                if(map_y < 0)
                    start_y = -map_y;
                else if(map_y + height > costmap->getSizeInCellsY())
                    end_y = std::max(0, (int)costmap->getSizeInCellsY() - map_y);

                if((int)(start_y+map_y) < min_j)
                    start_y = min_j - map_y;
                if((int)(end_y+map_y) > max_j)
                    end_y = max_j - map_y;

                double bx = ox + res / 2;
                double by = oy + res / 2;

                // Update costs using Gaussian distribution
                for(int i = start_x; i < end_x; i++) {
                    for(int j = start_y; j < end_y; j++) {
                        unsigned char old_cost = costmap->getCost(i+map_x, j+map_y);
                        if(old_cost == NO_INFORMATION)
                            continue;
                            
                        double x = bx + i*res;
                        double y = by + j*res;
                        double ma = atan2(y-cy, x-cx);
                        double diff = angles::shortest_angular_distance(angle, ma);
                        double a;

                        // Calculate distance from current point to person
                        double dx = x - cx;
                        double dy = y - cy;
                        double current_dist = hypot(dx, dy);

                        // Calculate cost based on distance ratio
                        double dist_ratio = current_dist / effective_dist;
                        if (!std::isfinite(dist_ratio))
                            continue;
                        if(dist_ratio <= 1.0) {
                            // Inside the interaction space
                            if(fabs(diff) < M_PI/2) {
                                // In the direction of the object
                                a = gaussian(x, y, cx, cy, amplitude_, covar_ * factor, covar_, angle);
                            } else {
                                // Outside the direction of the object
                                a = gaussian(x, y, cx, cy, amplitude_, covar_, covar_, 0);
                            }
                        } else {
                            // Outside the interaction space (lambda > 1): no social cost
                            continue;
                        }

                        if(a < cutoff_)
                            continue;

                        unsigned char cvalue = (unsigned char)a;
                        costmap->setCost(i+map_x, j+map_y, std::max(cvalue, old_cost));
                    }
                }
            }
        }
    }
};

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::HumanObjectLayer, costmap_2d::Layer)