#include <social_navigation_layers/human_object_layer.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{
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
        amplitude_ = config.amplitude;  // Giữ nguyên giá trị amplitude từ cấu hình
        covar_ = config.covar;
        factor_ = config.factor;
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

    // Helper function to calculate dynamic cutoff based on distance
    double HumanObjectLayer::calculateDynamicCutoff(double distance) {
        // Base cutoff value from config
        double base_cutoff = cutoff_;
        
        // Adjust cutoff based on distance
        // For closer distances, use higher cutoff to make the costmap stop closer to the object
        // For longer distances, use lower cutoff to allow the costmap to extend further
        double distance_factor = std::max(0.1, std::min(1.0, distance / 5.0)); // Normalize distance to [0.1, 1.0]
        return base_cutoff * (1.0 - 0.5 * distance_factor); // Adjust cutoff between 0.5*base_cutoff and base_cutoff
    }

    // Helper function to calculate elliptical Gaussian distribution
    double HumanObjectLayer::ellipticalGaussian(double x, double y, double x1, double y1, double x2, double y2, double A, double var) {
        // Calculate center of ellipse (midpoint between person and object)
        double center_x = (x1 + x2) / 2.0;
        double center_y = (y1 + y2) / 2.0;
        
        // Calculate distance between foci (person and object)
        double foci_distance = hypot(x2 - x1, y2 - y1);
        
        // Calculate angle of major axis
        double major_axis_angle = atan2(y2 - y1, x2 - x1);
        
        // Transform point to ellipse coordinate system
        double dx = x - center_x;
        double dy = y - center_y;
        
        // Rotate point to align with major axis
        double rotated_x = dx * cos(-major_axis_angle) - dy * sin(-major_axis_angle);
        double rotated_y = dx * sin(-major_axis_angle) + dy * cos(-major_axis_angle);
        
        // Calculate distances along major and minor axes
        double major_axis_dist = rotated_x;
        double minor_axis_dist = rotated_y;
        
        // Calculate ellipse parameters
        double a = foci_distance * 0.75;  // Semi-major axis length (0.75 is a scaling factor)
        double b = a * 0.5;              // Semi-minor axis length (half of major axis)
        
        // Check if point is inside ellipse using standard ellipse equation
        double ellipse_value = (major_axis_dist * major_axis_dist) / (a * a) + 
                             (minor_axis_dist * minor_axis_dist) / (b * b);
        
        if (ellipse_value > 1.0) {
            return 0.0;  // Point is outside ellipse
        }
        
        // Calculate Gaussian value based on distance from center
        // Use different variances for major and minor axes
        double major_var = var;
        double minor_var = var * 0.5;  // Smaller variance for minor axis
        
        double gaussian_value = A * exp(-(major_axis_dist * major_axis_dist) / (2.0 * major_var) -
                                      (minor_axis_dist * minor_axis_dist) / (2.0 * minor_var));
        
        return gaussian_value;
    }

    // Hàm helper để tính toán chi phí dựa trên mật độ
    double HumanObjectLayer::calculateDensityAdjustedCost(double base_cost) {
        // Sử dụng công thức hàm mũ để giảm chi phí khi mật độ tăng
        double w = 0.5;  // Hệ số điều chỉnh
        return base_cost / exp(w * density_);
    }

    void HumanObjectLayer::updateBounds(double origin_x, double origin_y, double origin_z,
                                      double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_) return;

        boost::recursive_mutex::scoped_lock lock(lock_);
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

                // Calculate bounding box
                double bbox_radius = get_radius(cutoff_, amplitude_, covar_);
                
                // Center point of interaction space (person's position)
                double cx = person_pt_global.point.x;
                double cy = person_pt_global.point.y;
                
                // Update bounds
                *min_x = std::min(*min_x, cx - bbox_radius);
                *min_y = std::min(*min_y, cy - bbox_radius);
                *max_x = std::max(*max_x, cx + bbox_radius);
                *max_y = std::max(*max_y, cy + bbox_radius);
            }
        }
    }

    void HumanObjectLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_) return;

        boost::recursive_mutex::scoped_lock lock(lock_);
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

                // Calculate distance between person and object
                double dx = obj_pt_global.point.x - person_pt_global.point.x;
                double dy = obj_pt_global.point.y - person_pt_global.point.y;
                double dist = hypot(dx, dy);

                // Calculate dynamic cutoff based on distance
                double dynamic_cutoff = calculateDynamicCutoff(dist);

                // Calculate bounding box for the ellipse
                double bbox_radius = dist * 1.5; // 1.5 is the scaling factor used in ellipticalGaussian
                
                // Center point of interaction space (midpoint between person and object)
                double cx = (person_pt_global.point.x + obj_pt_global.point.x) / 2.0;
                double cy = (person_pt_global.point.y + obj_pt_global.point.y) / 2.0;
                
                // Calculate origin point for the bounding box
                double ox = cx - bbox_radius;
                double oy = cy - bbox_radius;

                int map_x, map_y;
                costmap->worldToMapNoBounds(ox, oy, map_x, map_y);

                // Calculate bounds for iteration
                unsigned int width = std::max(1, int((2 * bbox_radius) / res));
                unsigned int height = std::max(1, int((2 * bbox_radius) / res));

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

                // Update costs using elliptical Gaussian distribution
                for(int i = start_x; i < end_x; i++) {
                    for(int j = start_y; j < end_y; j++) {
                        unsigned char old_cost = costmap->getCost(i+map_x, j+map_y);
                        if(old_cost == NO_INFORMATION)
                            continue;
                            
                        double x = bx + i*res;
                        double y = by + j*res;

                        double a = ellipticalGaussian(x, y, 
                                                    person_pt_global.point.x, person_pt_global.point.y,
                                                    obj_pt_global.point.x, obj_pt_global.point.y,
                                                    amplitude_, covar_);

                        if(a < dynamic_cutoff)
                            continue;

                        // Điều chỉnh chi phí dựa trên mật độ
                        double adjusted_cost = calculateDensityAdjustedCost(a);
                        unsigned char cvalue = (unsigned char)adjusted_cost;
                        costmap->setCost(i+map_x, j+map_y, std::max(cvalue, old_cost));
                    }
                }
            }
        }
    }
};

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::HumanObjectLayer, costmap_2d::Layer)