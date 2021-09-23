#include <uma_navigation/input_layer.h>

#include <functional>
#include <algorithm>
#include <string>

InputLayer::InputLayer():
    dsrv_(nullptr)
{}

InputLayer::~InputLayer()
{
    if (dsrv_)
        delete dsrv_;
}

void InputLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;

    global_frame_ = layered_costmap_->getGlobalFrameID();

    std::string map_topic;
    nh.param("map_topic", map_topic, std::string("costmaps/local_costmap/costmap"));
    nh.param("subscribe_to_updates", subscribe_to_updates_, false);

    int temp_lethal_threshold, temp_unknown_cost_value;
    nh.param("lethal_cost_threshold", temp_lethal_threshold, static_cast<int>(100));
    lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);

    // Only resubscribe if topic has changed
    if (map_sub_.getTopic() != ros::names::resolve(map_topic))
    {
        // we'll subscribe to the latched topic that the map server uses
        ROS_INFO_STREAM("[PATH_PLANNER] Requesting the map at " << map_topic << "...");
        map_sub_ = g_nh.subscribe(map_topic, 1, &InputLayer::incomingMap, this);
        map_received_ = false;
        has_updated_data_ = false;

        ros::Rate r(10);
        while (!map_received_ && g_nh.ok())
        {
            ros::spinOnce();
            r.sleep();
        }

        ROS_INFO("[PATH_PLANNER] Received a %d X %d map at %f m/pix",
                 getSizeInCellsX(), getSizeInCellsY(), getResolution());

        if (subscribe_to_updates_)
        {
            ROS_INFO("[PATH_PLANNER] Subscribing to updates");
            map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &InputLayer::incomingUpdate, this);
        }
    }
    else
    {
        has_updated_data_ = true;
    }

    if (dsrv_)
    {
        delete dsrv_;
    }

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &InputLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void InputLayer::activate()
{
    onInitialize();
}

void InputLayer::deactivate()
{
    map_sub_.shutdown();
    if (subscribe_to_updates_)
        map_update_sub_.shutdown();
}

void InputLayer::reset()
{
    onInitialize();
}

void InputLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
        double *max_x, double *max_y)
{
    if (!layered_costmap_->isRolling())
    {
        if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
            return;
    }

    useExtraBounds(min_x, min_y, max_x, max_y);

    double wx, wy;

    mapToWorld(x_, y_, wx, wy);
    *min_x = std::min(wx, *min_x);
    *min_y = std::min(wy, *min_y);

    mapToWorld(x_ + width_, y_ + height_, wx, wy);
    *max_x = std::max(wx, *max_x);
    *max_y = std::max(wy, *max_y);

    has_updated_data_ = false;
}

void InputLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!map_received_)
        return;
    if (!layered_costmap_->isRolling())
        // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
        updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
        memcpy(master_grid.getCharMap(), costmap_, width * height);
}

void InputLayer::matchSize()
{
    Costmap2D *master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void InputLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr &new_map)
{
    unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

    ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);
    width = size_x;
    height = size_y;
    resolution_ = new_map->info.resolution;

    // resize costmap if size, resolution or origin do not match
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
        master->getSizeInCellsY() != size_y ||
        master->getResolution() != new_map->info.resolution ||
        master->getOriginX() != new_map->info.origin.position.x ||
        master->getOriginY() != new_map->info.origin.position.y ||
        !layered_costmap_->isSizeLocked()))
    {
        // Update the size of the layered costmap (and all layers, including this one)
        ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
        layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                            new_map->info.origin.position.y, true);
    }
    else if (size_x_ != size_x || size_y_ != size_y ||
        resolution_ != new_map->info.resolution ||
        origin_x_ != new_map->info.origin.position.x ||
        origin_y_ != new_map->info.origin.position.y)
    {
        // only update the size of the costmap stored locally in this layer
        // ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
        resizeMap(size_x, size_y, new_map->info.resolution,
            new_map->info.origin.position.x, new_map->info.origin.position.y);
    }

    // initialize the costmap with static data
    std::transform(new_map->data.begin(), new_map->data.end(), costmap_,
            std::bind(&InputLayer::interpretValue, this, std::placeholders::_1));
    map_frame_ = new_map->header.frame_id;

    // we have a new map, update full size of map
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
    map_received_ = true;
    has_updated_data_ = true;
}

void InputLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr &update)
{
    unsigned int di = 0;
    for (unsigned int y = 0; y < update->height ; y++)
    {
        unsigned int index_base = (update->y + y) * size_x_;
        for (unsigned int x = 0; x < update->width ; x++)
        {
            unsigned int index = index_base + x + update->x;
            costmap_[index] = interpretValue(update->data[di++]);
        }
    }
    x_ = update->x;
    y_ = update->y;
    width_ = update->width;
    height_ = update->height;
    has_updated_data_ = true;
}

void InputLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    if (config.enabled != enabled_)
    {
        enabled_ = config.enabled;
        has_updated_data_ = true;
        x_ = y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
    }
}

unsigned char InputLayer::interpretValue(unsigned char value)
{
    if (value >= lethal_threshold_)
        return costmap_2d::LETHAL_OBSTACLE;
    double scale = static_cast<double>(value / lethal_threshold_);
    return scale * costmap_2d::LETHAL_OBSTACLE;
}
