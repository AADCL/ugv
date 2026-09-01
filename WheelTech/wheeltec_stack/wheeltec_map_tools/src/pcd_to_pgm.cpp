#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

class PcdToPgm
{
public:
    PcdToPgm() : pnh_("~")
    {
        pnh_.param<std::string>("input_pcd", input_pcd_, "");
        pnh_.param<std::string>("output_pgm", output_pgm_, "");
        pnh_.param<std::string>("output_yaml", output_yaml_, "");
        pnh_.param("classification_mode", classification_mode_, false);
        pnh_.param<std::string>("ground_pcd", ground_pcd_, "");
        pnh_.param<std::string>("obstacle_pcd", obstacle_pcd_, "");

        pnh_.param<double>("resolution", resolution_, 0.05);
        pnh_.param<double>("padding_m", padding_m_, 0.50);
        pnh_.param<double>("floor_min_z", floor_min_z_, -0.30);
        pnh_.param<double>("floor_max_z", floor_max_z_, 0.05);
        pnh_.param<double>("obstacle_min_z", obstacle_min_z_, 0.05);
        pnh_.param<double>("obstacle_max_z", obstacle_max_z_, 1.20);
        pnh_.param<double>("free_dilation_m", free_dilation_m_, 0.10);
        pnh_.param<double>("obstacle_inflation_m", obstacle_inflation_m_, 0.30);
        pnh_.param("terrain_cost/enable", terrain_cost_enable_, false);
        pnh_.param<double>("terrain_cost/fit_radius_m", terrain_fit_radius_m_, 0.30);
        pnh_.param("terrain_cost/min_plane_cells", terrain_min_plane_cells_, 4);
        pnh_.param<double>("terrain_cost/max_height_delta_m", terrain_max_height_delta_m_, 0.40);
        pnh_.param<double>("terrain_cost/flat_slope_deg", terrain_flat_slope_deg_, 3.0);
        pnh_.param<double>("terrain_cost/max_slope_deg", terrain_max_slope_deg_, 22.0);
        pnh_.param("terrain_cost/min_cost", terrain_min_cost_, 15);
        pnh_.param("terrain_cost/max_cost", terrain_max_cost_, 80);
        pnh_.param<double>("terrain_cost/dilation_m", terrain_cost_dilation_m_, 0.25);

        terrain_min_plane_cells_ = std::max(3, terrain_min_plane_cells_);
        terrain_min_cost_ = std::max(1, std::min(98, terrain_min_cost_));
        terrain_max_cost_ = std::max(terrain_min_cost_, std::min(98, terrain_max_cost_));
        terrain_max_slope_deg_ = std::max(
            terrain_flat_slope_deg_ + 0.1, terrain_max_slope_deg_);

        generate();
    }

private:
    int index(int x, int y) const
    {
        return y * width_ + x;
    }

    void dilate(
        const std::vector<uint8_t>& input,
        std::vector<uint8_t>& output,
        int radius)
    {
        output.assign(input.size(), 0);

        for (int y = 0; y < height_; ++y)
        {
            for (int x = 0; x < width_; ++x)
            {
                if (!input[index(x, y)])
                    continue;

                for (int dy = -radius; dy <= radius; ++dy)
                {
                    for (int dx = -radius; dx <= radius; ++dx)
                    {
                        if (dx * dx + dy * dy > radius * radius)
                            continue;

                        const int nx = x + dx;
                        const int ny = y + dy;

                        if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_)
                            continue;

                        output[index(nx, ny)] = 1;
                    }
                }
            }
        }
    }

    std::string basename(const std::string& path) const
    {
        const std::size_t pos = path.find_last_of("/\\");
        if (pos == std::string::npos)
            return path;
        return path.substr(pos + 1);
    }

    void writePgm(const std::vector<uint8_t>& image)
    {
        std::ofstream file(output_pgm_, std::ios::binary);
        if (!file)
            throw std::runtime_error("Cannot open output PGM.");

        file << "P5\n";
        file << width_ << " " << height_ << "\n";
        file << "255\n";

        for (int y = height_ - 1; y >= 0; --y)
        {
            for (int x = 0; x < width_; ++x)
            {
                const uint8_t value = image[index(x, y)];
                file.write(reinterpret_cast<const char*>(&value), 1);
            }
        }
    }

    void writeYaml()
    {
        std::ofstream file(output_yaml_);
        if (!file)
            throw std::runtime_error("Cannot open output YAML.");

        file << "image: " << basename(output_pgm_) << "\n";
        file << "resolution: " << resolution_ << "\n";
        file << "origin: [" << min_x_ << ", " << min_y_ << ", 0.0]\n";
        file << "negate: 0\n";
        file << "occupied_thresh: 0.65\n";
        file << "free_thresh: 0.196\n";
        if (terrain_cost_enable_)
            file << "mode: raw\n";
    }

    void generate()
    {
        if (classification_mode_)
        {
            generateClassified();
            return;
        }

        if (input_pcd_.empty() || output_pgm_.empty() || output_yaml_.empty())
            throw std::runtime_error("PCD/PGM/YAML path is empty.");

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>);

        if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_pcd_, *cloud) < 0)
            throw std::runtime_error("Failed to load PCD.");

        if (cloud->empty())
            throw std::runtime_error("PCD is empty.");

        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        min_x_ = std::numeric_limits<double>::infinity();
        min_y_ = std::numeric_limits<double>::infinity();

        for (const auto& p : cloud->points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            min_x_ = std::min(min_x_, static_cast<double>(p.x));
            min_y_ = std::min(min_y_, static_cast<double>(p.y));
            max_x = std::max(max_x, static_cast<double>(p.x));
            max_y = std::max(max_y, static_cast<double>(p.y));
        }

        min_x_ -= padding_m_;
        min_y_ -= padding_m_;
        max_x += padding_m_;
        max_y += padding_m_;

        width_ = static_cast<int>(std::ceil((max_x - min_x_) / resolution_));
        height_ = static_cast<int>(std::ceil((max_y - min_y_) / resolution_));

        const std::size_t cell_count =
            static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);

        std::vector<uint32_t> floor_count(cell_count, 0);
        std::vector<uint32_t> obstacle_count(cell_count, 0);

        for (const auto& p : cloud->points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            const int gx = static_cast<int>(
                std::floor((p.x - min_x_) / resolution_));

            const int gy = static_cast<int>(
                std::floor((p.y - min_y_) / resolution_));

            if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_)
                continue;

            const int id = index(gx, gy);

            if (p.z >= floor_min_z_ && p.z <= floor_max_z_)
                ++floor_count[id];

            if (p.z >= obstacle_min_z_ && p.z <= obstacle_max_z_)
                ++obstacle_count[id];
        }

        std::vector<uint8_t> floor_mask(cell_count, 0);
        std::vector<uint8_t> obstacle_mask(cell_count, 0);

        for (std::size_t i = 0; i < cell_count; ++i)
        {
            if (floor_count[i] > 0)
                floor_mask[i] = 1;

            if (obstacle_count[i] > 0)
                obstacle_mask[i] = 1;
        }

        const int free_radius = static_cast<int>(
            std::round(free_dilation_m_ / resolution_));

        const int obstacle_radius = static_cast<int>(
            std::round(obstacle_inflation_m_ / resolution_));

        std::vector<uint8_t> free_mask;
        std::vector<uint8_t> inflated_obstacle;

        dilate(floor_mask, free_mask, free_radius);
        dilate(obstacle_mask, inflated_obstacle, obstacle_radius);

        std::vector<uint8_t> image(cell_count, 205);

        for (std::size_t i = 0; i < cell_count; ++i)
        {
            if (free_mask[i])
                image[i] = 254;
        }

        for (std::size_t i = 0; i < cell_count; ++i)
        {
            if (inflated_obstacle[i])
                image[i] = 0;
        }

        writePgm(image);
        writeYaml();

        ROS_INFO("PCD points: %zu", cloud->size());
        ROS_INFO("Map: %d x %d, resolution %.3f", width_, height_, resolution_);
        ROS_INFO("PGM: %s", output_pgm_.c_str());
        ROS_INFO("YAML: %s", output_yaml_.c_str());
    }

    void generateClassified()
    {
        if (ground_pcd_.empty() || obstacle_pcd_.empty() ||
            output_pgm_.empty() || output_yaml_.empty())
            throw std::runtime_error("Classified PCD/PGM/YAML path is empty.");

        pcl::PointCloud<pcl::PointXYZI> ground;
        pcl::PointCloud<pcl::PointXYZI> obstacles;
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(ground_pcd_, ground) < 0 ||
            pcl::io::loadPCDFile<pcl::PointXYZI>(obstacle_pcd_, obstacles) < 0)
            throw std::runtime_error("Failed to load classified terrain PCD files.");
        if (ground.empty() && obstacles.empty())
            throw std::runtime_error("Classified terrain PCD files are empty.");

        min_x_ = std::numeric_limits<double>::infinity();
        min_y_ = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        const auto update_bounds = [&](const pcl::PointCloud<pcl::PointXYZI>& cloud) {
            for (const auto& p : cloud.points)
            {
                if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
                min_x_ = std::min(min_x_, static_cast<double>(p.x));
                min_y_ = std::min(min_y_, static_cast<double>(p.y));
                max_x = std::max(max_x, static_cast<double>(p.x));
                max_y = std::max(max_y, static_cast<double>(p.y));
            }
        };
        update_bounds(ground);
        update_bounds(obstacles);
        if (!std::isfinite(min_x_) || !std::isfinite(min_y_) ||
            !std::isfinite(max_x) || !std::isfinite(max_y))
            throw std::runtime_error("Classified terrain PCD has no finite XY points.");

        min_x_ -= padding_m_;
        min_y_ -= padding_m_;
        max_x += padding_m_;
        max_y += padding_m_;
        width_ = std::max(1, static_cast<int>(std::ceil((max_x - min_x_) / resolution_)));
        height_ = std::max(1, static_cast<int>(std::ceil((max_y - min_y_) / resolution_)));
        const std::size_t cell_count = static_cast<std::size_t>(width_) * height_;
        std::vector<uint8_t> floor_mask(cell_count, 0);
        std::vector<uint8_t> obstacle_mask(cell_count, 0);
        std::vector<double> height_sum(cell_count, 0.0);
        std::vector<uint32_t> height_count(cell_count, 0);

        const auto mark = [&](const pcl::PointCloud<pcl::PointXYZI>& cloud,
                              std::vector<uint8_t>* mask) {
            for (const auto& p : cloud.points)
            {
                if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
                const int gx = static_cast<int>(std::floor((p.x - min_x_) / resolution_));
                const int gy = static_cast<int>(std::floor((p.y - min_y_) / resolution_));
                if (gx >= 0 && gy >= 0 && gx < width_ && gy < height_)
                    (*mask)[index(gx, gy)] = 1;
            }
        };
        mark(ground, &floor_mask);
        mark(obstacles, &obstacle_mask);

        for (const auto& p : ground.points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;
            const int gx = static_cast<int>(std::floor((p.x - min_x_) / resolution_));
            const int gy = static_cast<int>(std::floor((p.y - min_y_) / resolution_));
            if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_)
                continue;
            const int id = index(gx, gy);
            height_sum[id] += p.z;
            ++height_count[id];
        }

        const int free_radius = static_cast<int>(std::round(free_dilation_m_ / resolution_));
        const int obstacle_radius = static_cast<int>(std::round(obstacle_inflation_m_ / resolution_));
        std::vector<uint8_t> free_mask;
        std::vector<uint8_t> inflated_obstacle;
        dilate(floor_mask, free_mask, free_radius);
        dilate(obstacle_mask, inflated_obstacle, obstacle_radius);
        std::vector<uint8_t> image(cell_count, terrain_cost_enable_ ? 255 : 205);
        if (terrain_cost_enable_)
        {
            std::vector<uint8_t> terrain_cost(cell_count, 0);
            const int fit_radius = std::max(
                1, static_cast<int>(std::round(terrain_fit_radius_m_ / resolution_)));

            for (int y = 0; y < height_; ++y)
            {
                for (int x = 0; x < width_; ++x)
                {
                    const int center_id = index(x, y);
                    if (height_count[center_id] == 0)
                        continue;
                    const double center_z = height_sum[center_id] / height_count[center_id];
                    std::vector<Eigen::Vector3d> rows;
                    std::vector<double> values;
                    rows.reserve((2 * fit_radius + 1) * (2 * fit_radius + 1));
                    values.reserve(rows.capacity());

                    for (int dy = -fit_radius; dy <= fit_radius; ++dy)
                    {
                        for (int dx = -fit_radius; dx <= fit_radius; ++dx)
                        {
                            if (dx * dx + dy * dy > fit_radius * fit_radius)
                                continue;
                            const int nx = x + dx;
                            const int ny = y + dy;
                            if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_)
                                continue;
                            const int neighbor_id = index(nx, ny);
                            if (height_count[neighbor_id] == 0)
                                continue;
                            const double z = height_sum[neighbor_id] / height_count[neighbor_id];
                            if (std::abs(z - center_z) > terrain_max_height_delta_m_)
                                continue;
                            rows.emplace_back(dx * resolution_, dy * resolution_, 1.0);
                            values.push_back(z);
                        }
                    }

                    if (static_cast<int>(rows.size()) < terrain_min_plane_cells_)
                        continue;
                    Eigen::MatrixXd a(rows.size(), 3);
                    Eigen::VectorXd b(rows.size());
                    for (std::size_t i = 0; i < rows.size(); ++i)
                    {
                        a.row(i) = rows[i].transpose();
                        b(i) = values[i];
                    }
                    const Eigen::Vector3d plane = a.colPivHouseholderQr().solve(b);
                    const double slope_deg = std::atan(std::hypot(plane.x(), plane.y()))
                                           * 180.0 / M_PI;
                    if (slope_deg <= terrain_flat_slope_deg_)
                        continue;
                    const double ratio = std::max(0.0, std::min(
                        1.0,
                        (slope_deg - terrain_flat_slope_deg_) /
                        (terrain_max_slope_deg_ - terrain_flat_slope_deg_)));
                    terrain_cost[center_id] = static_cast<uint8_t>(std::round(
                        terrain_min_cost_ + ratio * (terrain_max_cost_ - terrain_min_cost_)));
                }
            }

            // Max-filter the terrain cost by half the chassis width plus padding.
            // This prevents a centre-line path from putting only one side of the car on a ramp.
            std::vector<uint8_t> expanded_cost(cell_count, 0);
            const int cost_radius = std::max(
                0, static_cast<int>(std::ceil(terrain_cost_dilation_m_ / resolution_)));
            for (int y = 0; y < height_; ++y)
            {
                for (int x = 0; x < width_; ++x)
                {
                    const uint8_t cost = terrain_cost[index(x, y)];
                    if (cost == 0)
                        continue;
                    for (int dy = -cost_radius; dy <= cost_radius; ++dy)
                    {
                        for (int dx = -cost_radius; dx <= cost_radius; ++dx)
                        {
                            if (dx * dx + dy * dy > cost_radius * cost_radius)
                                continue;
                            const int nx = x + dx;
                            const int ny = y + dy;
                            if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_)
                                continue;
                            const int id = index(nx, ny);
                            if (free_mask[id])
                                expanded_cost[id] = std::max(expanded_cost[id], cost);
                        }
                    }
                }
            }

            for (std::size_t i = 0; i < cell_count; ++i)
                if (free_mask[i]) image[i] = expanded_cost[i];
            for (std::size_t i = 0; i < cell_count; ++i)
                if (inflated_obstacle[i]) image[i] = 100;

            const std::size_t costly_cells = static_cast<std::size_t>(std::count_if(
                expanded_cost.begin(), expanded_cost.end(),
                [](uint8_t value) { return value > 0; }));
            ROS_INFO("Terrain slope cost: cells=%zu range=%d..%d dilation=%.2fm",
                     costly_cells, terrain_min_cost_, terrain_max_cost_, terrain_cost_dilation_m_);
        }
        else
        {
            for (std::size_t i = 0; i < cell_count; ++i)
                if (free_mask[i]) image[i] = 254;
            for (std::size_t i = 0; i < cell_count; ++i)
                if (inflated_obstacle[i]) image[i] = 0;
        }
        writePgm(image);
        writeYaml();
        ROS_INFO("Classified terrain: ground=%zu obstacles=%zu", ground.size(), obstacles.size());
        ROS_INFO("Map: %d x %d, resolution %.3f", width_, height_, resolution_);
    }

private:
    ros::NodeHandle pnh_;

    std::string input_pcd_;
    std::string output_pgm_;
    std::string output_yaml_;
    std::string ground_pcd_;
    std::string obstacle_pcd_;
    bool classification_mode_{false};

    double resolution_;
    double padding_m_;
    double floor_min_z_;
    double floor_max_z_;
    double obstacle_min_z_;
    double obstacle_max_z_;
    double free_dilation_m_;
    double obstacle_inflation_m_;
    bool terrain_cost_enable_{false};
    double terrain_fit_radius_m_;
    int terrain_min_plane_cells_;
    double terrain_max_height_delta_m_;
    double terrain_flat_slope_deg_;
    double terrain_max_slope_deg_;
    int terrain_min_cost_;
    int terrain_max_cost_;
    double terrain_cost_dilation_m_;

    int width_{0};
    int height_{0};

    double min_x_{0.0};
    double min_y_{0.0};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheeltec_pcd_to_pgm");

    try
    {
        PcdToPgm converter;
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("%s", e.what());
        return 1;
    }

    return 0;
}
