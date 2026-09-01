#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

        pnh_.param<double>("resolution", resolution_, 0.05);
        pnh_.param<double>("padding_m", padding_m_, 0.50);
        pnh_.param<double>("floor_min_z", floor_min_z_, -0.30);
        pnh_.param<double>("floor_max_z", floor_max_z_, 0.05);
        pnh_.param<double>("obstacle_min_z", obstacle_min_z_, 0.05);
        pnh_.param<double>("obstacle_max_z", obstacle_max_z_, 1.20);
        pnh_.param<double>("free_dilation_m", free_dilation_m_, 0.10);
        pnh_.param<double>("obstacle_inflation_m", obstacle_inflation_m_, 0.30);

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
    }

    void generate()
    {
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

private:
    ros::NodeHandle pnh_;

    std::string input_pcd_;
    std::string output_pgm_;
    std::string output_yaml_;

    double resolution_;
    double padding_m_;
    double floor_min_z_;
    double floor_max_z_;
    double obstacle_min_z_;
    double obstacle_max_z_;
    double free_dilation_m_;
    double obstacle_inflation_m_;

    int width_{0};
    int height_{0};

    double min_x_{0.0};
    double min_y_{0.0};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scout_pcd_to_pgm");

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

