#include <iostream>
#include <iomanip>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <gdal/ogr_spatialref.h>
#include <GeographicLib/MGRS.hpp>
#include <projection_converter/lat_lon_alt.hpp>

#include <projection_converter/converter_from_llh.hpp>
#include <projection_converter/converter_to_llh.hpp>

// Function to draw a progress bar
void drawProgressBar(int len, double percent) {
  std::cout << "Progress: ";
  for (int i = 0; i < len; ++i) {
    if (i < static_cast<int>(len * percent)) {
      std::cout << '=';
    }
    else {
      std::cout << ' ';
    }
  }
  std::cout << " [" << static_cast<int>(100 * percent) << "%]\r";
  std::cout.flush();
}


int main(int argc, char** argv) {
  if (argc != 5) {
    std::cout << "Usage: ./ProjectionConverter input_yaml output_yaml input_pcd output_pcd\n";
    return -1;
  }

  // Parse YAML configuration files
  YAML::Node input_config = YAML::LoadFile(argv[1]);
  YAML::Node output_config = YAML::LoadFile(argv[2]);

  // Load point cloud data from file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *cloud) == -1) {
    std::cerr << "Couldn't read file " << argv[3] << "\n";
    return -1;
  }

  // Define converters
  ConverterToLLH to_llh(input_config);
  ConverterFromLLH from_llh(output_config);

  // Convert points
  size_t n_points = cloud->points.size();
  for (size_t i = 0; i < n_points; ++i) {
    auto& point = cloud->points[i];
    LatLonAlt llh = to_llh.convert(point);
    point = from_llh.convert(llh);

    // Update and draw the progress bar
    drawProgressBar(70, static_cast<double>(i+1) / n_points);
  }
  std::cout << std::endl;

  // Save converted point cloud to file
  pcl::io::savePCDFileASCII(argv[4], *cloud);

  std::cout << "Point cloud projection conversion completed successfully.\n";

  return 0;
}