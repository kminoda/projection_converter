#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <gdal/ogr_spatialref.h>
#include <GeographicLib/MGRS.hpp>

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cout << "Usage: ./ProjectionConverter input_yaml output_yaml input_pcd output_pcd\n";
    return -1;
  }

  // Parse YAML configuration files
  YAML::Node input_config = YAML::LoadFile(argv[1]);
  YAML::Node output_config = YAML::LoadFile(argv[2]);

  // Check if the conversion is supported
  std::string input_projection_type = input_config["projector_type"].as<std::string>();
  std::string output_projection_type = output_config["projector_type"].as<std::string>();
  if (input_projection_type != "MGRS" || output_projection_type != "Stereographic") {
    std::cerr << "Error: Only conversion from MGRS to Stereographic is supported currently.\n";
    return -1;
  }

  // Load point cloud data from file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *cloud) == -1) {
    std::cerr << "Couldn't read file " << argv[3] << "\n";
    return -1;
  }

  // Initialize GDAL
  OGRSpatialReference oSourceSRS, oTargetSRS;
  oSourceSRS.SetWellKnownGeogCS("WGS84");
  oSourceSRS.SetUTM(54, TRUE);
  oTargetSRS.SetStereographic(output_config["mgrs_origin"]["latitude"].as<double>(), output_config["mgrs_origin"]["longitude"].as<double>(), 1, 0, 0);

  OGRCoordinateTransformation *poTransform = OGRCreateCoordinateTransformation(&oSourceSRS, &oTargetSRS);

  // Convert points
  for (auto& point : cloud->points) {
    double lat, lon;
    int zone; bool northp;
    double x = point.x;
    double y = point.y;
    double z = point.z;

    // Step 1: Convert MGRS to Geographic (lat/lon) using GeographicLib
    try {
      std::string mgrs = std::to_string(int(x)) + std::to_string(int(y));
      int zone = 0;
      bool longpath = false;
      GeographicLib::MGRS::Reverse(input_config["mgrs_grid"].as<std::string>() + mgrs, zone, northp, lat, lon, zone, longpath);
    } catch (const std::exception& e) {
      std::cerr << "Error: Could not convert from MGRS to UTM: " << e.what() << "\n";
      return -1;
    }

    // Step 2: Transform from Geographic (lat/lon) to Stereographic using GDAL
    if (!poTransform->Transform(1, &lon, &lat, &z)) {
      std::cerr << "Error: Transformation failed.\n";
      return -1;
    }

    point.x = x;
    point.y = y;
    point.z = z;
  }
  // Save converted point cloud to file
  pcl::io::savePCDFileASCII(argv[4], *cloud);

  // Cleanup
  OCTDestroyCoordinateTransformation(poTransform);

  std::cout << "Point cloud projection conversion completed successfully.\n";

  return 0;
}