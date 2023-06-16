#include <projection_converter/converter_to_llh.hpp>

ConverterToLLH::ConverterToLLH(const YAML::Node & config)
{
  projector_type_ = config["projector_type"].as<std::string>();
  if (projector_type_ == "MGRS") {
    mgrs_grid_ = config["mgrs_grid"].as<std::string>();
  }
}

pcl::PointXYZ ConverterToLLH::convert(const pcl::PointXYZ & xyz)
{
  pcl::PointXYZ llh;
  if (projector_type_ == "MGRS") {
    try {
      std::string mgrs = std::to_string(int(xyz.x)) + std::to_string(int(xyz.y));
      int zone;
      bool northp;
      double x, y;
      bool longpath = false;
      GeographicLib::MGRS::Reverse(mgrs_grid_ + mgrs, zone, northp, x, y, zone, longpath);

      int utm_zone = std::stoi(mgrs_grid_.substr(0,2));
      
      // Convert UTM to LLH
      GeographicLib::UTMUPS::Reverse(utm_zone, northp, x, y, lat_, lon_);
      llh.x = static_cast<float>(lat_); // Assign double lat, lon to float llh.x, llh.y
      llh.y = static_cast<float>(lon_);
      llh.z = xyz.z;
    } catch (const std::exception& e) {
      std::cerr << "Error: Could not convert from MGRS to UTM: " << e.what() << "\n";
      return pcl::PointXYZ();
    }
  }
  return llh;
}