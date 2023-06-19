#include <projection_converter/converter_to_llh.hpp>
#include <iomanip>


ConverterToLLH::ConverterToLLH(const YAML::Node & config)
{
  projector_type_ = config["projector_type"].as<std::string>();
  if (projector_type_ == "MGRS") {
    mgrs_grid_ = config["mgrs_grid"].as<std::string>();
  }
}

LatLonAlt ConverterToLLH::convert(const pcl::PointXYZ & xyz)
{
  LatLonAlt llh;
  if (projector_type_ == "MGRS") {
    try {
      std::string mgrs = std::to_string(int(xyz.x * 1e3)) + std::to_string(int(xyz.y * 1e3));

      int zone;
      bool northp;
      double x, y;
      int prec = 8;
      bool longpath = false;
      GeographicLib::MGRS::Reverse(mgrs_grid_ + mgrs, zone, northp, x, y, prec, longpath);

      int utm_zone = std::stoi(mgrs_grid_.substr(0,2));

      // Convert UTM to LLH
      GeographicLib::UTMUPS::Reverse(utm_zone, northp, x, y, llh.lat, llh.lon);

      llh.alt = xyz.z;
    } catch (const std::exception& e) {
      std::cerr << "Error: Could not convert from MGRS to UTM: " << e.what() << "\n";
      return LatLonAlt();
    }
  }
  return llh;
}