#include <iostream>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <gdal/ogr_spatialref.h>
#include <GeographicLib/MGRS.hpp>

class ConverterToLLH
{
public:
  ConverterToLLH(const YAML::Node & config);
  pcl::PointXYZ convert(const pcl::PointXYZ & llh);

private:
  std::string projector_type_;
  std::string mgrs_grid_;
  double lat_, lon_;
};