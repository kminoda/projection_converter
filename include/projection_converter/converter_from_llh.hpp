#include <iostream>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <gdal/ogr_spatialref.h>
#include <GeographicLib/MGRS.hpp>

class ConverterFromLLH
{
public:
  ConverterFromLLH(const YAML::Node & config);
  ~ConverterFromLLH();
  pcl::PointXYZ convert(const pcl::PointXYZ & xyz);

private:
  OGRCoordinateTransformation *poTransform_;
  std::string projector_type_;
};