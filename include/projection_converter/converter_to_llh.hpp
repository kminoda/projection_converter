#ifndef PROJECTION_CONVERTER__CONVERTER_TO_LLH_HPP
#define PROJECTION_CONVERTER__CONVERTER_TO_LLH_HPP

#include <iostream>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <gdal/ogr_spatialref.h>
#include <GeographicLib/MGRS.hpp>
#include <projection_converter/lat_lon_alt.hpp>


class ConverterToLLH
{
public:
  ConverterToLLH(const YAML::Node & config);
  LatLonAlt convert(const pcl::PointXYZ & llh);

private:
  std::string projector_type_;
  std::string mgrs_grid_;
  double lat_, lon_;
};

#endif // PROJECTION_CONVERTER__CONVERTER_TO_LLH_HPP
