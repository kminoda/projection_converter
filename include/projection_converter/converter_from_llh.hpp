#ifndef PROJECTION_CONVERTER__CONVERTER_FROM_LLH_HPP
#define PROJECTION_CONVERTER__CONVERTER_FROM_LLH_HPP

#include <iostream>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <gdal/ogr_spatialref.h>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/TransverseMercatorExact.hpp>
#include <projection_converter/lat_lon_alt.hpp>

#include <utility>


class ConverterFromLLH
{
public:
  ConverterFromLLH(const YAML::Node & config);
  ~ConverterFromLLH();
  pcl::PointXYZ convert(const LatLonAlt & xyz);

private:
  OGRCoordinateTransformation *poTransform_;
  std::string projector_type_;
  std::pair<double, double> origin_xy_;
  double central_meridian_;

};

#endif // PROJECTION_CONVERTER__CONVERTER_FROM_LLH_HPP
