#include <projection_converter/converter_from_llh.hpp>

ConverterFromLLH::ConverterFromLLH(const YAML::Node & config)
{
  projector_type_ = config["projector_type"].as<std::string>();

  if (projector_type_ == "Stereographic") {
    OGRSpatialReference oSourceSRS, oTargetSRS;
    
    // Set the source SRS to WGS84 Geographic
    oSourceSRS.SetWellKnownGeogCS("WGS84");
    
    // Set the target SRS to a Stereographic projection
    oTargetSRS.SetOS(config["mgrs_origin"]["latitude"].as<double>(), config["mgrs_origin"]["longitude"].as<double>(), 1, 0, 0);

    poTransform_ = OGRCreateCoordinateTransformation(&oSourceSRS, &oTargetSRS);
  }
}

ConverterFromLLH::~ConverterFromLLH()
{
  // Cleanup
  OCTDestroyCoordinateTransformation(poTransform_);
}

pcl::PointXYZ ConverterFromLLH::convert(const pcl::PointXYZ & llh)
{
  pcl::PointXYZ xyz;
  if (projector_type_ == "Stereographic") {
    double x = llh.x; // longitude
    double y = llh.y; // latitude

    if (!poTransform_->Transform(1, &x, &y)) {
      std::cerr << "Error: Transformation failed.\n";
      return pcl::PointXYZ();
    }

    xyz.x = x;
    xyz.y = y;
    xyz.z = llh.z;

  } else {
    std::cerr << "Error: Only conversion to Stereographic is supported currently.\n";
    return pcl::PointXYZ();
  }
  return xyz;
}