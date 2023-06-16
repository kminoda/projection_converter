# Point Cloud Projection Converter
This project includes a tool for converting point cloud data between different geodetic projection systems. The projection systems supported can include MGRS, UTM, Stereographic, Transverse Mercator, etc.

The conversion details (input and output projection types) are specified in two YAML configuration files.

For example, to convert from MGRS to Stereographic projection, you would use configuration files like this:

```yaml
# input.yaml
projector_type: "MGRS"
mgrs_grid: "54SUE"
```
```yaml
# output.yaml
projector_type: "Stereographic"
mgrs_origin:
  latitude: xx
  longitude: yy
  altitude: zz
```

## Dependencies
- PCL (Point Cloud Library) 1.3 or higher
- yaml-cpp

## Building the Project
To build the project, use the following commands:

```bash
mkdir build
cd build
cmake ..
make
```
This will create an executable named `ProjectionConverter` in the build directory.

## Usage
To run the program, navigate to the build directory and use the following command:

```bash
./ProjectionConverter path_to_input_yaml path_to_output_yaml path_to_input_pcd_file path_to_output_pcd_file
```
Replace `path_to_input_yaml`, `path_to_output_yaml`, `path_to_pointcloud_file`, and `path_to_output_pcd_file` with the paths to your input YAML configuration file, output YAML configuration file, and PCD file, respectively.
