#include "specter_sense/frame_source.hpp"

namespace specter {

std::vector<DiscoveredSensor> discover_sensors() {
  std::vector<DiscoveredSensor> sensors;
#ifdef SPECTER_SENSE_HAS_KINECT_V1
  auto v1 = discover_kinect_v1_sensors();
  sensors.insert(sensors.end(), v1.begin(), v1.end());
#endif
#ifdef SPECTER_SENSE_HAS_KINECT_V2
  auto v2 = discover_kinect_v2_sensors();
  sensors.insert(sensors.end(), v2.begin(), v2.end());
#endif
  return sensors;
}

}  // namespace specter
