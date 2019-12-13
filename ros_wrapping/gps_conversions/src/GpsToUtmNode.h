#ifndef GPS_TO_UTM_NODE_H
#define GPS_TO_UTM_NODE_H

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <gps_common/GPSFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

struct UtmPose
{
  double easting = 0.;   ///< [m] X direction, pointing east.
  double northing = 0.;  ///< [m] Y direction, pointing north.
  double altitude = 0.;  ///< [m] Z direction, pointing up.
  double bearing = 0.;   ///< [deg] clockwise, 0 is north.
  uint8_t zone;          ///< UTM longitude zone number
  char band;             ///< MGRS latitude band letter

  UtmPose() = default;

  UtmPose(const geodesy::UTMPoint& utmPoint, double bearing)
    : easting(utmPoint.easting), northing(utmPoint.northing), altitude(utmPoint.altitude)
    , bearing(bearing), zone(utmPoint.zone), band(utmPoint.band)
  {}

  UtmPose(double easting_, double northing_, double altitude_, double bearing_, uint8_t zone_, char band_)
    : easting(easting_), northing(northing_), altitude(altitude_), bearing(bearing_), zone(zone_), band(band_)
  {}

  bool isValid() const { return (easting || northing || altitude || bearing); }
};


class GpsToUtmNode
{
public:

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  GpsToUtmNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     GPS pose callback, projecting GPS (Lon/Lat/Alt) to UTM (X/Y/Z)
   * @param[in] msg New GPS position with its associated covariance.
   */
  void GpsPoseCallback(const gps_common::GPSFix& msg);

private:

  //------------------------------------------------------------------------------
  void ProcessUtmPose(const gps_common::GPSFix& msg, const UtmPose& utmPose);

  // ROS publishers & subscribers
  ros::Subscriber GpsPoseSub;
  ros::Publisher UtmPosePub;
  tf2_ros::TransformBroadcaster TfBroadcaster;
  tf2_ros::StaticTransformBroadcaster StaticTfBroadcaster;

  // UTM Band/zone
  char UtmBand;     ///< MGRS latitude band letter.
  uint8_t UtmZone;  ///< UTM longitude zone number.

  // Parameters
  std::string FrameId;
  std::string ChildFrameId;
  std::string MapFrameId = "map";
  double TimeOffset = 0.;          ///< Output odom time = GPS time + TimeOffset
  bool PublishTfToMap = false;     ///< true: publish a static TF from FrameId to MapFrameId to match 1st GPS pose to local map.
  bool OriginOnFirstPose = false;  ///< false: publish gps pose in UTM coordinates. true: publish pose in MapFrameId (= 1st GPS pose).
  UtmPose FirstGpsPose;            ///< 1st GPS pose received, only used if OriginOnFirstPose = true.
  UtmPose PreviousGpsPose;         ///< Previous GPS pose received.
  gps_common::GPSFix PreviousMsg;  ///< Previous message received, only used if heading is not defined.
};

#endif // GPS_TO_UTM_NODE_H