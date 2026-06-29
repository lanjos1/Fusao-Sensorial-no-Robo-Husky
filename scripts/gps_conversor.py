#!/usr/bin/env python
import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

EARTH_RADIUS_M = 6378137.0
HIGH_VARIANCE = 1e5

class GpsConverter(object):
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.child_frame_id = rospy.get_param("~child_frame_id", "gps_link")
        self.use_first_fix_as_origin = rospy.get_param("~use_first_fix_as_origin", True)
        
        self.origin_lat = rospy.get_param("~origin_latitude", None)
        self.origin_lon = rospy.get_param("~origin_longitude", None)
        self.origin_alt = rospy.get_param("~origin_altitude", 0.0)
        self.default_variance = rospy.get_param("~position_covariance", 4.0)

        if not self.use_first_fix_as_origin and (self.origin_lat is None or self.origin_lon is None):
            raise rospy.ROSInitException("Set origin_latitude/origin_longitude or enable use_first_fix_as_origin")

        self.pub = rospy.Publisher("/gps/odom", Odometry, queue_size=10)
        self.sub = rospy.Subscriber("/fix", NavSatFix, self.fix_cb, queue_size=10)

    def fix_cb(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            rospy.logwarn_throttle(5.0, "Ignoring GPS sample without fix")
            return

        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            rospy.loginfo("GPS local origin set to lat=%.8f lon=%.8f", self.origin_lat, self.origin_lon)

        lat_rad = math.radians(msg.latitude)
        lon_rad = math.radians(msg.longitude)
        lat0_rad = math.radians(self.origin_lat)
        lon0_rad = math.radians(self.origin_lon)

        delta_lon = lon_rad - lon0_rad
        delta_lat = lat_rad - lat0_rad

        pos_x = delta_lon * math.cos(lat0_rad) * EARTH_RADIUS_M
        pos_y = delta_lat * EARTH_RADIUS_M
        pos_z = msg.altitude - self.origin_alt

        odom = Odometry()
        odom.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        odom.pose.pose.position.x = pos_x
        odom.pose.pose.position.y = pos_y
        odom.pose.pose.position.z = pos_z
        odom.pose.pose.orientation.w = 1.0 

        covariance = [0.0] * 36
        
        in_cov = msg.position_covariance
        has_valid_cov = any(in_cov)

        covariance[0]  = in_cov[0] if has_valid_cov else self.default_variance  
        covariance[7]  = in_cov[4] if has_valid_cov else self.default_variance  
        covariance[14] = in_cov[8] if has_valid_cov else self.default_variance  
        
        covariance[21] = HIGH_VARIANCE  
        covariance[28] = HIGH_VARIANCE  
        covariance[35] = HIGH_VARIANCE  

        odom.pose.covariance = covariance
        self.pub.publish(odom)

if __name__ == "__main__":
    rospy.init_node("gps_to_odom")
    try:
        GpsConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass