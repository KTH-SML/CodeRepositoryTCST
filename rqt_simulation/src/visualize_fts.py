import rospy
from tf import transformations

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose

def init():
    global ID
    global marker_pub
    ID = 0

    marker_pub = rospy.Publisher('/region_markers', MarkerArray, queue_size=1, latch=True)


def create_marker(pose2d, text):
    global ID
    x, y, theta = pose2d

    ID += 1
    arrow_marker = Marker(id=ID)
    arrow_marker.type = Marker.ARROW
    arrow_marker.ns = "map"
    arrow_marker.header.frame_id = "map"
    arrow_marker.header.stamp = rospy.Time.now()
    q = Quaternion(*transformations.quaternion_from_euler(0, 0, theta))
    arrow_marker.pose = Pose(position=Point(x=x, y=y), orientation=q)
    arrow_marker.scale = Vector3(x=.5, y=.2, z=.3)
    arrow_marker.color.a = 1
    arrow_marker.color.r = 1.0

    ID += 1
    text_marker = Marker(id=ID)
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.ns = "map"
    text_marker.header.frame_id = "map"
    text_marker.header.stamp = rospy.Time.now()
    text_marker.text = text
    text_marker.pose.position.x = x
    text_marker.pose.position.y = y
    text_marker.pose.position.z = .5
    text_marker.scale.z = 1
    text_marker.color.a = 1
    text_marker.color.b = 1.0

    return [arrow_marker, text_marker]


def send_markers(marker_list):
    marker_pub.publish(MarkerArray(marker_list))
