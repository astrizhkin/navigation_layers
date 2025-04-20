import rospy
from contact_sensor_layer.msg import Contact

rospy.init_node('sender')
r = Contact()

r.header.frame_id = 'contact_1'
r.is_active = True

pub = rospy.Publisher('/bumper', Contact, queue_size=1)

while not rospy.is_shutdown():
    r.header.stamp = rospy.Time.now()
    pub.publish(r)

    rospy.sleep(5)
