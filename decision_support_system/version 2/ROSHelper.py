#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

class ROSHelper:
	"""Помогает интегрироваться в ROS."""

	def __init__(self):
		"""Помогает интегрироваться в ROS."""
		self._pub = rospy.Publisher('chatter', String, queue_size=100)
		rospy.init_node('talker', anonymous=True)

	def SendMessage(self,text):
		"""Отправить сообщение в ROS, если он доступен. В противном случае игнорировть сооющение."""
		try:
			rospy.loginfo(text)
			self._pub.publish(text)
		except:
			pass


