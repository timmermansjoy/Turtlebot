#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

def callback_game_on(msg):
    rospy.loginfo('received message from referee: ', msg)
    if msg.data == 'Start':
        rospy.loginfo('publishing to topic manual_autonomous')
        start_message.data = True
        manual_autonomous_pub.publish(start_message)
    
def callback_sector_crossed(msg):
    

if __name__ == '__main__':
    rospy.init_node('stern4most_communication_AI2')
    rospy.loginfo('stern4most node has been initialized')
    manual_autonomous_pub = rospy.Publisher('manual_autonomous', Bool, queue_size=10)
    rospy.loginfo('created publisher for topic manual_autonomous')
    game_on_subscriber = rospy.Subscriber('game_on', String, callback_game_on)
    rospy.loginfo('subscribed to topic game_on')
    sector_crossed_sub = rospy.Subscriber('sector_crossed', Bool, callback_sector_crossed)
    rospy.loginfo('subscribed to topic sector_info')
    start_message = Bool()

    rospy.spin()
