#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from referee.srv import *

player_name = "AI2"


class stern4most_communication_AI2:
    def __init__(self):
        
        # ---- Subscribers ----
        self.game_on_subscriber = rospy.Subscriber('game_on', String, self.callback_game_on)
        rospy.loginfo('subscribed to topic game_on')
        self.start_message = Bool()

        self.sector_crossed_sub = rospy.Subscriber('sector_crossed', Bool, self.callback_sector_crossed)
        rospy.loginfo('subscribed to topic sector_info')

        self.ranking_sub = rospy.Subscriber('ranking', String, self.callback_ranking)
        rospy.loginfo('subscribed to topic ranking')

        self.sternformost_sub = rospy.Subscriber('sterformost', Bool, self.callback_sternformost)
        rospy.loginfo('subscribed to topic sternformost')
        self.BACKWARDS = False

        # ---- Publishers ----
        self.manual_autonomous_pub = rospy.Publisher('manual_autonomous', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic manual_autonomous')

        self.ranking_pub = rospy.Publisher('dashboard_ranking', String, queue_size=10)
        rospy.loginfo('created publisher for topic dashboard_ranking')

        # ---- Service ----
        self.sector_update = rospy.ServiceProxy('status_update', SectorUpdate)

        self.sector = 0 if self.BACKWARDS else 1
        

    def callback_ranking(self, msg):
        self.ranking_pub.publish(msg)

    def callback_game_on(self, msg):
        if msg.data == 'Start':
            self.start_message.data = True
            rospy.loginfo('publishing to topic manual_autonomous with value ' + str(self.start_message.data))
            self.manual_autonomous_pub.publish(self.start_message)

            rospy.loginfo('Calling sector_update')
            self.sector_update(player_name, int(self.sector))

    def callback_sector_crossed(self, msg):
        self.sector = self.sector % 16 + 1
        if self.sector == 0:
            self.sector += 1

        response = self.sector_update(player_name, int(self.sector))
        if response.status == 'WRONG_SECTOR' or response.status == 'FINISHED':
            self.start_message.data = False
            self.manual_autonomous_pub.publish(self.start_message)
    
    def callback_sternformost(self, msg):
        self.BACKWARDS = msg.data


if __name__ == '__main__':
    rospy.init_node('stern4most_communication_AI2')
    rospy.loginfo('stern4most node has been initialized')
    stern4most_communication_AI2()
    rospy.spin()
