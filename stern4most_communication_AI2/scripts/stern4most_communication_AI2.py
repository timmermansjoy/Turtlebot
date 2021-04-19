#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from referee.srv import *

player_name = "AI2"


class stern4most_communication_AI2:
    def __init__(self):
        self.manual_autonomous_pub = rospy.Publisher('manual_autonomous', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic manual_autonomous')

        self.game_on_subscriber = rospy.Subscriber('game_on', String, self.callback_game_on)
        rospy.loginfo('subscribed to topic game_on')

        self.sector_crossed_sub = rospy.Subscriber('sector_crossed', Bool, self.callback_sector_crossed)
        rospy.loginfo('subscribed to topic sector_info')

        self.ranking_sub = rospy.Subscriber('ranking', String, self.callback_ranking)
        rospy.loginfo('subscribed to topic ranking')

        self.ranking_pub = rospy.Publisher('dashboard_ranking', String, queue_size=10)
        rospy.loginfo('created publisher for topic dashboard_ranking')

        self.sector = 1
        self.start_message = Bool()
        self.sector_update = rospy.ServiceProxy('status_update', SectorUpdate)

    def callback_ranking(self, msg):
        self.ranking_pub.publish(msg)

    def callback_game_on(self, msg):
        rospy.loginfo('received message from referee: ' + str(msg.data))
        if msg.data == 'Start':
            rospy.loginfo('publishing to topic manual_autonomous')
            self.start_message.data = True
            self.manual_autonomous_pub.publish(self.start_message)
            self.sector_update(player_name, int(self.sector))

    def callback_sector_crossed(self, msg):
        self.sector = self.sector % 16 + 1
        self.sector = 1 if self.sector == 0 else self.sector
        response = self.sector_update(player_name, int(self.sector))
        if response.status == 'WRONG_SECTOR' or response.status == 'FINISHED':
            self.start_message.data = False
            self.manual_autonomous_pub.publish(self.start_message)


if __name__ == '__main__':
    rospy.init_node('stern4most_communication_AI2')
    rospy.loginfo('stern4most node has been initialized')
    stern4most_communication_AI2()
    rospy.spin()
