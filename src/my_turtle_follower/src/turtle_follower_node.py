#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
import math


class TurtleFollowerNode:
    # Конструктор класса
    def __init__(self):
        # Инициализация узла ROS
        rospy.init_node('turtle_follower')

        # Получение параметров скорости и имени следуемой черепашки
        self.speed = rospy.get_param('~speed', 1.0)
        self.following_turtle_name = rospy.get_param('~following_turtle_name', 'turtle1')

        # Подписка на тему позиции следуемой черепашки
        self.following_turtle_pose_sub = rospy.Subscriber('/'+self.following_turtle_name+'/pose', Pose, self.following_turtle_pose_callback)

        # Публикация сообщений о скорости движения черепашки
        self.follower_turtle_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

        # Ожидание сообщения о позиции следуемой черепашки
        rospy.wait_for_message('/'+self.following_turtle_name+'/pose', Pose)

        # Запуск метода run()
        self.run()

    # Callback-функция для получения позиции следуемой черепашки
    def following_turtle_pose_callback(self, data):
        self.following_turtle_pose = data
        self.is_moving = (abs(data.linear_velocity) > 0.1 or abs(data.angular_velocity) > 0.1)

    # Основной метод управления движением черепашки
    def run(self):
        rate = rospy.Rate(10) # 10 Гц
        is_following = True

        while not rospy.is_shutdown():

            # Получение позиции черепашки, которую мы следим
            follower_turtle_pose = rospy.wait_for_message('/turtle2/pose', Pose)

            # Если мы перестали следить за черепашкой, но позиция следуемой черепашки не определена, продолжаем ожидать
            if not is_following:
                if self.following_turtle_pose is None:
                    continue
                else:
                    is_following = True

            # Вычисление разницы в координатах между черепашками
            dx = self.following_turtle_pose.position.x - follower_turtle_pose.position.x
            dy = self.following_turtle_pose.position.y - follower_turtle_pose.position.y

            # Вычисление угла поворота черепашки
            desired_yaw = math.atan2(dy, dx)
            err_yaw = desired_yaw - follower_turtle_pose.orientation.z

            # Настройка скорости движения черепашки
            cmd_vel = Twist()
            cmd_vel.linear.x = self.speed
            cmd_vel.angular.z = 3.0 * err_yaw # Пропорциональный регулятор
            self.follower_turtle_vel_pub.publish(cmd_vel)

            # Если расстояние между черепашками меньше 0.5, останавливаемся
            if math.sqrt(dx**2 + dy**2) < 0.5:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.follower_turtle_vel_pub.publish(cmd_vel)

            rate.sleep()


if __name__ == '__main__':
    try:
        TurtleFollowerNode()
    except rospy.ROSInterruptException:
        pass