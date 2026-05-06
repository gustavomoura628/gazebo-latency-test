#!/usr/bin/env python3
"""Pygame-based teleop with simultaneous key support"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

LINEAR_SPEED = 0.5   # m/s
ANGULAR_SPEED = 1.5  # rad/s

class GameTeleop(Node):
    def __init__(self):
        super().__init__('game_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption('Robot Teleop - WASD')
        self.font = pygame.font.Font(None, 36)
        self.clock = pygame.time.Clock()

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            # Get all pressed keys simultaneously
            keys = pygame.key.get_pressed()

            linear = 0.0
            angular = 0.0

            if keys[pygame.K_w] or keys[pygame.K_UP]:
                linear += LINEAR_SPEED
            if keys[pygame.K_s] or keys[pygame.K_DOWN]:
                linear -= LINEAR_SPEED
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                angular += ANGULAR_SPEED
            if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                angular -= ANGULAR_SPEED

            # Publish velocity
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.pub.publish(twist)

            # Draw UI
            self.screen.fill((30, 30, 30))

            # Title
            title = self.font.render('Robot Teleop', True, (255, 255, 255))
            self.screen.blit(title, (130, 20))

            # Controls display
            controls = [
                'W/Up: Forward',
                'S/Down: Backward',
                'A/Left: Turn Left',
                'D/Right: Turn Right',
                'ESC: Quit'
            ]
            small_font = pygame.font.Font(None, 24)
            for i, text in enumerate(controls):
                surf = small_font.render(text, True, (180, 180, 180))
                self.screen.blit(surf, (20, 70 + i * 25))

            # Current velocity indicator
            vel_text = f'Lin: {linear:+.1f} m/s  Ang: {angular:+.1f} rad/s'
            vel_surf = self.font.render(vel_text, True, (100, 255, 100))
            self.screen.blit(vel_surf, (50, 220))

            # Visual direction indicator
            cx, cy = 320, 150
            pygame.draw.circle(self.screen, (60, 60, 60), (cx, cy), 50, 2)

            if linear != 0 or angular != 0:
                end_x = cx + int(angular / ANGULAR_SPEED * -30)
                end_y = cy + int(linear / LINEAR_SPEED * -30)
                pygame.draw.line(self.screen, (100, 255, 100), (cx, cy), (end_x, end_y), 3)
            else:
                pygame.draw.circle(self.screen, (100, 100, 100), (cx, cy), 5)

            pygame.display.flip()
            self.clock.tick(30)

        # Stop robot on exit
        self.pub.publish(Twist())
        pygame.quit()

def main():
    rclpy.init()
    node = GameTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
