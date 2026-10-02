#!/usr/bin/env python3
"""First-person client for the fleetscope_player avatar in Gazebo Fortress.

Displays the head-camera image (bridged from gz-transport by ros_gz_bridge)
in a pygame window and sends movement commands back:

  WASD / arrows  walk (hold Shift to sprint)
  mouse          look (yaw on the body, pitch on the head joint)
  Space          jump
  Esc            quit

Topics (ROS 2 side of the parameter bridge):
  pub /model/fleetscope_player/cmd_vel                  geometry_msgs/Twist
  pub /model/fleetscope_player/joint/head_pitch/cmd_pos std_msgs/Float64
  sub /player/camera/image                              sensor_msgs/Image
"""

import sys
import time

import numpy as np
import pygame

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

WIDTH, HEIGHT = 960, 540
WALK_SPEED = 3.0
SPRINT_SPEED = 6.0
YAW_SENS = 0.35        # rad/s of yaw rate per mouse pixel per frame
PITCH_SENS = 0.0035    # rad per mouse pixel
PITCH_LIMIT = 1.2      # rad, matches head_pitch joint limits
CMD_TOPIC = "/model/fleetscope_player/cmd_vel"
PITCH_TOPIC = "/player/head_pitch/cmd_pos"
IMAGE_TOPIC = "/player/camera/image"


class FpsBridge(Node):
    def __init__(self):
        super().__init__("fleetscope_fps_client")
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.pitch_pub = self.create_publisher(Float64, PITCH_TOPIC, 10)
        self.latest_image = None
        self.frames_received = 0
        self.create_subscription(
            Image, IMAGE_TOPIC, self.on_image, QoSProfile(depth=5))

    def on_image(self, msg: Image):
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(
                f"unsupported image encoding: {msg.encoding}", once=True)
            return
        frame = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            frame = frame[:, :, ::-1]
        self.latest_image = frame
        self.frames_received += 1


def main():
    rclpy.init()
    node = FpsBridge()

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("FleetScope first person - Esc to quit")
    pygame.mouse.set_visible(False)
    pygame.event.set_grab(True)
    font = pygame.font.Font(None, 22)
    clock = pygame.time.Clock()

    pitch = 0.0
    last_log = time.monotonic()
    running = True

    try:
        while running:
            rclpy.spin_once(node, timeout_sec=0)
            mouse_dx = mouse_dy = 0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False
                elif event.type == pygame.MOUSEMOTION:
                    mouse_dx += event.rel[0]
                    mouse_dy += event.rel[1]

            keys = pygame.key.get_pressed()
            speed = SPRINT_SPEED if (
                keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]) else WALK_SPEED
            fwd = (keys[pygame.K_w] or keys[pygame.K_UP]) - (
                keys[pygame.K_s] or keys[pygame.K_DOWN])
            side = (keys[pygame.K_a] or keys[pygame.K_LEFT]) - (
                keys[pygame.K_d] or keys[pygame.K_RIGHT])

            twist = Twist()
            twist.linear.x = float(fwd * speed)
            twist.linear.y = float(side * speed)
            twist.linear.z = 1.0 if keys[pygame.K_SPACE] else 0.0
            twist.angular.z = float(
                max(-8.0, min(8.0, -mouse_dx * YAW_SENS)))
            node.cmd_pub.publish(twist)

            pitch = max(-PITCH_LIMIT, min(PITCH_LIMIT,
                                          pitch + mouse_dy * PITCH_SENS))
            node.pitch_pub.publish(Float64(data=pitch))

            frame = node.latest_image
            if frame is not None:
                surface = pygame.surfarray.make_surface(
                    np.ascontiguousarray(frame.transpose(1, 0, 2)))
                screen.blit(surface, (0, 0))
            else:
                screen.fill((20, 24, 28))
                msg = font.render("waiting for camera...", True, (200, 200, 200))
                screen.blit(msg, (WIDTH // 2 - msg.get_width() // 2,
                                  HEIGHT // 2))

            # crosshair + HUD
            cx, cy = WIDTH // 2, HEIGHT // 2
            pygame.draw.line(screen, (255, 255, 255), (cx - 8, cy), (cx + 8, cy))
            pygame.draw.line(screen, (255, 255, 255), (cx, cy - 8), (cx, cy + 8))
            hud = font.render(
                f"WASD move | Shift sprint | Space jump | mouse look | "
                f"{clock.get_fps():.0f} fps", True, (240, 240, 240))
            screen.blit(hud, (8, HEIGHT - 24))
            pygame.display.flip()

            now = time.monotonic()
            if now - last_log > 2.0:
                print(f"[fps_client] camera frames: {node.frames_received}, "
                      f"pitch: {pitch:+.2f} rad", flush=True)
                last_log = now

            clock.tick(60)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass  # e.g. rclpy context torn down by SIGTERM mid-loop
    finally:
        # stop the avatar before leaving
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
                time.sleep(0.1)
                node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass
        pygame.quit()


if __name__ == "__main__":
    sys.exit(main())
