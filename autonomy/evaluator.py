"""Privileged comparison of known-map localization and raw dead reckoning."""
from collections import deque
import math
import time
import rclpy
from nav_msgs.msg import Odometry
from milestone.evaluator import Evaluator
from milestone.core import estate_pose,wrap
from milestone.unit import RUN,atomic_json,yaw,stamp

class Comparison(Evaluator):
    def __init__(self):
        super().__init__()
        self.odom_pending=deque(maxlen=300);self.odom_errors=[];self.odom_yaw_errors=[]
        self.create_subscription(Odometry,'/husky/odometry/filtered',self.odometry,10)

    def odometry(self,msg):
        p=msg.pose.pose
        self.odom_pending.append((stamp(msg),estate_pose(p.position.x,p.position.y,yaw(p.orientation))))

    def score(self):
        super().score()
        while self.odom_pending:
            t,p=self.odom_pending[0];truth=self.truth.at(t,max_gap=.25)
            if truth is None:
                if not self.truth.items or t>=self.truth.items[-1][0]:break
                self.odom_pending.popleft();continue
            self.odom_pending.popleft();self.odom_errors.append(math.hypot(p[0]-truth[0],p[1]-truth[1]));self.odom_yaw_errors.append(abs(wrap(p[2]-truth[2])))
        if self.odom_errors:
            atomic_json(RUN/'dead-reckoning-evaluation.json',dict(samples=len(self.odom_errors),
                position_rmse_m=math.sqrt(sum(e*e for e in self.odom_errors)/len(self.odom_errors)),
                position_max_m=max(self.odom_errors),last_position_error_m=self.odom_errors[-1],updated_wall=time.time()))

def main():
    rclpy.init();n=Comparison()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.score();n.log.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
