"""Privileged scorer. Truth is never republished to robot or twin topics."""
from collections import deque
import json
import math
import time
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from .core import PoseHistory, wrap
from .unit import RUN, atomic_json, yaw

class Evaluator(Node):
    def __init__(self):
        super().__init__('fleetscope_evaluator')
        self.truth=PoseHistory(); self.pending=deque(maxlen=300)
        self.errors=[]; self.yaw_errors=[]; self.truth_count=0; self.missed=0
        self.first=None; self.last=None; self.distance=0.
        self.create_subscription(TFMessage,'/evaluation/ground_truth',self.ground_truth,30)
        self.create_subscription(String,'/twin/husky/state',self.estimate,30)
        self.create_timer(.5,self.score)
        self.log=(RUN/'pose-errors.jsonl').open('a')

    def ground_truth(self,msg):
        for tf in msg.transforms:
            # Model PosePublisher supplies a capture stamp and model child_frame_id.
            if tf.child_frame_id not in ('husky_mobile_manipulator','fleetscope_milestone/husky_mobile_manipulator'):
                continue
            p=tf.transform.translation; q=tf.transform.rotation
            t=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            value=[p.x,p.y,yaw(q)]
            self.truth.add(t,value); self.truth_count+=1
            if self.first is None: self.first=value
            if self.last: self.distance+=math.hypot(value[0]-self.last[0],value[1]-self.last[1])
            self.last=value

    def estimate(self,msg): self.pending.append(json.loads(msg.data))

    def score(self):
        while self.pending:
            state=self.pending[0]; value=self.truth.at(state['stamp'],max_gap=.25)
            if value is None:
                if not self.truth.items or state['stamp']>=self.truth.items[-1][0]: break
                self.pending.popleft(); self.missed+=1; continue
            self.pending.popleft(); p=state['pose']
            error=math.hypot(p[0]-value[0],p[1]-value[1]); angle=abs(wrap(p[2]-value[2]))
            self.errors.append(error); self.yaw_errors.append(angle)
            self.log.write(json.dumps({'stamp':state['stamp'],'estimate':p,'truth':value,'position_error_m':error,'yaw_error_rad':angle})+'\n')
        self.log.flush()
        result={'samples':len(self.errors),'truth_samples':self.truth_count,'unmatched':self.missed,
                'position_rmse_m':math.sqrt(sum(e*e for e in self.errors)/len(self.errors)) if self.errors else None,
                'position_max_m':max(self.errors) if self.errors else None,
                'yaw_rmse_rad':math.sqrt(sum(e*e for e in self.yaw_errors)/len(self.yaw_errors)) if self.yaw_errors else None,
                'truth_distance_m':self.distance,'last_truth_pose':self.last,
                'last_truth_stamp':self.truth.items[-1][0] if self.truth.items else None,'updated_wall':time.time()}
        atomic_json(RUN/'evaluation.json',result)

def main():
    rclpy.init(); node=Evaluator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.score(); node.log.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
