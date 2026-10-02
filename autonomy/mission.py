"""Bounded inspection jobs executed by Nav2, with explicit failure and cancellation."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from milestone.unit import RUN,atomic_json

ROUTES={
    'inspection':[('Warehouse entrance',(30.,-61.,math.pi/2)),('Collection shelter',(-18.,3.,math.pi/2)),('Depot forecourt',(-12.,-33.,-math.pi/2))],
    'quick':[('South forecourt',(-12.,-47.,-math.pi/2)),('East forecourt',(-5.,-47.,math.pi/2)),('Depot forecourt',(-12.,-33.,-math.pi/2))],
}

class Inspection(Node):
    def __init__(self):
        super().__init__('inspection_mission')
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.lifecycle=self.create_client(GetState,'/bt_navigator/get_state');self.lifecycle_pending=False;self.nav_active=False
        self.pub=self.create_publisher(String,'/inspection/status',10)
        self.ack_pub=self.create_publisher(String,'/inspection/ack',10)
        self.create_subscription(String,'/inspection/command',self.command,10)
        self.state=dict(status='IDLE',checkpoints=[],index=0,request_id=None)
        self.goal_handle=None;self.sending=False;self.goal_started=0.;self.deadline=900.
        self.events=(RUN/'mission-events.jsonl').open('a')
        self.create_timer(.5,self.tick)

    def event(self,name):
        self.state['stamp']=self.get_clock().now().nanoseconds/1e9
        self.events.write(json.dumps(dict(event=name,**self.state))+'\n');self.events.flush()
        # Acknowledge commands immediately, even if simulation time is paused.
        self.pub.publish(String(data=json.dumps(self.state)));atomic_json(RUN/'mission-status.json',self.state)

    def command(self,msg):
        try:
            cmd=json.loads(msg.data)
            def ack(accepted, reason):
                self.ack_pub.publish(String(data=json.dumps(dict(request_id=cmd.get('request_id'),
                    status='ACKNOWLEDGED' if accepted else 'REJECTED', reason=reason))))
            if cmd['action']=='cancel':
                if self.state['status'] in ('RUNNING','WAITING_FOR_NAV2'):
                    self.state['status']='CANCELING'
                    if self.goal_handle:self.goal_handle.cancel_goal_async()
                    elif not self.sending:self.state['status']='CANCELED'
                    self.event('cancel_requested')
                ack(True, 'Cancellation accepted; mission status reports completion')
                return
            if cmd['action']!='start': ack(False, 'Unknown mission action'); return
            if self.state['status'] in ('RUNNING','WAITING_FOR_NAV2','CANCELING'):
                ack(False, 'An inspection is already active'); return
            points=ROUTES[cmd.get('route','inspection')]
            self.state=dict(status='WAITING_FOR_NAV2',route=cmd.get('route','inspection'),request_id=cmd['request_id'],index=0,
                            checkpoints=[dict(name=name,pose=list(pose),status='PENDING') for name,pose in points])
            self.wait_started=time.monotonic();self.event('requested')
            ack(True, 'Inspection accepted')
        except (ValueError,KeyError,TypeError):self.get_logger().warning('Invalid inspection command')

    def feedback(self,msg):
        f=msg.feedback
        self.state['distance_remaining_m']=float(f.distance_remaining);self.state['recoveries']=f.number_of_recoveries

    def lifecycle_result(self,future):
        self.lifecycle_pending=False
        try:self.nav_active=future.result().current_state.id==3
        except Exception:self.nav_active=False

    def accepted(self,future):
        self.sending=False;self.goal_handle=future.result()
        if not self.goal_handle or not self.goal_handle.accepted:
            if self.state['status']=='CANCELING':
                self.state['status']='CANCELED';self.event('canceled');return
            self.finish(False,'Nav2 rejected goal');return
        self.goal_handle.get_result_async().add_done_callback(self.result)
        if self.state['status']=='CANCELING':self.goal_handle.cancel_goal_async()

    def result(self,future):
        code=future.result().status;self.goal_handle=None
        if self.state['status']=='CANCELING':
            timed_out=self.state.get('reason')=='Checkpoint deadline exceeded'
            self.state['status']='FAILED' if timed_out else 'CANCELED'
            self.state['checkpoints'][self.state['index']]['status']='UNREACHABLE' if timed_out else 'CANCELED'
            self.event('canceled');return
        if code==GoalStatus.STATUS_SUCCEEDED:
            self.state['checkpoints'][self.state['index']]['status']='REACHED';self.event('checkpoint_reached')
            self.state['index']+=1
            if self.state['index']==len(self.state['checkpoints']):self.state['status']='SUCCEEDED';self.event('completed')
        else:
            self.state['nav2_result_status']=code
            self.finish(False,'Nav2 could not complete the route after its recovery attempts')

    def finish(self,success,reason):
        self.state['status']='SUCCEEDED' if success else 'FAILED';self.state['reason']=reason
        if self.state['index']<len(self.state['checkpoints']):self.state['checkpoints'][self.state['index']]['status']='UNREACHABLE'
        self.event('finished')

    def tick(self):
        if self.state['status']=='WAITING_FOR_NAV2':
            if not self.lifecycle_pending and self.lifecycle.service_is_ready():
                self.lifecycle_pending=True;self.lifecycle.call_async(GetState.Request()).add_done_callback(self.lifecycle_result)
            if self.nav_active and self.nav.server_is_ready():self.state['status']='RUNNING'
            elif time.monotonic()-self.wait_started>60:self.finish(False,'Nav2 did not become ready within 60 seconds')
        if self.state['status']=='RUNNING' and not self.goal_handle and not self.sending:
            p=self.state['checkpoints'][self.state['index']];x,y,a=p['pose']
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='estate';goal.pose.header.stamp=self.get_clock().now().to_msg()
            goal.pose.pose.position.x=x;goal.pose.pose.position.y=y;goal.pose.pose.orientation.z=math.sin(a/2);goal.pose.pose.orientation.w=math.cos(a/2)
            p['status']='ACTIVE';self.goal_started=self.get_clock().now().nanoseconds/1e9;self.sending=True
            self.nav.send_goal_async(goal,feedback_callback=self.feedback).add_done_callback(self.accepted);self.event('goal_sent')
        if self.goal_handle and self.state['status']=='RUNNING' and self.get_clock().now().nanoseconds/1e9-self.goal_started>self.deadline:
            self.goal_handle.cancel_goal_async();self.state['status']='CANCELING';self.state['reason']='Checkpoint deadline exceeded';self.event('timeout')
        self.state['stamp']=self.get_clock().now().nanoseconds/1e9
        self.pub.publish(String(data=json.dumps(self.state)));atomic_json(RUN/'mission-status.json',self.state)

def main():
    rclpy.init();n=Inspection()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok() and n.goal_handle:n.goal_handle.cancel_goal_async()
        n.events.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
