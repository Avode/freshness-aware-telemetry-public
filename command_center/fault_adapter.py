"""Explicit simulation-only faults upstream of autonomy and passive monitoring."""
import copy
import rclpy
from std_srvs.srv import SetBool
from milestone.adapter import Adapter


class FaultAdapter(Adapter):
    def __init__(self):
        self.faults = dict(camera_freeze=False, imu_dropout=False, lidar_dropout=False)
        self.cached_image = None
        super().__init__()
        for name in self.faults:
            self.create_service(SetBool, '/simulation/'+name, lambda req, res, name=name: self.toggle(name, req, res))

    def toggle(self, name, request, response):
        self.faults[name] = request.data
        response.success = True; response.message = f'{name}: {"active" if request.data else "cleared"}'
        return response

    def image(self, msg):
        if not self.faults['camera_freeze']: self.cached_image = copy.deepcopy(msg)
        if self.cached_image is not None: super().image(copy.deepcopy(self.cached_image))

    def imu(self, msg):
        if not self.faults['imu_dropout']: super().imu(msg)

    def scan(self, msg):
        if not self.faults['lidar_dropout']: super().scan(msg)


def main():
    rclpy.init(); node = FaultAdapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
