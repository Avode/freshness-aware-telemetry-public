from collections import OrderedDict
import json
import queue
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import uuid
from .gateway import Gateway, mqtt
from . import protocol


class GatewayBoundaryTests(unittest.TestCase):
    def node(self):
        connected = threading.Event(); connected.set()
        return SimpleNamespace(run_id='run', pending=queue.Queue(), forwarded=OrderedDict(),
            connected=connected, last_heartbeat=time.monotonic(), ack_pub=Mock(), command_pub=Mock(),
            client=Mock(publish=Mock(return_value=SimpleNamespace(rc=mqtt.MQTT_ERR_SUCCESS))),
            stats=dict(telemetry_published=0,mac_acks=0,commands_forwarded=0,rejected=0,last_mac_ack_wall=None))

    def test_broker_publish_success_never_generates_robot_receipt_ack(self):
        node=self.node()
        packet=dict(schema=1,robot_id=protocol.ROBOT,run_id='run',seq=3,stamp=1.,state=dict(pose=[0.,0.,0.]))
        Gateway.receive_telemetry(node, SimpleNamespace(data=json.dumps(packet)))
        node.ack_pub.publish.assert_not_called()
        self.assertIn(3,node.forwarded)
        Gateway.receive_telemetry(node, SimpleNamespace(data=json.dumps(packet)))
        node.client.publish.assert_called_once()

    def test_only_valid_mac_ack_for_forwarded_packet_is_relayed(self):
        node=self.node(); node.forwarded[3]=time.monotonic()
        ack=dict(schema=1,robot_id=protocol.ROBOT,run_id='run',seq=3)
        for value,retained in [(dict(ack,seq=4),False),(dict(ack,run_id='old'),False),(ack,True),(ack,False)]:
            node.pending.put((protocol.PREFIX+'telemetry_ack',json.dumps(value).encode(),retained))
        Gateway.drain(node)
        node.ack_pub.publish.assert_called_once()
        self.assertEqual(json.loads(node.ack_pub.publish.call_args.args[0].data),{'run_id':'run','seq':3})
        self.assertEqual(node.stats['rejected'],3)

    def test_stale_session_retained_and_expired_commands_do_not_dispatch(self):
        node=self.node(); now=time.time()
        cmd=dict(request_id=str(uuid.uuid4()),robot_id=protocol.ROBOT,run_id='run',action='cancel',expires_wall=now+8.)
        node.pending.put((protocol.PREFIX+'commands',json.dumps(cmd).encode(),True))
        node.pending.put((protocol.PREFIX+'commands',json.dumps(dict(cmd,expires_wall=now-1.)).encode(),False))
        Gateway.drain(node); node.command_pub.publish.assert_not_called()
        node.last_heartbeat=time.monotonic()-5.
        node.pending.put((protocol.PREFIX+'commands',json.dumps(cmd).encode(),False))
        Gateway.drain(node); node.command_pub.publish.assert_not_called()
        node.last_heartbeat=time.monotonic()
        node.pending.put((protocol.PREFIX+'commands',json.dumps(cmd).encode(),False))
        Gateway.drain(node); node.command_pub.publish.assert_called_once()


if __name__ == '__main__':
    unittest.main()
