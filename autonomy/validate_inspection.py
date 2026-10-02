"""Extended acceptance run: full inspection, inaccessible goal, cancellation."""
import json
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from .control import Controls,LATEST,read

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Controls()
    report=dict(started_wall=time.time(),run=str(LATEST.resolve()),checks={})
    def record(name,passed,details):
        report['checks'][name]=dict(passed=bool(passed),details=details)
        (LATEST/'inspection-validation.json').write_text(json.dumps(report,indent=2))
        print(f"{'PASS' if passed else 'FAIL'} {name}: {details}",flush=True)
        if not passed:raise AssertionError(name)
    def wait_terminal(seconds):
        end=time.monotonic()+seconds;last_log=0.
        while time.monotonic()<end:
            node.wait(.5);state=read('mission-status.json')
            if time.time()-read('edge-status.json').get('updated_wall',0)>5:raise RuntimeError('The simulation stack stopped or became unresponsive')
            if state.get('status') in ('SUCCEEDED','FAILED','CANCELED'):return state
            if time.monotonic()-last_log>30:
                print(f"{state.get('route')}: checkpoint {state.get('index',0)+1}, {state.get('distance_remaining_m',0):.1f} m remaining",flush=True);last_log=time.monotonic()
        raise TimeoutError('Mission did not terminate within validation deadline')
    try:
        if read('mission-status.json').get('status') in ('RUNNING','WAITING_FOR_NAV2','CANCELING'):raise RuntimeError('Finish or cancel the active job first')
        node.block(False);node.wait(3.)
        node.command('start','inspection');state=wait_terminal(1800)
        record('full_inspection',state['status']=='SUCCEEDED',state)
        report['full_inspection_evaluation']=read('evaluation.json')
        report['dead_reckoning_evaluation']=read('dead-reckoning-evaluation.json')
        node.save()
        node.block(True,x=-12.,y=-47.,width=6.);node.wait(3.)
        node.command('start','quick');state=wait_terminal(180)
        record('inaccessible_goal_reported',state['status']=='FAILED' and state['checkpoints'][0]['status']=='UNREACHABLE',state)
        node.wait(2.)
        record('stopped_after_failure',abs(read('twin-status.json')['state']['speed'])<.03,read('twin-status.json')['drive_status'])
        node.block(False);node.wait(4.)
        node.command('start','inspection');node.wait(4.);node.command('cancel');state=wait_terminal(15);node.wait(2.)
        record('cancel_stops_active_job',state['status']=='CANCELED' and abs(read('twin-status.json')['state']['speed'])<.03,state)
        report['passed']=True
    except (Exception,KeyboardInterrupt) as error:
        report['passed']=False;report['error']=str(error) or 'Interrupted';raise
    finally:
        report['finished_wall']=time.time();(LATEST/'inspection-validation.json').write_text(json.dumps(report,indent=2))
        if rclpy.ok():
            try:
                if read('mission-status.json').get('status') in ('RUNNING','WAITING_FOR_NAV2'):node.command('cancel')
            except Exception as error:print(f'Cleanup: {error}',flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
