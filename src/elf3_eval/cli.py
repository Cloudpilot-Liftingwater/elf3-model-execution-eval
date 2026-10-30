import argparse
import json
from pathlib import Path
from .contract import clean

def main(argv=None):
    p=argparse.ArgumentParser(description='ELF3 passive model/command/encoder evaluation')
    sub=p.add_subparsers(dest='command',required=True)
    x=sub.add_parser('import',help='stream original JSONL[.gz] into a derived SQLite index')
    x.add_argument('source');x.add_argument('--output',required=True)
    x.add_argument('--source-age-max',type=float,default=1.)
    x.add_argument('--clock-jump-ms',type=float,default=20.)
    x=sub.add_parser('analyze',help='offline analysis; never imports ROS or connects to a robot')
    x.add_argument('database');x.add_argument('--output',required=True)
    for flag in ('command-age-ms','model-age-ms','feedback-gap-ms'):x.add_argument('--'+flag,type=float)
    x.add_argument('--lag-max-ms',type=float,default=250.);x.add_argument('--lag-step-ms',type=float,default=5.)
    x.add_argument('--include-phases',default='active');x.add_argument('--model')
    x.add_argument('--start',type=float);x.add_argument('--end',type=float)
    x.add_argument('--sonic-reference',metavar='BUNDLE_DIR',help='optional offline g1_kin bundle: kinematic_manifest.json and g1_kin_step28800.onnx')
    x.add_argument('--acceptance-profile',help='elf3-dynamic-screening-v1 or a JSON profile; engineering screening, no control action')
    x=sub.add_parser('record',help='passive ROS/socket subscriptions, no control publication')
    x.add_argument('--output',required=True);x.add_argument('--seconds',type=float,default=60.,help='duration; 0 records until SIGINT/SIGTERM')
    x.add_argument('--address',default='elf3_model_eval_v1');x.add_argument('--socket-only',action='store_true')
    x.add_argument('--robot-id',help='physical robot serial/test identifier; never inferred from the recording workstation hostname')
    x.add_argument('--legacy-sonic',action='store_true')
    for name,value in (('state-topic','/hardware/actuator_states'),('command-topic','/hardware/actuators_cmds'),('mode-topic','/hardware/state_machine_info')):x.add_argument('--'+name,default=value)
    x=sub.add_parser('check',help='local dependency and contract check; no robot connection')
    x.add_argument('--ros',action='store_true')
    args=p.parse_args(argv); kw=vars(args).copy();cmd=kw.pop('command')
    try:
        if cmd=='import':
            from .storage import import_recording
            kw['clock_jump']=kw.pop('clock_jump_ms')/1000
            result=import_recording(**kw)
        elif cmd=='analyze':
            from .analysis import analyze,Options
            database=kw.pop('database');output=kw.pop('output');kw['include_phases']=tuple(kw['include_phases'].split(','))
            result=analyze(database,output,Options(**kw))['summary']
        elif cmd=='record':
            from .record import record
            result=record(**kw)
        else:
            from .contract import BODY
            import numpy
            result=dict(body_joints=len(BODY),numpy=numpy.__version__,offline_ready=True,hardware_commands_sent=0)
            if args.ros:
                import rclpy
                from communication.msg import ActuatorStates,ActuatorCmds
                result['ros_imports_ready']=True
        print(json.dumps(clean(result),ensure_ascii=False,indent=2,allow_nan=False))
        if cmd=='record' and (result['writer_errors'] or result['queue_lost'] or result['callback_errors'] or not result['written']):return 3
        return 0
    except (ValueError,OSError,ImportError) as exc:p.exit(2,str(exc)+'\n')

if __name__=='__main__':raise SystemExit(main())
