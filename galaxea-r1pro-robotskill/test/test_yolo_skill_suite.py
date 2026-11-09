"""Live perception regression through Pilot skills; per-case evidence and bounded retries."""
import argparse,datetime,json,os,subprocess,time,uuid
import urllib.request
from pathlib import Path
import yaml

TRANSIENT=('时间差超过','消息超时','message_timeout','Failed to connect to ROS')

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--reference',type=Path,help='prepare_yolo_reference.py 生成的 reference.json')
    parser.add_argument('--attempts',type=int,default=3,choices=range(1,6))
    parser.add_argument('--skip-map-target',action='store_true')
    parser.add_argument('--replay',action='store_true',help='仅使用 YoloDetect.Replay；不读取实时 TF')
    parser.add_argument('--gray-box-only',action='store_true')
    parser.add_argument('--top-k',type=int,default=2)
    args=parser.parse_args()
    if args.top_k < 1:parser.error('--top-k must be positive')
    root=Path(__file__).resolve().parent
    if args.replay:
        os.environ.setdefault('PROFILE',str(root/'../src/config/yolo-replay-deployment.yaml'))
    run=root/'../outputs'/('yolo-suite-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]);run.mkdir(parents=True)
    expected_ability='YoloDetect.Replay' if args.replay else 'YoloDetect.Demo'
    report={'started_at':datetime.datetime.now().isoformat(),'scope':'four business tasks through skill/Pilot',
        'data_mode':'saved RGB-D replay' if args.replay else 'live camera',
        'calibration_accuracy_verified':False,'cases':[]}
    reference=json.loads(args.reference.read_text()) if args.reference else None
    if reference:(run/'reference.json').write_text(json.dumps(reference,indent=2))
    def save():
        (run/'summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    # A listening framework is insufficient: the selected Ability must be connected.
    try:
        profile=Path(os.environ.get('PROFILE',str(root/'../src/config/yolo-robot-deployment.yaml')))
        endpoint=yaml.safe_load(profile.read_text())['ability_framework']['endpoint'].rstrip('/')
        with urllib.request.urlopen(endpoint+'/api/ability-heartbeat',timeout=5) as response:
            heartbeat=json.load(response)
        ready=[h for h in heartbeat if h.get('abilityName')==expected_ability and h.get('state')=='Running' and h.get('abilityPort',0)>0]
        if not ready:raise RuntimeError('YOLO Ability 尚未连接：需要 Running 且 abilityPort > 0')
        other=[h for h in heartbeat if h.get('abilityName','').startswith('YoloDetect.') and h.get('abilityName')!=expected_ability and h.get('state')=='Running']
        if other:raise RuntimeError('存在另一种活动 YOLO 实例，无法保证回放/实机路由唯一')
        report['ready_instances']=[h['id'] for h in ready]
        with urllib.request.urlopen(endpoint+'/api/manifest/'+expected_ability+'/1.0.0',timeout=5) as response:
            manifest=json.load(response)
        if 'perception.estimate_object_pose' not in json.dumps(manifest):
            raise RuntimeError('YOLO manifest 缺少姿态估计动作绑定')
    except Exception as exc:
        report['preflight']={'status':'blocked','error':str(exc)};save()
        print('PREFLIGHT BLOCKED',exc,'SUMMARY',run/'summary.json',flush=True)
        return 2
    def execute(name,data):
        folder=run/name;folder.mkdir();inp=folder/'input.json';inp.write_text(json.dumps(data,indent=2))
        case={'name':name,'input':str(inp),'attempts':[]};report['cases'].append(case)
        output={};reading={}
        for attempt in range(args.attempts):
            dest=folder/str(attempt+1);dest.mkdir();start=time.perf_counter()
            try:
                p=subprocess.run([str(root/'../start-shell/run_yolo_detect_skill.sh'),str(inp)],env={**os.environ,'OUTPUT_ROOT':str(dest)},capture_output=True,text=True,timeout=300)
                (dest/'console.log').write_text(p.stdout+'\n'+p.stderr);code=p.returncode
            except subprocess.TimeoutExpired as exc:
                (dest/'console.log').write_text(str(exc));code=124
            elapsed=time.perf_counter()-start
            results=list(dest.glob('yolo-detect.*/result.json'))
            document=json.loads(results[0].read_text()) if results else {}
            output=document.get('result') or {};reading=(output.get('readings') or [{}])[0]
            error=reading.get('error') or document.get('error') or (None if code==0 else 'Pilot/script did not finish successfully; see console.log')
            status='passed' if code==0 and output.get('overall_status')=='succeeded' else 'failed'
            entry={'status':status,'exit_code':code,'wall_seconds':elapsed,'error':error,
                'skill_elapsed_seconds':reading.get('elapsed_seconds'),'object_count':reading.get('object_count'),
                'result_path':str(results[0]) if results else None}
            case['attempts'].append(entry);case['status']=status;case['error']=error
            case['total_wall_seconds']=sum(x['wall_seconds'] for x in case['attempts']);save()
            print(f'{name} attempt {attempt+1}: {status}, {elapsed:.3f}s, {error or "OK"}',flush=True)
            if status=='passed' or not any(s in str(error) for s in TRANSIENT):break
        return reading.get('result') or {},case
    common={'sensor_id':'head_camera','conf_threshold':.25,'iou_threshold':.7,'timeout_seconds':180,
            'gray_box_only':args.gray_box_only,'top_k':args.top_k}
    detection,detect_case=execute('01_yolo_detect',{**common,'mode':'detect','require_objects':True})
    cloud,cloud_case=execute('02_generate_point_cloud',{**common,'mode':'point_cloud','require_objects':True})
    objects=cloud.get('objects',[])
    selected=sorted(objects,key=lambda o:(o.get('class_name')!='gray_box',-o.get('confidence',0)))[:1]
    target=selected[0]['center_robot'] if selected else (reference or {}).get('reference_target_point')
    frame=cloud.get('robot_frame') or (reference or {}).get('reference_camera',{}).get('robot_frame')
    if target and frame:
        report['selected_target']={'source':('replay point cloud' if args.replay else 'live point cloud') if selected else 'saved reference fallback','frame':frame,'point_m':target}
        target_input={**common,'target_point':target,'target_input_frame':frame,'target_frame':frame,'trans_flag':False}
        execute('03_match_target',{**target_input,'mode':'match'})
        if reference:
            execute('04_estimate_object_pose',{**target_input,**reference,'mode':'pose'})
        else:report['cases'].append({'name':'04_estimate_object_pose','status':'blocked','error':'Provide --reference with a calibrated reference capture'})
    else:
        for name in ['03_match_target','04_estimate_object_pose']:
            report['cases'].append({'name':name,'status':'blocked','error':'No live 3D candidate or saved reference target; refusing to invent coordinates'})
    if not args.skip_map_target and not args.replay:
        map_input=json.loads((root/'../inputs/yolo_detect_match.json').read_text())
        execute('05_match_original_map_target',map_input)
    # Lifecycle controls are tested separately against only our completed perception record.
    try:
        from yolo_suite_lifecycle import check_lifecycle
        source=next((a['result_path'] for c in [detect_case,cloud_case] for a in c['attempts'] if a['status']=='passed'),None)
        report['lifecycle']=check_lifecycle(Path(source).parent,root,run,expected_ability) if source else {'status':'blocked','error':'No successful owned YOLO invocation'}
    except Exception as exc:report['lifecycle']={'status':'failed','error':str(exc)}
    report['finished_at']=datetime.datetime.now().isoformat();save()
    print('SUMMARY',run/'summary.json',flush=True)
    return 0 if all(c['status']=='passed' for c in report['cases']) and report['lifecycle']['status']=='passed' else 3

if __name__=='__main__':raise SystemExit(main())
