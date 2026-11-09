import pathlib,subprocess,time,json,re,datetime
r=pathlib.Path(__file__).resolve().parents[3]
started=datetime.datetime.now().isoformat();start=time.perf_counter()
p=subprocess.run([str(r/'start-shell/run_yolo_detect_skill.sh'),'inputs/yolo_detect_point_cloud.json'],cwd=r,capture_output=True,text=True,timeout=180)
elapsed=time.perf_counter()-start
(r/'test/single-skill/yolo_runtime/end-to-end-20260928.log').write_text(p.stdout+'\n'+p.stderr)
m=re.search(r'测试结果目录: (.+)',p.stdout)
report={'started_at':started,'wall_seconds':elapsed,'exit_code':p.returncode,'scope':'shell script preflight + Pilot/skill worker + real RGB-D capture + YOLO + point cloud + result writing','run_dir':m.group(1) if m else None}
if m:
 resultfile=pathlib.Path(m.group(1))/'result.json'
 if resultfile.exists():
  result=json.loads(resultfile.read_text());report['created_at']=result.get('created_at');report['updated_at']=result.get('updated_at');report['overall_status']=result.get('result',{}).get('overall_status');report['readings']=[{'status':x['status'],'error':x['error'],'object_count':x.get('object_count')} for x in result.get('result',{}).get('readings',[])]
(r/'test/single-skill/yolo_runtime/end-to-end-20260928.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
