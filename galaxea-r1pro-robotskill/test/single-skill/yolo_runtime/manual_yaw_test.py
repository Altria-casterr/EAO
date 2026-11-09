import pathlib,subprocess,json
r=pathlib.Path(__file__).resolve().parents[3]
summary=[]
for degrees in [0,180]:
 name=f'yolo_detect_match_manual_yaw{degrees}.json'
 result=subprocess.run([str(r/'start-shell/run_yolo_detect_skill.sh'),'inputs/'+name],cwd=r,capture_output=True,text=True,timeout=180)
 log=r/'test/single-skill/yolo_runtime'/f'manual-yaw{degrees}.log';log.write_text(result.stdout+'\n'+result.stderr)
 summary.append({'yaw_degrees':degrees,'exit_code':result.returncode,'log':str(log)})
 print(json.dumps(summary[-1]),flush=True)
 if result.returncode==0:break
(r/'test/single-skill/yolo_runtime/manual-yaw-summary.json').write_text(json.dumps(summary,indent=2))
