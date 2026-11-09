import pathlib,json,time,urllib.request,subprocess
r=pathlib.Path(__file__).resolve().parents[3]
for _ in range(40):
 h=json.load(urllib.request.urlopen('http://127.0.0.1:8080/api/ability-heartbeat'))
 if any(x.get('abilityName')=='YoloDetect.Demo' and x.get('state')=='Running' and x.get('abilityPort',0)>0 for x in h):break
 time.sleep(1)
else:raise RuntimeError('YOLO instance not ready')
time.sleep(2)
p=subprocess.run([str(r/'start-shell/run_yolo_detect_skill.sh'),'inputs/yolo_detect_match.json'],cwd=r,capture_output=True,text=True,timeout=180)
(r/'test/single-skill/yolo_runtime/body-match.log').write_text(p.stdout+'\n'+p.stderr)
print('EXIT',p.returncode,flush=True)
