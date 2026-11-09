import sys,json,pathlib,subprocess,numpy as np
from scipy.spatial.transform import Rotation
r=pathlib.Path(__file__).resolve().parents[3];sys.path.insert(0,str(r/'test/single-skill'))
from yolo_detect_native.scripts.coordinates import lookup_transform
pose=lookup_transform('body','map',host='172.16.70.147',port=9091,timeout=20,max_age_s=1)
angles=Rotation.from_quat(pose['orientation']).as_euler('xyz',degrees=True)
original=json.loads((r/'inputs/yolo_detect_match.json').read_text());target=np.asarray(original['target_point'])
evidence={'actual_body_in_map':pose,'actual_rpy_degrees':angles.tolist(),'target_map_m':target.tolist(),'trials':[]}
for yaw in [90,-90]:
 rpy=angles.copy();rpy[2]=yaw
 rotation=Rotation.from_euler('xyz',rpy,degrees=True).as_matrix()
 converted=(target-np.asarray(pose['position']))@rotation
 name='yolo_detect_match_manual_yaw'+str(yaw)+'.json'
 data={**original,'target_input_frame':'body','target_frame':'body','trans_flag':False,'target_point':converted.tolist()}
 (r/'inputs'/name).write_text(json.dumps(data,indent=2))
 result=subprocess.run([str(r/'start-shell/run_yolo_detect_skill.sh'),'inputs/'+name],cwd=r,capture_output=True,text=True,timeout=180)
 (r/'test/single-skill/yolo_runtime'/('manual-yaw'+str(yaw)+'.log')).write_text(result.stdout+'\n'+result.stderr)
 evidence['trials'].append({'yaw_degrees':yaw,'converted_point_m':converted.tolist(),'exit_code':result.returncode})
 (r/'test/single-skill/yolo_runtime/manual-yaw90-evidence.json').write_text(json.dumps(evidence,indent=2))
 print(json.dumps(evidence['trials'][-1]),flush=True)
