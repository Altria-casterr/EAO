import os,sys,json,pathlib,time,datetime
os.environ['OMP_NUM_THREADS']='4';os.environ['MKL_NUM_THREADS']='4'
r=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(r/'test/single-skill/yolo_runtime/src'))
import cv2,numpy as np
from galaxea_r1pro import R1ProSDK,RobotProfile
from galaxea_r1pro_0.service.models import ModelProfileRegistry
from galaxea_r1pro_0.abilities.yolo_detect.handler import ObjectPerceptionHandler
from galaxea_r1pro_0.tasks.yolo_detect import GeneratePointCloudTask
output=r/'outputs'/('yolo-overlay-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));output.mkdir()
print('OUTPUT',str(output),flush=True)
class RecordedHandler(ObjectPerceptionHandler):
 def _segment(self,data,*,geometry):
  result=super()._segment(data,geometry=geometry)
  self.recorded=result
  return result
models=ModelProfileRegistry.load(str(r/'test/single-skill/yolo_runtime/src/galaxea_r1pro_0/config/models.example.json'))
sdk=R1ProSDK.open(RobotProfile.load(str(r/'test/single-skill/yolo_runtime/src/galaxea_r1pro_0/config/r1pro-yolo-test.json')))
h=RecordedHandler(sdk,models,models.cameras)
try:
 task=GeneratePointCloudTask(sdk,models=models,handler=h)
 for attempt in range(5):
  try:
   result=task.execute({'sensor_id':'head_camera','conf_threshold':.25,'iou_threshold':.7});break
  except Exception as e:
   if not ('时间差' in str(e) or '消息超时' in str(e)) or attempt==4:raise
   print('capture retry',str(e),flush=True);time.sleep(.3)
 rgb,depth,detected,geometry=h.recorded
 cv2.imwrite(str(output/'rgb.png'),rgb.data)
 np.save(output/'depth.npy',depth.data)
 objects=result['result']['objects']
 for o in objects:o['box_xyxy']=detected[o['index']].box_xyxy
 payload={'captured_at':datetime.datetime.now().isoformat(),'source':'direct GeneratePointCloudTask; same Ability code and configs, with RGB retention','rgb_metadata':rgb.metadata,'depth_metadata':depth.metadata,'result':result['result']}
 (output/'result.json').write_text(json.dumps(payload,indent=2,ensure_ascii=False))
 (r/'test/single-skill/yolo_runtime/latest-overlay-path.txt').write_text(str(output))
 print('DONE',len(objects),flush=True)
finally:
 h.close();sdk.close()
