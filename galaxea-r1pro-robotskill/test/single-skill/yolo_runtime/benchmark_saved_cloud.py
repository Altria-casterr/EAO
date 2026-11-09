import os
os.environ['OMP_NUM_THREADS']='4';os.environ['MKL_NUM_THREADS']='4'
import pathlib,sys,json,time,types
r=pathlib.Path(__file__).resolve().parents[3];sys.path.insert(0,str(r/'test/single-skill/yolo_runtime/src'))
import cv2,numpy as np
from galaxea_r1pro_0.service.models import ModelProfileRegistry
from galaxea_r1pro_0.abilities.yolo_detect.handler import ObjectPerceptionHandler
from galaxea_r1pro_0.abilities.yolo_detect.camera_geometry import resolve_geometry
from galaxea_r1pro_0.tasks.yolo_detect import _clouds,_public_object
p=pathlib.Path((r/'test/single-skill/yolo_runtime/latest-overlay-path.txt').read_text().strip());rgb=cv2.imread(str(p/'rgb.png'));depth=types.SimpleNamespace(data=np.load(p/'depth.npy'))
models=ModelProfileRegistry.load(str(r/'test/single-skill/yolo_runtime/src/galaxea_r1pro_0/config/models.example.json'));handler=ObjectPerceptionHandler(None,models,models.cameras)
data=handler.prepare({'sensor_id':'head_camera','conf_threshold':.25,'iou_threshold':.7});runs=[]
for i in range(4):
 start=time.perf_counter();geometry=resolve_geometry(None,data,'camera_color_optical_frame',allow_dynamic=False);t1=time.perf_counter()
 detected=handler.segment_image(rgb,data);t2=time.perf_counter()
 clouds=_clouds(depth,detected,geometry,data);t3=time.perf_counter()
 public=[_public_object(o) for o in clouds];json.dumps(public);end=time.perf_counter()
 runs.append({'run':i,'geometry_ms':(t1-start)*1000,'yolo_call_ms':(t2-t1)*1000,'pointcloud_ms':(t3-t2)*1000,'serialization_ms':(end-t3)*1000,'total_ms':(end-start)*1000,'objects':len(clouds),'filter_only_sum_ms':sum(o['filter_stats']['elapsed_ms'] for o in clouds)})
 print(json.dumps(runs[-1]),flush=True)
report={'source_dir':str(p),'device':'cpu','imgsz':640,'mode':'same Ability processing functions on saved RGB-D, excludes capture, TF query and skill/Pilot','runs':runs,'warm_mean_ms':{k:sum(row[k] for row in runs[1:])/3 for k in ['geometry_ms','yolo_call_ms','pointcloud_ms','serialization_ms','total_ms','filter_only_sum_ms']}}
(p/'pointcloud_benchmark.json').write_text(json.dumps(report,indent=2));print('REPORT',json.dumps(report),flush=True);handler.close()
