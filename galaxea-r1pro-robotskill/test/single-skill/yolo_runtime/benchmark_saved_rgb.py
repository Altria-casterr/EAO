import os
os.environ['OMP_NUM_THREADS']='4';os.environ['MKL_NUM_THREADS']='4'
import pathlib,time,json,cv2
from ultralytics import YOLO
r=pathlib.Path(__file__).resolve().parents[3]
p=pathlib.Path((r/'test/single-skill/yolo_runtime/latest-overlay-path.txt').read_text().strip());im=cv2.imread(str(p/'rgb.png'))
cfg=json.loads((r/'test/single-skill/yolo_runtime/src/galaxea_r1pro_0/config/models.example.json').read_text());name=cfg['defaults']['yolo_detect'];settings=cfg['profiles'][name]['settings']
t=time.perf_counter();model=YOLO(settings['checkpoint_path']);load=time.perf_counter()-t
runs=[]
for i in range(4):
 t=time.perf_counter();result=model.predict(im,device=settings.get('device'),imgsz=settings.get('imgsz',896),conf=.25,iou=.7,retina_masks=True,verbose=False)[0];elapsed=time.perf_counter()-t
 runs.append({'run':i,'predict_wall_ms':elapsed*1000,'speed_ms':result.speed,'detections':len(result.boxes)})
report={'source_rgb':str(p/'rgb.png'),'settings':settings,'torch_threads_requested':4,'model_load_ms':load*1000,'runs':runs,'warm_mean_inference_ms':sum(x['speed_ms']['inference'] for x in runs[1:])/3,'warm_mean_predict_wall_ms':sum(x['predict_wall_ms'] for x in runs[1:])/3}
(p/'inference_benchmark.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
