import os
os.environ['OMP_NUM_THREADS']='4';os.environ['MKL_NUM_THREADS']='4'
import roslibpy,time,json,pathlib,cv2
from galaxea_r1pro.galaxea_r1pro_0.utils.utils_ImageCapture import decode_color_message
r=pathlib.Path(__file__).resolve().parent
ros=roslibpy.Ros(host='172.16.70.147',port=9091);ros.run(timeout=10)
seen={};messages={};subs=[]
for topic in ['/camera/camera/color/image_raw','/camera/camera/aligned_depth_to_color/image_raw','/camera/camera/depth/image_rect_raw']:
 def cb(m,t=topic):
  seen[t]={'encoding':m.get('encoding'),'data_length':len(m.get('data',[])),'header':m.get('header')}
  if t.endswith('/color/image_raw') and m.get('data') and 'color' not in messages:messages['color']=m
 t=roslibpy.Topic(ros,topic,'sensor_msgs/msg/Image',throttle_rate=500,queue_length=1);t.subscribe(cb);subs.append(t)
time.sleep(10)
for t in subs:t.unsubscribe()
ros.close()
(r/'camera-probe.json').write_text(json.dumps(seen,indent=2));print(json.dumps(seen),flush=True)
if 'color' not in messages:raise RuntimeError('no RGB data')
im=decode_color_message(messages['color']);cv2.imwrite(str(r/'camera.jpg'),im)
from ultralytics import YOLO
m=YOLO('/home/morefine/ws/SEMF/workspace/semantic-robotsdk/galaxea-r1pro-sdk/src/galaxea_r1pro/galaxea_r1pro_0/utils/yolo11x-seg_for_WAIC_0715.pt');start=time.monotonic();prediction=m.predict(im,device='cpu',imgsz=640,conf=.25,iou=.7,verbose=False)[0]
cv2.imwrite(str(r/'rgb-detection.jpg'),prediction.plot())
result={'diagnostic_only':True,'path':'raw RGB -> YOLO (not Ability skill)','inference_seconds':time.monotonic()-start,'objects':[{'class_id':int(b.cls.item()),'name':prediction.names[int(b.cls.item())],'confidence':float(b.conf.item()),'xyxy':b.xyxy[0].tolist()} for b in prediction.boxes]}
(r/'rgb-detection.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
