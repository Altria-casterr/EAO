"""Prepare a frozen reference from a recorded, synchronized RGB-D capture."""
import argparse,json,shutil
from pathlib import Path
import cv2,numpy as np

def prepare(source, destination):
    saved=json.loads((source/'result.json').read_text())['result']
    objects=[o for o in saved['objects'] if o['class_name']=='gray_box']
    if not objects:raise ValueError('参考数据没有 gray_box，不能自动指定参考物体')
    selected=max(objects,key=lambda o:o['confidence'])
    destination.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(source/'rgb.png',destination/'zero_color_s_box.png')
    depth=np.load(source/'depth.npy')
    if depth.dtype!=np.uint16 or depth.ndim!=2:
        raise ValueError('参考 PNG 要求原始 uint16 深度，禁止隐式改变深度单位')
    if not cv2.imwrite(str(destination/'zero_depth_s_box.png'),depth):raise IOError('写深度 PNG 失败')
    config={'reference_dir':str(destination.resolve()),'reference_class_name':'gray_box',
        'reference_target_point':selected['center_robot'],
        'reference_camera':{'camera_frame':saved['camera_frame'],'base_frame':saved['robot_frame'],
            'robot_frame':saved['robot_frame'],'camera_intrinsics':saved['intrinsics'],
            'depth_scale':saved['depth_scale'],'transform_camera_to_robot':saved['transform_camera_to_robot']}}
    (destination/'reference.json').write_text(json.dumps(config,indent=2))
    (destination/'provenance.json').write_text(json.dumps({'source_capture':str(source.resolve()),
        'selected_object':selected,'purpose':'pipeline test, not pose accuracy ground truth'},indent=2))
    return destination/'reference.json'

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('destination',type=Path);args=p.parse_args()
    print(prepare(args.source,args.destination))
