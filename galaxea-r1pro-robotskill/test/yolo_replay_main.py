"""Test-only Ability entrypoint: real model/geometry, saved frames, no ROS connection."""
import json,os
from pathlib import Path
from datetime import datetime,timezone
import cv2,numpy as np
from galaxea_r1pro_0.abilities.yolo_detect.handler import ObjectPerceptionHandler
from galaxea_r1pro_0.service.entrypoint import run_ability,RuntimeConfig
from galaxea_r1pro_0.service.roles import AbilityRole
from galaxea_r1pro import RobotProfile
from galaxea_r1pro.galaxea_r1pro_0.utils.types import SensorFrame


def install_replay():
    config=RuntimeConfig.load('YoloDetect.Replay')
    profile=RobotProfile.load(config.robot_profile_path)
    if profile.options.get('test_mode') is not True:
        raise RuntimeError('Replay requires test_mode=true; refusing to connect to robot')
    source=Path(os.environ['YOLO_REPLAY_DIR']).resolve()
    record=json.loads((source/'result.json').read_text())
    image=cv2.imread(str(source/'rgb.png'));depth=np.load(source/'depth.npy')
    if image is None or depth.shape!=image.shape[:2]:raise ValueError('Invalid recorded RGB-D')
    frame=record['result']['camera_frame']
    def capture(self,sensor_id):
        if self.sdk._backend.profile.options.get('test_mode') is not True:raise RuntimeError('Replay backend changed')
        if sensor_id!=record['result']['sensor_id']:raise ValueError('Replay sensor_id mismatch')
        now=datetime.now(timezone.utc)
        rgbmeta={**record['rgb_metadata'],'replay_source':str(source),'data_mode':'replay'}
        depthmeta={**record['depth_metadata'],'replay_source':str(source),'data_mode':'replay'}
        return (SensorFrame(sensor_id,frame,now,'rgb8',image.copy(),rgbmeta),
                SensorFrame(sensor_id,frame,now,'16UC1',depth.copy(),depthmeta))
    ObjectPerceptionHandler.capture=capture

if __name__=='__main__':
    install_replay()
    run_ability(AbilityRole.yolo_detect,'YoloDetect.Replay')
