"""只订阅 TF；将 map 目标转换为检测坐标系，输入输出均为米。"""
import time
import numpy as np
from scipy.spatial.transform import Rotation


def apply_transform(point, transform):
    point = np.asarray(point, dtype=np.float64)
    position = np.asarray(transform['position'], dtype=np.float64)
    quat = np.asarray(transform['orientation'], dtype=np.float64)
    if point.shape != (3,) or position.shape != (3,) or quat.shape != (4,):
        raise ValueError('坐标转换要求 point[3]、position[3]、orientation[4]')
    if not np.isfinite(np.concatenate((point, position, quat))).all() or np.linalg.norm(quat)<1e-8:
        raise ValueError('目标点或 TF 包含无效数值')
    return (point @ Rotation.from_quat(quat).as_matrix().T + position).tolist()


def lookup_transform(source, target, *, host, port, timeout=5.0, max_age_s=1.0):
    import roslibpy
    from galaxea_r1pro.galaxea_r1pro_0.utils.tf_buffer import TransformBuffer
    if source == target:
        return {'position':[0.,0.,0.], 'orientation':[0.,0.,0.,1.]}
    buffer=TransformBuffer()
    ros=roslibpy.Ros(host=host, port=port)
    subscriptions=[]
    try:
        ros.run(timeout=timeout)
        for topic in ('/tf_static','/tf'):
            subscription=roslibpy.Topic(ros, topic, 'tf2_msgs/msg/TFMessage')
            subscription.subscribe(lambda msg, static=(topic=='/tf_static'):buffer.update(msg,static=static))
            subscriptions.append(subscription)
        deadline=time.monotonic()+timeout
        while True:
            try:
                matrix=buffer.lookup(source,target,max_age_s)
                return {'position':matrix[:3,3].tolist(),
                        'orientation':Rotation.from_matrix(matrix[:3,:3]).as_quat().tolist()}
            except LookupError as exc:
                if time.monotonic()>=deadline:
                    raise ValueError(f'无法将 {source} 目标转换到 {target}：TF 缺失或过期；不能把 body 当作 base_link') from exc
                time.sleep(.05)
    finally:
        for subscription in subscriptions:subscription.unsubscribe()
        if ros.is_connected:ros.close()
