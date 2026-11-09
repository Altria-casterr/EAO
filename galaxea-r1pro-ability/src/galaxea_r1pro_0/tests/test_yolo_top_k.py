from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from galaxea_r1pro_0.tasks.yolo_detect import _clouds, _selection_options, YoloUtils


def test_label_filter_depth_order_and_no_unselected_cloud_work(monkeypatch):
    depth = np.array([[100, 300, 200, 400, 0, np.nan]], dtype=float)
    labels = ['gray_box_p','gray_box','gray_box','gray_box','gray_box','gray_box']
    objects = [NS(mask=np.eye(6, dtype=bool)[i:i+1],class_name=name,class_id=i,confidence=.9)
               for i,name in enumerate(labels)]
    generate=Mock(return_value={})
    monkeypatch.setattr(YoloUtils,'generate_point_cloud',generate)
    geometry=NS(depth_scale=.001,intrinsics=dict(fx=1e12,fy=1e12,cx=0,cy=0),transform=None)
    result=_clouds(NS(data=depth),objects,geometry,{'gray_box_only':True,'refine_mask':False})
    assert [o['index'] for o in result] == [2,1]
    assert [o['median_depth_m'] for o in result] == [.2,.3]
    assert generate.call_count == 2
    generate.reset_mock()
    result=_clouds(NS(data=depth),objects,geometry,{'refine_mask':False,'top_k':1})
    assert [o['index'] for o in result] == [0]
    assert generate.call_count == 1
    depth[:]=200
    result=_clouds(NS(data=depth),objects,geometry,{'gray_box_only':True,'refine_mask':False})
    assert [o['index'] for o in result] == [1,2]
    depth[:]=0
    generate.reset_mock()
    assert _clouds(NS(data=depth),objects,geometry,{}) == []
    generate.assert_not_called()


@pytest.mark.parametrize('data',[{'top_k':0},{'top_k':-1},{'top_k':True},
    {'top_k':2.5},{'top_k':'2'},{'gray_box_only':'false'},{'gray_box_only':1}])
def test_strict_options(data):
    with pytest.raises(ValueError):_selection_options(data)


def test_range_not_z_selects_center_box(monkeypatch):
    depth=np.array([[960,1000,961]],dtype=float)
    objects=[NS(mask=np.eye(3,dtype=bool)[i:i+1],class_name='gray_box',class_id=i,confidence=.9) for i in range(3)]
    geometry=NS(depth_scale=.001,intrinsics=dict(fx=1,fy=1,cx=1,cy=0),transform=None)
    generate=Mock(return_value={})
    monkeypatch.setattr(YoloUtils,'generate_point_cloud',generate)
    result=_clouds(NS(data=depth),objects,geometry,{'gray_box_only':True,'top_k':2,'refine_mask':False})
    assert [o['index'] for o in result] == [1,0]
    assert result[0]['camera_distance_m']==1.0
    assert result[1]['camera_distance_m']==pytest.approx(.96*np.sqrt(2))
    assert generate.call_count==2
