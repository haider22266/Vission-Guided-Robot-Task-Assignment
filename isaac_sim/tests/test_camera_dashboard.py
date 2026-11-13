from concurrent.futures import Future
import io
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_dashboard import CameraDashboard
from environment_cdd_runtime import SceneResearchRuntime


class FakeRGB:
    shape=(224,224,4)
    size=224*224*4
    def __getitem__(self,key):return self
    def copy(self):return self


def test_live_camera_updates_while_classifier_is_busy_without_queuing_frames():
    runtime=SceneResearchRuntime.__new__(SceneResearchRuntime)
    runtime.disabled=False
    pending=Future()
    runtime.future=pending
    runtime.last=-10.
    runtime.frames={1:0}
    runtime.sensors={1:SimpleNamespace(get_data=lambda:FakeRGB())}
    displayed=[]
    states=[]
    runtime.dashboard=SimpleNamespace(frame=lambda *args:displayed.append(args),update_status=states.append)
    runtime.pool=SimpleNamespace(submit=lambda *args:(_ for _ in ()).throw(AssertionError('Cannot enqueue behind a pending batch')))
    runtime.update(4.)
    runtime.update(5.)
    assert runtime.future is pending
    assert runtime.frames[1]==2 and len(displayed)==2
    assert states==[True,True]


def test_prediction_collection_keeps_its_original_timestamp():
    runtime=SceneResearchRuntime.__new__(SceneResearchRuntime)
    runtime.disabled=False
    result={'robot_id':1,'source':'efficientnet','predicted_class':'Fire Disaster','confidence':.8,'simulation_time':1.}
    runtime.future=Future()
    runtime.future.set_result([result])
    runtime.last=5.
    runtime.results={}
    runtime.log=io.StringIO()
    displayed=[]
    runtime.dashboard=SimpleNamespace(result=displayed.append)
    runtime.update(5.1)
    assert runtime.future is None
    assert displayed==[result]
    assert runtime.results[1]['simulation_time']==1.


def test_dashboard_labels_camera_prediction_times_and_mock_source_separately():
    dashboard=CameraDashboard.__new__(CameraDashboard)
    dashboard.results={1:{'predicted_class':'Non-Damage','confidence':1.,'simulation_time':1.,
                          'hazard_risk':0.,'rescue_signal':0.,'source':'mock'}}
    dashboard.frame_times={1:5.}
    dashboard.records={1:{'task':'Arrange carton','state':'navigate_to_original_prop'}}
    dashboard.labels={1:SimpleNamespace(text='')}
    dashboard.tasks={1:SimpleNamespace(text='')}
    dashboard.refresh(1)
    assert 'Camera @ 5.0s' in dashboard.labels[1].text
    assert 'result @ 1.0s' in dashboard.labels[1].text
    assert 'MOCK TEST' in dashboard.labels[1].text
    assert 'Arrange carton' in dashboard.tasks[1].text


def test_disabled_cameras_need_no_isaac_imports_workers_or_checkpoint(tmp_path):
    runtime=SceneResearchRuntime(None,[{'id':1}],SimpleNamespace(perception_backend='off',tasks_per_robot=0))
    runtime.update(1.)
    assert runtime.priority(1)==.5
    summary=runtime.summary()
    assert summary['camera_frames']=={}
    assert summary['vision_log'] is None
    assert not summary['visualization']['enabled']
    assert not hasattr(runtime,'process') and not hasattr(runtime,'sensors')
    runtime.close()
