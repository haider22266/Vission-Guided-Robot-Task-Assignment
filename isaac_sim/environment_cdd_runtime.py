"""Cameras and priority scheduling added to an already loaded warehouse stage.

The original controller owns object reservations, navigation, contact checks
and rack placement. This adapter ranks concurrent role-constrained workers;
it is a baseline scheduler, not a learned QMIX policy.
"""
import base64
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path, PureWindowsPath
import subprocess


class SceneResearchRuntime:
    def __init__(self,stage,records,args):
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom
        self.records={r['id']:r for r in records}
        self.results={}
        self.frames={rid:0 for rid in self.records}
        self.sensors={}
        self.products=[]
        self.backend=args.perception_backend
        self.continuous=args.tasks_per_robot==0
        self.last=-10.
        self.process=None
        self.future=None
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.dashboard=None
        self.log_path=args.report.with_suffix('.vision.jsonl')
        self.log_path.parent.mkdir(parents=True,exist_ok=True)
        self.log=self.log_path.open('w',encoding='utf-8')
        for rid,r in self.records.items():
            path=str(r['base'].GetPath())+'/CDD_RGB'
            camera=UsdGeom.Camera.Define(stage,path)
            camera.CreateFocalLengthAttr(14.)
            camera.CreateClippingRangeAttr((.05,100.))
            camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
                Gf.Vec3d(.4,0,1.1),Gf.Vec3d(2.,0,.6),Gf.Vec3d(0,0,1)).GetInverse())
            product=rep.create.render_product(path,(224,224))
            self.products.append(product)
            annotator=rep.AnnotatorRegistry.get_annotator('rgb')
            annotator.attach([product])
            self.sensors[rid]=annotator
        if not args.headless:
            from camera_dashboard import CameraDashboard
            self.dashboard=CameraDashboard(self.records,self.backend)
        if self.backend=='efficientnet':
            worker=Path(__file__).resolve().parents[1]/'scripts/cdd_inference_worker.py'
            def wsl_path(value):
                path=PureWindowsPath(value)
                if len(path.drive)==2 and path.drive[1]==':':
                    return '/mnt/'+path.drive[0].lower()+'/'+path.as_posix()[3:]
                return str(value)
            command=[args.classifier_python,str(worker),'--model',str(args.efficientnet_model.resolve())]
            if args.classifier_wsl:
                command=['wsl.exe','-d',args.classifier_distro,'--','bash',
                         wsl_path(str(worker.with_name('run_cdd_worker.sh'))),wsl_path(args.classifier_python),
                         '--model',wsl_path(str(args.efficientnet_model.resolve()))]
            self.process=subprocess.Popen(command,
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,encoding='utf-8',bufsize=1)
        print('CDD integration: same stage; mounted cameras; '+self.backend+
              (' perception; continuous shared cleanup queue (not QMIX)' if self.continuous else
               ' perception; role-constrained priority baseline (not QMIX)'),flush=True)

    def classify(self,rid,rgb,simulation_time):
        if self.backend=='mock':
            return {'robot_id':rid,'source':'mock','predicted_class':'Non-Damage','confidence':1.,
                    'hazard_risk':0.,'rescue_signal':0.,'simulation_time':simulation_time,
                    'interpretation':'Transport test only; no image classifier executed'}
        request={'robot_id':rid,'shape':list(rgb.shape),'rgb':base64.b64encode(rgb.tobytes()).decode('ascii')}
        self.process.stdin.write(json.dumps(request)+'\n')
        self.process.stdin.flush()
        line=self.process.stdout.readline()
        if not line:
            raise RuntimeError('CDD classifier process exited; check its TensorFlow environment/model')
        result=json.loads(line)
        if 'error' in result:
            raise RuntimeError(result['error'])
        result['robot_id']=rid
        result['simulation_time']=simulation_time
        return result

    def update(self,simulation_time):
        if self.future is not None:
            if self.future.done():
                for result in self.future.result():
                    rid=result['robot_id']
                    self.results[rid]=result
                    if self.dashboard:self.dashboard.result(result)
                    self.log.write(json.dumps(result)+'\n')
                    print('CDD area clue:',rid,result['predicted_class'],round(result['confidence'],3),flush=True)
                self.log.flush()
                self.future=None
        if simulation_time-self.last<1.:
            return
        self.last=simulation_time
        batch=[]
        for rid,annotator in self.sensors.items():
            data=annotator.get_data()
            if not hasattr(data,'shape') or len(data.shape)!=3 or not data.size:
                continue
            rgb=data[:,:,:3].copy()
            self.frames[rid]+=1
            if self.dashboard:self.dashboard.frame(rid,rgb,simulation_time)
            batch.append((rid,rgb,simulation_time))
        if batch and self.future is None:
            self.future=self.pool.submit(lambda:[self.classify(*item) for item in batch])
        if self.dashboard:self.dashboard.update_status(self.future is not None)

    def priority(self,rid):
        result=self.results.get(rid,{})
        return .5+.5*max(result.get('hazard_risk',0.),result.get('rescue_signal',0.))

    def summary(self):
        return {'perception_backend':self.backend,'allocator':('priority-ordered shared cleanup queue; not QMIX' if self.continuous
                    else 'role-constrained priority baseline; not QMIX'),
                'same_stage':True,'camera_frames':self.frames,'latest_area_clues':self.results,
                'priorities':{rid:self.priority(rid) for rid in self.records},
                'vision_log':str(self.log_path),
                'visualization':self.dashboard.summary() if self.dashboard else {'enabled':False,'reason':'headless run'},
                'association':'Mounted robot camera -> current robot work area; no object or person localization',
                'limitations':('Shared cleanup assigns original floor props to available identical manipulators; not a learned role-switching policy. No ROS interface.'
                    if self.continuous else 'Original robot/object/rack roles retained. No cross-role reassignment or ROS interface in this entry point.')}

    def close(self):
        if self.process is not None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.pool.shutdown(wait=True,cancel_futures=True)
        if self.dashboard:self.dashboard.close()
        self.log.close()
        for rid,annotator in self.sensors.items():
            annotator.detach([self.products[list(self.sensors).index(rid)].path])
        for product in self.products:
            product.destroy()
