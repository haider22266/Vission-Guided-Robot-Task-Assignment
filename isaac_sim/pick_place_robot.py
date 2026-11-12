"""Official Franka meshes driven by Lula IK/FK, with contact-gated carrying.

This is kinematic manipulation. Source robot physics is disabled and no
force/closure stability claim is made. Objects never move before verification.
"""
import math
import numpy as np


class Picker:
    def __init__(self,stage,robot_id,info,data,broker,asset_root):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
        from isaacsim.robot_motion.motion_generation.interface_config_loader import load_supported_lula_kinematics_solver_config
        from compat import cube
        self.Gf,self.stage,self.id=Gf,stage,robot_id
        self.info,self.data,self.broker=info,data,broker
        self.base=np.array(info['base'],dtype=float)
        self.home_y=float(self.base[1])
        path='/World/Robots/'+robot_id
        root=UsdGeom.Xform.Define(stage,path)
        self.base_op=root.AddTranslateOp()
        self.base_op.Set(Gf.Vec3d(*map(float,self.base)))
        cube(stage,path+'/mobile_base',(0,0,-.10),(.42,.36,.18),info['color'])
        for i,(x,y) in enumerate([(-.15,-.21),(-.15,.21),(.15,-.21),(.15,.21)]):
            cube(stage,path+f'/wheel_{i}',(x,y,-.14),(.12,.08,.12),(.055,.06,.07))
        self.arm_path=path+'/Arm'
        arm=UsdGeom.Xform.Define(stage,self.arm_path).GetPrim()
        asset=asset_root+'/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd'
        arm.GetReferences().AddReference(asset)
        if len(list(Usd.PrimRange(arm)))<3:
            raise RuntimeError(f'Franka asset unavailable: {asset}')
        for prim in Usd.PrimRange(arm):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            if prim.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
        self.solver=LulaKinematicsSolver(**load_supported_lula_kinematics_solver_config('Franka'))
        frames=set(self.solver.get_all_frame_names())
        self.links={}
        for prim in arm.GetChildren():
            name=prim.GetName()
            if name not in frames and name not in ('panda_leftfinger','panda_rightfinger'):
                continue
            xf=UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            self.links[name]=(xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'display'),
                              xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat,'display'))
        for name in ('panda_leftfinger','panda_rightfinger'):
            if name not in self.links:
                raise RuntimeError('Expected real Franka finger links are missing')
            # Thin visible rubber pads define an explicit grasp-contact surface.
            cube(stage,self.arm_path+'/'+name+'/ContactPad',(0,-.002,.045),(.018,.004,.025),(.04,.045,.05))
        self.q=np.array([0.,-.7,0.,-2.2,0.,1.5,.8])
        self.q_target=self.q.copy()
        self.opening=.04
        self.finger_target=.04
        self.phase='waiting'
        self.phase_age=0.
        self.task=None
        self.held=False
        self.stable=0
        self.completed=0
        self.last_error=None
        self.update_meshes()

    def quat(self,rotation):
        matrix=np.eye(4)
        matrix[:3,:3]=rotation.T
        return self.Gf.Quatf(self.Gf.Matrix4d(*map(float,matrix.ravel())).ExtractRotationQuat())

    def update_meshes(self):
        self.base_op.Set(self.Gf.Vec3d(*map(float,self.base)))
        self.solver.set_robot_base_pose(self.base,np.array([1.,0.,0.,0.]))
        for name,(translation,orient) in self.links.items():
            if name in ('panda_leftfinger','panda_rightfinger'):
                continue
            p,r=self.solver.compute_forward_kinematics(name,self.q)
            translation.Set(self.Gf.Vec3d(*map(float,p-self.base)))
            orient.Set(self.quat(r))
        self.hand,self.rotation=self.solver.compute_forward_kinematics('panda_hand',self.q)
        self.tcp=self.hand+self.rotation@np.array([0.,0.,.1034])
        self.pads=[]
        for sign,name in ((1,'panda_leftfinger'),(-1,'panda_rightfinger')):
            rotation=self.rotation if sign==1 else self.rotation@np.diag([-1.,-1.,1.])
            position=self.hand+self.rotation@np.array([0.,sign*self.opening,.0584])
            translation,orient=self.links[name]
            translation.Set(self.Gf.Vec3d(*map(float,position-self.base)))
            orient.Set(self.quat(rotation))
            self.pads.append(position+rotation@np.array([0.,-.004,.045]))
        if self.held:
            position=self.hand+self.rotation@self.grasp_offset
            rotation=self.rotation@self.grasp_rotation
            entry=self.data[self.task['object']]
            entry['translate'].Set(self.Gf.Vec3d(*map(float,position)))
            entry['orient'].Set(self.quat(rotation))
            entry['position']=tuple(map(float,position))

    def target(self,phase,tcp):
        tcp=np.asarray(tcp,dtype=float)
        self.solver.set_robot_base_pose(self.base,np.array([1.,0.,0.,0.]))
        rotation=np.diag([1.,-1.,-1.])
        hand=tcp-rotation@np.array([0.,0.,.1034])
        q,success=self.solver.compute_inverse_kinematics('panda_hand',hand,np.array([0.,1.,0.,0.]),
                                                         self.q,.0002,.0005)
        if not success:
            self.phase='IK blocked'
            self.last_error=f'No IK solution for {phase}; object has not been acquired'
            return False
        self.q_target=q
        self.tcp_target=tcp
        self.phase=phase
        self.phase_age=0.
        print(f'{self.id}: {phase} | object={self.task["object"]}',flush=True)
        return True

    def ready(self):
        return np.linalg.norm(self.tcp-self.tcp_target)<.001 and np.max(np.abs(self.q-self.q_target))<.002

    def step(self,dt):
        self.phase_age+=dt
        delta=self.q_target-self.q
        largest=float(np.max(np.abs(delta)))
        if largest>0:
            self.q+=delta*min(1.,1.2*dt/largest)
        self.opening+=float(np.clip(self.finger_target-self.opening,-.035*dt,.035*dt))
        self.update_meshes()
        if self.phase=='waiting':
            self.task=self.broker.claim(self.id,self.info['kind'],self.info['lane'])
            if self.task:
                self.phase='approach'
                self.phase_age=0
            return
        if self.phase=='IK blocked':
            return
        item=self.broker.items[self.task['object']]
        source=np.array(self.data[item.name]['position'])
        destination=np.array(self.broker.slots[self.task['slot']]['position'])+np.array([0,0,item.height/2])
        if self.phase=='approach':
            self.base[1]=min(0.,self.base[1]+.12*dt)
            if self.base[1]>=0:
                self.finger_target=.04
                self.target('pregrasp',source+[0,0,.14])
        elif self.phase=='pregrasp' and self.ready():
            self.target('align grasp',source)
        elif self.phase=='align grasp' and self.ready():
            self.finger_target=item.width/2+.004
            self.phase='close fingers'
            self.stable=0
        elif self.phase=='close fingers':
            from pxr import Usd, UsdGeom
            # Measure the actual rendered USD pad faces, not just desired IK poses.
            visible_pads=[]
            for name in ('panda_leftfinger','panda_rightfinger'):
                prim=self.stage.GetPrimAtPath(self.arm_path+'/'+name+'/ContactPad')
                matrix=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                visible_pads.append(np.array(matrix.Transform(self.Gf.Vec3d(0,-.5,0))))
            desired=[source+self.rotation@np.array([0,item.width/2,0]),
                     source+self.rotation@np.array([0,-item.width/2,0])]
            gaps=[float(np.linalg.norm(p-q)) for p,q in zip(visible_pads,desired)]
            center=float(np.linalg.norm(self.tcp-source))
            orientation=float(np.linalg.norm(self.rotation-np.diag([1.,-1.,-1.])))
            touching=center<=.0015 and max(gaps)<=.002 and orientation<=.005
            self.stable=self.stable+1 if touching else 0
            errors=dict(center_error=center,left_gap=gaps[0],right_gap=gaps[1],
                        orientation_error=orientation,stable_frames=self.stable)
            if self.broker.grasp(self.id,errors):
                self.grasp_offset=self.rotation.T@(source-self.hand)
                self.grasp_rotation=self.rotation.T
                self.held=True
                print(f'{self.id}: VERIFIED GRASP {item.name} pad gaps={gaps}',flush=True)
                self.target('lift',source+[0,0,.18])
            elif self.phase_age>10:
                self.last_error=f'Grasp not verified; pad gaps {gaps}. Object remains unacquired.'
        elif self.phase=='lift' and self.ready():
            self.target('carry to rack',destination+[0,0,.18])
        elif self.phase=='carry to rack' and self.ready():
            self.target('place on shelf',destination)
        elif self.phase=='place on shelf' and self.ready():
            actual=np.array(self.data[item.name]['position'])
            if np.linalg.norm(actual-destination)<=.004:
                self.phase='open and release'
                self.finger_target=.04
                self.phase_age=0.
        elif self.phase=='open and release':
            # Once fingers visibly clear the item, leave it stationary on its shelf.
            clearance=self.opening-.004-item.width/2
            if clearance>=.003:
                self.held=False
            if not self.held and abs(self.opening-.04)<.0001:
                error=float(np.linalg.norm(np.array(self.data[item.name]['position'])-destination))
                if self.broker.complete(self.id,error,clearance):
                    self.completed+=1
                    print(f'{self.id}: COMPLETED {item.name} -> {self.task["slot"]} error={error:.6f}m',flush=True)
                    self.target('retreat',destination+[0,0,.18])
        elif self.phase=='retreat' and self.ready():
            self.task=None
            self.phase='waiting'
