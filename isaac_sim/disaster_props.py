"""Editable fallen cartons, rack beams, debris and recognizable hand tools."""
def add_disaster_props(stage):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    base='/World/DisasterProps'
    if stage.GetPrimAtPath(base):
        raise ValueError('Disaster props already exist')
    root=UsdGeom.Xform.Define(stage,base).GetPrim()
    materials={}

    def material(color,metal=0.):
        key=(color,metal)
        if key not in materials:
            path=base+'/Materials/m_'+str(len(materials))
            mat=UsdShade.Material.Define(stage,path)
            shader=UsdShade.Shader.Define(stage,path+'/Shader')
            shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.32 if metal else .85)
            shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
            materials[key]=mat
        return materials[key]

    def box(path,position,size,color,metal=0.,collision=True):
        # Polygon meshes are used so the NuRec triangle-map builder sees props.
        mesh=UsdGeom.Mesh.Define(stage,path)
        mesh.CreatePointsAttr([(-.5,-.5,-.5),(.5,-.5,-.5),(.5,.5,-.5),(-.5,.5,-.5),
                              (-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)])
        mesh.CreateFaceVertexCountsAttr([4]*6)
        mesh.CreateFaceVertexIndicesAttr([0,3,2,1,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.AddTranslateOp().Set(Gf.Vec3d(*position))
        mesh.AddScaleOp().Set(Gf.Vec3f(*size))
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(color,metal))
        if collision:
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        return mesh

    def group(path,xy,yaw=0.,tilt=0.):
        obj=UsdGeom.Xform.Define(stage,path)
        translate=obj.AddTranslateOp()
        translate.Set(Gf.Vec3d(*xy,0))
        obj.AddRotateXYZOp().Set(Gf.Vec3f(0,tilt,yaw))
        return obj,translate

    def ground(obj,translate,lift=.015):
        bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default']).ComputeWorldBound(obj.GetPrim()).ComputeAlignedRange()
        pos=translate.Get()
        translate.Set(Gf.Vec3d(pos[0],pos[1],pos[2]-bounds.GetMin()[2]+lift))

    # Keep the short demo task paths near y=0 clear. Cartons spill from the
    # north storage edge, with irregular yaw and several resting on a side.
    positions=[(-3.5,3.35),(-2.85,3.65),(-3.65,4.05),
               (-1.7,3.5),(-1.0,3.8),(-1.9,4.3),
               (.05,3.5),(.7,3.8),(-.1,4.25),
               (2.25,3.55),(2.95,3.9),(2.2,4.5)]
    for i,xy in enumerate(positions):
        path=base+f'/Cartons/carton_{i:02d}'
        obj,translate=group(path,xy,[-22,17,48,-12,28,-35,13,-18,31,-42,19,8][i],90 if i%3==1 else 8)
        w,d,h=(.62+.08*(i%2),.48+.04*(i%3),.48+.06*(i%3))
        color=[(.68,.47,.25),(.78,.59,.36),(.62,.42,.23)][i%3]
        box(path+'/cardboard',(0,0,0),(w,d,h),color)
        tape=(.83,.68,.43)
        for side in (-1,1):
            suffix='negative' if side<0 else 'positive'
            box(path+'/tape_y_top_'+suffix,(0,0,side*(h/2+.004)),(.065,d+.012,.006),tape,collision=False)
            box(path+'/tape_x_top_'+suffix,(0,0,side*(h/2+.008)),(w+.012,.065,.006),tape,collision=False)
            box(path+'/tape_front_'+suffix,(0,side*(d/2+.004),0),(.065,.006,h),tape,collision=False)
            box(path+'/tape_side_'+suffix,(side*(w/2+.004),0,0),(.006,.065,h),tape,collision=False)
        box(path+'/shipping_label',(.17,-d/2-.009,.04),(.18,.006,.13),(.88,.86,.77),collision=False)
        for stripe in range(5):
            box(path+f'/barcode_{stripe}',(.105+stripe*.027,-d/2-.014,.04),(.009,.003,.085),(.12,.11,.1),collision=False)
        obj.GetPrim().SetCustomDataByKey('prop_type','fallen_carton')
        ground(obj,translate)

    for i,(x,y,yaw) in enumerate([(-2.4,4.7,25),(.7,4.75,-32),(2.9,5.1,12)]):
        path=base+f'/FallenRackBeams/beam_{i}'
        obj,translate=group(path,(x,y),yaw,4)
        box(path+'/web',(0,0,0),(1.25,.075,.14),(.88,.33,.025),.45)
        for z in (-.07,.07):
            box(path+('/flange_bottom' if z<0 else '/flange_top'),(0,0,z),(1.25,.18,.035),(.95,.43,.03),.5)
        ground(obj,translate)
    for i in range(8):
        path=base+f'/Debris/fragment_{i}'
        obj,translate=group(path,(-3.7+i*.82,5.5-(i%2)*.25),i*29)
        box(path+'/fragment',(0,0,0),(.18+.04*(i%3),.13,.12),(.45,.47,.48))
        ground(obj,translate)

    steel=(.58,.64,.68)
    # Contrasting handles and clear silhouettes make the small tools visible.
    path=base+'/Tools/Hammer'
    obj,translate=group(path,(-2.1,3.0),-35)
    box(path+'/wood_handle',(0,-.12,0),(.065,.5,.065),(.5,.25,.07))
    box(path+'/steel_head',(0,.16,.015),(.3,.115,.12),steel,.8)
    box(path+'/claw',( .12,.23,.015),(.055,.12,.09),steel,.8)
    ground(obj,translate)

    path=base+'/Tools/Screwdriver'
    obj,translate=group(path,(-.45,3.05),40)
    box(path+'/red_grip',(0,-.16,0),(.095,.24,.08),(.8,.045,.025))
    box(path+'/shaft',(0,.11,0),(.022,.3,.022),steel,.9)
    box(path+'/flat_tip',(0,.285,0),(.055,.065,.014),steel,.9)
    ground(obj,translate)

    path=base+'/Tools/Wrench'
    obj,translate=group(path,(1.25,3.1),-23)
    box(path+'/handle',(0,0,0),(.065,.43,.045),steel,.85)
    for side in (-1,1):
        suffix='lower' if side<0 else 'upper'
        box(path+'/neck_'+suffix,(0,side*.22,0),(.17,.09,.05),steel,.85)
        for jaw in (-1,1):
            box(path+f'/jaw_{suffix}_{jaw+1}',(jaw*.065,side*.3,0),(.04,.17,.05),steel,.85)
    ground(obj,translate)
    root.SetCustomDataByKey('carton_count',12)
    root.SetCustomDataByKey('description','12 larger taped fallen cartons; 3 rack beams; debris; hammer, screwdriver, wrench')
    return {'cartons':12,'fallen_rack_beams':3,'debris_fragments':8,'tools':['hammer','screwdriver','wrench']}
