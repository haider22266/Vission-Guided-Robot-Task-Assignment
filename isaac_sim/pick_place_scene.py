"""An open-front five-lane scene with grasp-sized environment objects."""


def build_scene(stage):
    from compat import cube, Gf, UsdGeom, UsdLux
    from pick_place_tasks import Item
    kinds=['rear_carton','rubble','screwdriver','rear_beam','wrench']
    colors=[(.85,.58,.15),(.2,.65,.95),(.85,.2,.25),(.2,.8,.4),(.65,.3,.85)]
    cube(stage,'/World/Floor',(0,0,-.06),(10,5,.12),(.12,.16,.18))
    # No front wall, side walls, or roof are authored in this demo.
    cube(stage,'/World/RearWall',(0,1.6,1.1),(10,.1,2.2),(.34,.38,.4))
    light=UsdLux.DomeLight.Define(stage,'/World/Light')
    light.CreateIntensityAttr(1600)
    items=[]
    slots={}
    data={}
    for lane,(kind,color) in enumerate(zip(kinds,colors)):
        x=(lane-2)*1.8
        cube(stage,f'/World/FeedTables/table_{lane}',(x+.49,.35,.14),(.55,.42,.28),(.32,.34,.35))
        rack=f'/World/Racks/rack_{lane}'
        cube(stage,rack+'/shelf',(x+.46,-.46,.29),(.65,.5,.04),(.47,.49,.48))
        for i,(dx,dy) in enumerate([(-.3,-.23),(-.3,.23),(.3,-.23),(.3,.23)]):
            cube(stage,rack+f'/post_{i}',(x+.46+dx,-.46+dy,.23),(.035,.035,.46),(.94,.42,.03))
        for index,(sx,sy) in enumerate([(.32,-.32),(.58,-.32),(.32,-.6),(.58,-.6)]):
            name=f'lane_{lane}_slot_{index}'
            slots[name]={'lane':lane,'position':(x+sx,sy,.31)}
            cube(stage,rack+f'/slot_{index}',(x+sx,sy,.312),(.13,.13,.003),color)
        for index in range(2):
            name=f'{kind}_{index:02d}'
            path='/World/Items/'+name
            obj=UsdGeom.Xform.Define(stage,path)
            width=.044 if kind!='screwdriver' else .03
            height=.056 if kind=='rear_carton' else .04
            position=(x+.4+index*.24,.35,.28+height/2)
            move=obj.AddTranslateOp()
            move.Set(Gf.Vec3d(*position))
            rotate=obj.AddOrientOp()
            rotate.Set(Gf.Quatf(1))
            def part(label,p,size,c):
                return cube(stage,path+'/'+label,p,size,c)
            if kind=='rear_carton':
                part('carton',(0,0,0),(.065,width,height),(.7,.5,.28))
                part('tape_top',(0,0,height/2+.001),(.009,width,.002),(.88,.74,.46))
                for sign in (-1,1):
                    part('tape_side_'+str(sign+1),(0,sign*width/2,0),(.009,.002,height),(.88,.74,.46))
            elif kind=='rubble':
                part('rubble',(0,0,0),(.06,width,height),(.5,.52,.53))
                part('chipped_top',(.015,0,.025),(.025,.035,.012),(.4,.43,.45))
            elif kind=='screwdriver':
                part('red_handle',(0,0,0),(.07,width,height),(.85,.06,.025))
                part('shaft',(.09,0,0),(.11,.008,.008),(.65,.69,.72))
                part('flat_tip',(.154,0,0),(.025,.016,.004),(.65,.69,.72))
            elif kind=='rear_beam':
                part('web',(0,0,0),(.22,width,height),(.95,.38,.025))
                part('flange_top',(0,0,.02),(.22,width,.006),(.95,.5,.03))
            else:
                part('handle',(0,0,0),(.16,width,height),(.62,.67,.7))
                for end in (-1,1):
                    for jaw in (-1,1):
                        part(f'jaw_{end+1}_{jaw+1}',(end*.09,jaw*.027,0),(.045,.018,height),(.62,.67,.7))
            item=Item(name,kind,lane,width,height)
            items.append(item)
            data[name]={'translate':move,'orient':rotate,'position':position,'path':path}
        data[f'robot_{lane+1}']={'base':(x,-.22,.2),'color':color,'lane':lane,'kind':kind}
    return items,slots,data
