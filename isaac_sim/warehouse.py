"""Procedural warehouse inspired by the reference; dimensions are metres."""
import random

from compat import cube, Gf, UsdGeom, UsdLux
from pxr import UsdPhysics, UsdShade, Sdf


def build_warehouse(stage):
    rng = random.Random(42)
    materials = {}

    def box(name, position, size, color, metal=0.0, collision=True, yaw=0):
        shape = cube(stage, '/World/Warehouse/' + name, position, size, color)
        if yaw:
            shape.AddRotateZOp().Set(yaw)
        key = (color, metal)
        if key not in materials:
            path = '/World/Materials/material_' + str(len(materials))
            material = UsdShade.Material.Define(stage, path)
            shader = UsdShade.Shader.Define(stage, path + '/Shader')
            shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.4 if metal else .75)
            shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metal)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
            materials[key] = material
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(materials[key])
        if collision:
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        return shape

    steel = (.09, .12, .14)
    orange = (.95, .46, .035)
    yellow = (.96, .78, .04)
    # Front and roof remain open for the overview camera.
    box('Floor', (0, 0, -.12), (26, 32, .24), (.095, .135, .16), .35)
    for x in range(-12, 13, 2):
        box(f'FloorJoints/x_{x+12}', (x, 0, .002), (.018, 32, .004), (.23, .27, .29), collision=False)
    for y in range(-16, 17, 2):
        box(f'FloorJoints/y_{y+16}', (0, y, .002), (26, .018, .004), (.23, .27, .29), collision=False)
    for side, x in [('left', -13), ('right', 13)]:
        box(f'Walls/{side}_lower', (x, 0, 1.6), (.25, 32, 3.2), (.58, .61, .61))
        box(f'Walls/{side}_upper', (x, 0, 5.7), (.25, 32, 5), (.62, .32, .045))
        for n, y in enumerate(range(-15, 17, 4)):
            box(f'Columns/{side}_{n}', (x*.98, y, 4.2), (.25, .3, 8.4), steel, .7)
    box('Walls/back_lower', (0, 16, 1.6), (26, .25, 3.2), (.58, .61, .61))
    box('Walls/back_upper', (0, 16, 5.7), (26, .25, 5), (.62, .32, .045))
    box('LoadingDoor', (0, 15.8, 2), (4, .12, 4), (.25, .29, .3), .6)
    for i in range(15):
        box(f'DoorSlats/slat_{i}', (0, 15.71, .2+i*.25), (3.9, .02, .025), steel, .6)

    # Three longitudinal rack rows leave two wide robot aisles.
    for row, x in enumerate((-10, 0, 10)):
        for bay in range(4):
            y = -3 + bay*4
            prefix = f'Racks/row_{row}/bay_{bay}'
            for end, dy in enumerate((-1.9, 1.9)):
                for side, dx in enumerate((-.95, .95)):
                    box(f'{prefix}/post_{end}_{side}', (x+dx, y+dy, 3.6), (.12, .12, 7.2), steel, .7)
                    box(f'{prefix}/guard_{end}_{side}', (x+dx, y+dy, .35), (.24, .24, .7), yellow)
            for level, z in enumerate((.45, 2.5, 4.55, 6.6)):
                box(f'{prefix}/shelf_{level}', (x, y, z), (2, 3.9, .1), (.43, .46, .46), .6)
                for side, dx in enumerate((-1, 1)):
                    box(f'{prefix}/beam_{level}_{side}', (x+dx, y, z-.12), (.12, 4, .2), orange, .5)
                for parcel in range(3):
                    height = rng.uniform(.45, 1.1)
                    bx, by = x+rng.uniform(-.3, .3), y-1.25+parcel*1.2
                    color = rng.choice([(.66,.48,.27), (.8,.67,.45), (.72,.61,.43), (.32,.16,.29)])
                    box(f'{prefix}/cargo_{level}_{parcel}', (bx, by, z+.05+height/2),
                        (rng.uniform(.8, 1.3), .95, height), color)
                    box(f'{prefix}/tape_{level}_{parcel}', (bx, by, z+.055+height),
                        (.09, .96, .008), (.85,.76,.55), collision=False)
            for side, dx in enumerate((-1.3, 1.3)):
                box(f'{prefix}/aisle_line_{side}', (x+dx, y, .012), (.08, 4, .015), yellow, collision=False)
        # Red-white protective foot at the front of each rack.
        for stripe in range(8):
            box(f'RackFeet/row_{row}_{stripe}', (x-.98+stripe*.28, -5.15, .02),
                (.28, .35, .03), (.85,.85,.8) if stripe % 2 else (.65,.045,.025), collision=False)

    # Fallen cargo in the central cross-aisle; front staging corridor stays clear.
    for i in range(22):
        x, y = rng.uniform(-4, 4), rng.uniform(-6.9, -5.6)
        height = rng.uniform(.18, .75)
        box(f'Debris/box_{i}', (x, y, height/2),
            (rng.uniform(.25,.8), rng.uniform(.25,.7), height),
            rng.choice([(.76,.61,.39), (.58,.45,.27), (.55,.59,.6)]), yaw=rng.uniform(-60,60))
    for i in range(7):
        box(f'Debris/plank_{i}', (rng.uniform(-4,4), -6.2+rng.uniform(-.5,.5), .12),
            (1.6, .13, .12), (.48,.29,.13), yaw=rng.uniform(-70,70))
    # Obstacles across nominal patrol lines exercise replanning in both aisles.
    box('AisleObstacles/left_crate', (-3.8, 0, .4), (.9, 1, .8), (.76,.61,.39))
    box('AisleObstacles/right_crate', (4, 5, .45), (1.1, .9, .9), (.76,.61,.39))
    for x in (-6, 6):
        for y in (-10, 0, 10):
            light = UsdLux.RectLight.Define(stage, f'/World/Warehouse/Lights/light_{x+6}_{y+10}')
            light.AddTranslateOp().Set(Gf.Vec3d(x, y, 7.8))
            light.CreateWidthAttr(2)
            light.CreateHeightAttr(.7)
            light.CreateIntensityAttr(650)
            light.CreateColorAttr(Gf.Vec3f(1, .89, .72))
            box(f'LightHousing/light_{x+6}_{y+10}', (x, y, 7.86), (2.2,.9,.12), steel, .7, collision=False)
    stage.GetPrimAtPath('/World/Warehouse').SetCustomDataByKey(
        'layout', '26x32m open-front warehouse; static collision geometry; synthetic cargo')


def add_display_arm(stage, path):
    """Geometric arm silhouette, with no joints or manipulation controller."""
    from pxr import UsdGeom
    for name, start, end, radius in [
        ('pedestal', (0,0,.18), (0,0,.6), .13),
        ('upper', (0,0,.6), (.12,0,1.15), .095),
        ('forearm', (.12,0,1.15), (.48,0,1.45), .08),
        ('wrist', (.48,0,1.45), (.65,0,1.25), .065),
    ]:
        a, b = Gf.Vec3d(*start), Gf.Vec3d(*end)
        link = UsdGeom.Cylinder.Define(stage, path+'/arm/'+name)
        link.CreateRadiusAttr(radius)
        link.CreateHeightAttr((b-a).GetLength())
        link.AddTranslateOp().Set((a+b)*.5)
        rotation = Gf.Rotation(Gf.Vec3d(0,0,1), (b-a).GetNormalized())
        link.AddOrientOp().Set(Gf.Quatf(rotation.GetQuat()))
        link.CreateDisplayColorAttr([Gf.Vec3f(.78,.8,.79)])
    for side in (-1, 1):
        cube(stage, path+f'/arm/finger_{side+1}', (.65,side*.065,1.17), (.05,.025,.18), (.15,.17,.18))
