"""Isaac Sim 5.1 compatibility boundary; import after SimulationApp starts."""
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, UsdGeom, UsdLux


def cube(stage, path, position, scale, color):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.0)
    shape.AddTranslateOp().Set(Gf.Vec3d(*position))
    shape.AddScaleOp().Set(Gf.Vec3f(*scale))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return shape


def camera(stage, path, position):
    sensor = UsdGeom.Camera.Define(stage, path)
    sensor.AddTranslateOp().Set(Gf.Vec3d(*position))
    sensor.AddRotateXYZOp().Set(Gf.Vec3f(75, 0, -90))
    sensor.CreateFocalLengthAttr(18.0)
    sensor.CreateClippingRangeAttr(Gf.Vec2f(.1, 1000))
    return sensor
