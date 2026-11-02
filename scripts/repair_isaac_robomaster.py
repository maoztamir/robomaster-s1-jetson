"""Reimport the saved RoboMaster URDF with resolved meshes using Isaac Sim 5.1.

Run through Isaac Sim's standalone launcher. Original URDF/USD files are preserved.
This repairs the base asset; sensor placement and mecanum physics remain separate.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
try:
    import omni.kit.commands
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    enable_extension("isaacsim.asset.importer.urdf")
    app.update()
    root = Path("/home/ubuntu/robomaster-s1-jetson")
    output = root / "assets" / "robomaster"
    output.mkdir(parents=True, exist_ok=True)
    source = Path("/home/ubuntu/robomaster_isaac/urdf/robomaster_s1_base.urdf")
    description = root / "vendor" / "robomaster_description"
    tree = ET.parse(source)
    for mesh in tree.findall(".//mesh"):
        uri = mesh.attrib["filename"]
        prefix = "package://robomaster_description/"
        if not uri.startswith(prefix):
            raise RuntimeError(f"Unexpected mesh URI: {uri}")
        resolved = description / uri[len(prefix):]
        if not resolved.is_file():
            raise FileNotFoundError(resolved)
        mesh.set("filename", str(resolved))
    urdf = output / "robomaster_resolved.urdf"
    tree.write(urdf, encoding="utf-8", xml_declaration=True)
    ok, config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not ok:
        raise RuntimeError("Could not create URDF import configuration")
    config.merge_fixed_joints = False
    config.convex_decomp = False
    config.import_inertia_tensor = True
    config.fix_base = False
    config.distance_scale = 1.0
    model_path = output / "robomaster.usd"
    ok, _ = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=str(urdf), import_config=config,
        dest_path=str(model_path), get_articulation_root=True,
    )
    if not ok:
        raise RuntimeError("RoboMaster import failed")
    model = Usd.Stage.Open(str(model_path))
    model.SetDefaultPrim(model.GetPrimAtPath("/robomaster_s1_base"))
    model.GetRootLayer().Save()
    meshes = [p for p in Usd.PrimRange.Stage(model, Usd.TraverseInstanceProxies())
              if p.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise RuntimeError("Import has no meshes; refusing to accept empty geometry")
    scene_path = output / "robomaster_inspection.usda"
    scene = Usd.Stage.CreateNew(str(scene_path))
    UsdGeom.SetStageUpAxis(scene, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(scene, 1.0)
    world = UsdGeom.Xform.Define(scene, "/World")
    scene.SetDefaultPrim(world.GetPrim())
    robot = scene.DefinePrim("/World/RoboMaster", "Xform")
    robot.GetReferences().AddReference("robomaster.usd", "/robomaster_s1_base")
    light = UsdLux.DomeLight.Define(scene, "/World/Light")
    light.CreateIntensityAttr(1000)
    ground = UsdGeom.Cube.Define(scene, "/World/Floor")
    ground.CreateSizeAttr(1)
    ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    ground.AddScaleOp().Set(Gf.Vec3d(4, 4, 0.1))
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.18, 0.18)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    scene.GetRootLayer().Save()
    print(f"ROBOMASTER_IMPORT_OK meshes={len(meshes)} scene={scene_path}", flush=True)
finally:
    app.close()
