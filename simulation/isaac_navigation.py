"""RoboMaster navigation prototype in Isaac Sim 5.1, isolated on domain 43.

Uses ideal planar mecanum motion and ground-truth localization, not roller physics.
Scans intersect the same box geometry shown in Isaac. Cameras use RTX rendering.
"""
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
from planar import integrate, overlaps, ray_distance, wheel_rates

CONFIG = json.loads((ROOT / "config/robomaster_sim.json").read_text())
SLAM = os.environ.get("ROBOMASTER_SLAM") == "1"
if os.environ.get("ROS_DOMAIN_ID") != "43":
    raise RuntimeError("Simulation must run on ROS_DOMAIN_ID=43, never hardware domain 42")
if CONFIG["sensor_mounts_status"].startswith("provisional") and os.environ.get("ROBOMASTER_ALLOW_PROVISIONAL") != "1":
    raise RuntimeError("Sensor mounts are unmeasured. Use --provisional for the labelled navigation prototype.")

from isaacsim import SimulationApp
stream_args = ["--enable", "omni.kit.livestream.webrtc"] if os.environ.get("ROBOMASTER_WEBRTC") == "1" else []
app = SimulationApp({"headless": os.environ.get("ROBOMASTER_HEADLESS") == "1", "width": 1280, "height": 720,
                     "extra_args": stream_args})
import numpy as np
import omni.ui as ui
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from pxr import Gf, Usd, UsdGeom, UsdLux

enable_extension("isaacsim.ros2.bridge")
app.update()
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, CameraInfo, JointState
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker

world = World(stage_units_in_meters=1., physics_dt=1/30, rendering_dt=1/30)
stage = omni.usd.get_context().get_stage()
world.scene.add_default_ground_plane()
UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1000)
robot = stage.DefinePrim("/World/RoboMaster", "Xform")
robot.GetReferences().AddReference(CONFIG["robot_usd"], CONFIG["robot_prim"])
# The repaired model has variants on its nested base_link. Disable physical
# articulation explicitly: this prototype integrates planar motion itself.
for prim_path in [p.GetPath() for p in Usd.PrimRange(robot)]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        continue
    variants = prim.GetVariantSets()
    if variants.HasVariantSet("Physics"):
        variants.GetVariantSet("Physics").SetVariantSelection("None")
    if variants.HasVariantSet("Robot"):
        variants.GetVariantSet("Robot").SetVariantSelection("None")
robot_xform = UsdGeom.Xformable(robot)
robot_xform.ClearXformOpOrder()
robot_translate = robot_xform.AddTranslateOp()
robot_rotate = robot_xform.AddRotateZOp()

xmin, ymin, xmax, ymax = CONFIG["room_bounds"]
wall_thickness = CONFIG.get("wall_thickness", .1)
walls = [((xmin+xmax)/2, ymin, xmax-xmin+wall_thickness, wall_thickness, 1.),
         ((xmin+xmax)/2, ymax, xmax-xmin+wall_thickness, wall_thickness, 1.),
         (xmin, (ymin+ymax)/2, wall_thickness, ymax-ymin, 1.),
         (xmax, (ymin+ymax)/2, wall_thickness, ymax-ymin, 1.)]
static_boxes = walls + CONFIG["obstacles"]
def add_box(path, box, color):
    x,y,w,h,z = box
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(x,y,z/2))
    cube.AddScaleOp().Set(Gf.Vec3d(w,h,z))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cube
# Solid maze walls: their full box geometry is shared by map, scan
# and collision guard. Floor tiles are purely visual, below the scan plane.
for i, box in enumerate(walls):
    add_box(f"/World/Walls/Wall_{i}", box, (.68,.70,.72))
palette = [(.30,.39,.43), (.57,.36,.18), (.26,.32,.39), (.38,.43,.48)]
for i, box in enumerate(CONFIG["obstacles"]):
    name = CONFIG.get("obstacle_names", [])[i] if i < len(CONFIG.get("obstacle_names", [])) else f"Obstacle_{i}"
    add_box(f"/World/Furniture/{name}", box, palette[i % len(palette)])
    # Thin trim stays inside the obstacle envelope, above the LiDAR plane.
    x,y,w,h,z = box
    trim = add_box(f"/World/Furniture/{name}_Top", (x,y,w*.98,h*.98,.025), (.16,.18,.20))
    UsdGeom.Xformable(trim).GetOrderedXformOps()[0].Set(Gf.Vec3d(x,y,z-.0125))
for ix in range(math.ceil(xmax-xmin)):
    for iy in range(math.ceil(ymax-ymin)):
        shade = .31 if (ix+iy)%2 else .35
        add_box(f"/World/FloorTiles/Tile_{ix}_{iy}",
                (xmin+ix+.5,ymin+iy+.5,.985,.985,.002), (shade,shade,shade))
dynamic_prim = add_box("/World/DynamicBlockage", CONFIG["dynamic_blockage"], (.9,.3,.1))
dynamic_prim.MakeInvisible()
# Visible sensor placeholders follow the configured simulation mount locations.
lidar_body = UsdGeom.Cylinder.Define(stage, "/World/RoboMaster/LidarHousing")
lidar_body.CreateRadiusAttr(.035)
lidar_body.CreateHeightAttr(.025)
lidar_body.AddTranslateOp().Set(Gf.Vec3d(*CONFIG["lidar_xyz"]))
lidar_body.CreateDisplayColorAttr([Gf.Vec3f(.1,.1,.1)])

cameras = []
for side, sign in (("left",1), ("right",-1)):
    xyz = np.array(CONFIG["camera_xyz"], dtype=float)
    roll,pitch,yaw = CONFIG["camera_rpy"]
    rotation = Gf.Rotation(Gf.Vec3d(1,0,0), math.degrees(roll)) * Gf.Rotation(Gf.Vec3d(0,1,0), math.degrees(pitch)) * Gf.Rotation(Gf.Vec3d(0,0,1), math.degrees(yaw))
    xyz += np.array(rotation.TransformDir(Gf.Vec3d(0, sign * CONFIG["stereo_baseline"] / 2, 0)))
    camera = Camera(prim_path=f"/World/RoboMaster/{side}_camera", name=f"{side}_camera",
                    translation=xyz, frequency=5, resolution=tuple(CONFIG["camera_resolution"]))
    # Isaac camera API uses world axes (+X forward, +Z up) by default.
    q = rotation.GetQuat()
    camera.set_local_pose(translation=xyz, orientation=np.array([q.GetReal(), *q.GetImaginary()]), camera_axes="world")
    cameras.append((side, camera, xyz))
set_camera_view(eye=[4.,-5.,5.], target=[0.,0.,0.], camera_prim_path="/OmniverseKit_Persp")
world.reset()
for side, camera, xyz in cameras:
    camera.initialize()
    aperture = camera.get_horizontal_aperture()
    camera.set_focal_length(aperture/(2*math.tan(math.radians(CONFIG["camera_horizontal_fov_deg"])/2)))

# Display the existing sensor frames inside the streamed Isaac window. Reusing
# these images avoids creating extra viewports or additional camera renders.
camera_providers = {}
camera_preview = None
camera_preview_status = None
camera_preview_live = False
if os.environ.get("ROBOMASTER_HEADLESS") != "1" and os.environ.get("ROBOMASTER_CAMERA_PREVIEW", "1") == "1":
    camera_preview = ui.Window("Robot Cameras", width=680, height=250)
    camera_preview.setPosition(20, 430)
    with camera_preview.frame:
        with ui.VStack(spacing=4):
            camera_preview_status = ui.Label("Waiting for camera frames...", height=20)
            with ui.HStack(spacing=8):
                for side, _, _ in cameras:
                    provider = ui.ByteImageProvider()
                    camera_providers[side] = provider
                    with ui.VStack(spacing=4):
                        ui.Label(f"{side.capitalize()} camera", height=18)
                        ui.ImageWithProvider(provider, height=180,
                            fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT)


def stamp(seconds):
    ns = round(seconds*1e9)
    return Time(sec=ns//1000000000, nanosec=ns%1000000000)

def transform(parent, child, xyz, yaw=0., quaternion=None):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, xyz)
    q = quaternion or (0.,0.,math.sin(yaw/2), math.cos(yaw/2))
    msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w = map(float,q)
    return msg

class SimNode(Node):
    def __init__(self):
        super().__init__("robomaster_sim")
        self.pose = tuple(CONFIG["start_pose"])
        self.velocity = (0.,0.,0.)
        self.command = (0.,0.,0.)
        self.last_command = -math.inf
        self.t = 0.
        self.blocked = False
        self.collisions = 0
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 1)
        self.create_service(SetBool, "/simulation/blockage", self.blockage)
        self.create_service(Trigger, "/simulation/status", self.navigation_status)
        self.clock_pub = self.create_publisher(Clock,"/clock",qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry,"/odom",10)
        self.scan_pub = self.create_publisher(LaserScan,"/scan",qos_profile_sensor_data)
        self.image_pubs = {s:self.create_publisher(Image,f"/stereo/{s}/camera/image_raw",qos_profile_sensor_data) for s,_,_ in cameras}
        self.info_pubs = {s:self.create_publisher(CameraInfo,f"/stereo/{s}/camera/camera_info",qos_profile_sensor_data) for s,_,_ in cameras}
        self.joint_pub = self.create_publisher(JointState,"/joint_states",10)
        self.marker_pub = self.create_publisher(Marker,"/simulation/robot_marker",10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        static = [transform("base_link","laser",CONFIG["lidar_xyz"],CONFIG["lidar_yaw"])]
        if not SLAM:
            static.append(transform("map","odom",(0,0,0)))
        # Optical +Z forward, +X right, +Y down relative to ROS body axes.
        optical = Gf.Rotation(Gf.Vec3d(1,0,0),-90) * Gf.Rotation(Gf.Vec3d(0,0,1),-90)
        r,p,y = CONFIG["camera_rpy"]
        body = Gf.Rotation(Gf.Vec3d(1,0,0),math.degrees(r)) * Gf.Rotation(Gf.Vec3d(0,1,0),math.degrees(p)) * Gf.Rotation(Gf.Vec3d(0,0,1),math.degrees(y))
        q = (optical * body).GetQuat()
        for side,_,xyz in cameras:
            static.append(transform("base_link",f"{side}_camera_optical_frame",xyz,quaternion=(*q.GetImaginary(),q.GetReal())))
        self.static_tf.sendTransform(static)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid,"/simulation/reference_map" if SLAM else "/map",latched)
        grid = OccupancyGrid()
        grid.header.frame_id = "odom" if SLAM else "map"
        grid.info.resolution = CONFIG["map_resolution"]
        grid.info.width = round((xmax-xmin)/grid.info.resolution)
        grid.info.height = round((ymax-ymin)/grid.info.resolution)
        grid.info.origin.position.x, grid.info.origin.position.y = xmin,ymin
        grid.info.origin.orientation.w = 1.
        grid.data = [100 if any(abs(xmin+(col+.5)*grid.info.resolution-b[0]) <= b[2]/2 and abs(ymin+(row+.5)*grid.info.resolution-b[1]) <= b[3]/2 for b in static_boxes) else 0
                     for row in range(grid.info.height) for col in range(grid.info.width)]
        self.map_pub.publish(grid)
        self.get_logger().info("RoboMaster planar navigation prototype ready on domain 43; mounts: " + CONFIG["sensor_mounts_status"])
    def navigation_status(self, request, response):
        response.success = True
        response.message = json.dumps({"pose": self.pose, "velocity": self.velocity,
            "collision_guard_count": self.collisions, "simulation_time": self.t})
        return response
    def on_command(self,msg):
        command = msg.linear.x,msg.linear.y,msg.angular.z
        if not all(math.isfinite(v) for v in command):
            command = (0.,0.,0.)
        self.command = tuple(max(-limit,min(limit,v)) for v,limit in zip(command,CONFIG["max_velocity"]))
        self.last_command = time.monotonic()
    def blockage(self,request,response):
        if request.data and overlaps(self.pose, CONFIG["footprint_xy"], CONFIG["dynamic_blockage"]):
            response.success=False
            response.message="Move the simulated robot clear of the blockage location first"
            return response
        self.blocked = request.data
        dynamic_prim.MakeVisible() if self.blocked else dynamic_prim.MakeInvisible()
        response.success=True
        response.message="Simulation blockage enabled" if self.blocked else "Simulation blockage removed"
        return response
    def step(self,dt,frame):
        self.t += dt
        self.clock_pub.publish(Clock(clock=stamp(self.t)))
        command = self.command if time.monotonic()-self.last_command < CONFIG["command_timeout_s"] else (0.,0.,0.)
        boxes = static_boxes + ([CONFIG["dynamic_blockage"]] if self.blocked else [])
        next_pose = integrate(self.pose,command,dt)
        if any(overlaps(next_pose,CONFIG["footprint_xy"],b) for b in boxes):
            command=(0.,0.,0.)
            self.collisions += 1
        else:
            self.pose=next_pose
        self.velocity=command
        x,y,yaw=self.pose
        robot_translate.Set(Gf.Vec3d(x,y,0.))
        robot_rotate.Set(math.degrees(yaw))
        tf=transform("odom","base_link",(x,y,0.),yaw)
        tf.header.stamp=stamp(self.t)
        self.tf.sendTransform(tf)
        odom=Odometry()
        odom.header=tf.header
        odom.child_frame_id="base_link"
        odom.pose.pose.position.x,odom.pose.pose.position.y=x,y
        odom.pose.pose.orientation=tf.transform.rotation
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z=command
        self.odom_pub.publish(odom)
        joint=JointState()
        joint.header.stamp=stamp(self.t)
        joint.name=[f"{s}_wheel_joint" for s in ("front_right","front_left","rear_left","rear_right")]
        joint.velocity=list(wheel_rates(command))
        self.joint_pub.publish(joint)
        if frame%3==0:
            scan=LaserScan()
            scan.header.frame_id="laser"
            scan.header.stamp=stamp(self.t)
            scan.angle_min=-math.pi
            scan.angle_increment=2*math.pi/360
            scan.angle_max=scan.angle_min+359*scan.angle_increment
            scan.range_min=.15
            scan.range_max=12.
            scan.scan_time=.1
            lx,ly,lz=CONFIG["lidar_xyz"]
            origin=(x+math.cos(yaw)*lx-math.sin(yaw)*ly,y+math.sin(yaw)*lx+math.cos(yaw)*ly)
            scan_boxes=[b for b in boxes if lz <= b[4]]
            ranges=[ray_distance(origin,yaw+CONFIG["lidar_yaw"]+scan.angle_min+i*scan.angle_increment,scan_boxes,12.) for i in range(360)]
            scan.ranges=[float(v) if .15 <= v < 12. else math.inf for v in ranges]
            self.scan_pub.publish(scan)
            marker=Marker()
            marker.header.frame_id="base_link"
            marker.header.stamp=stamp(self.t)
            marker.ns="simulation_footprint"
            marker.id=0
            marker.type=Marker.CUBE
            marker.action=Marker.ADD
            marker.pose.position.z=.075
            marker.pose.orientation.w=1.
            marker.scale.x,marker.scale.y=CONFIG["footprint_xy"]
            marker.scale.z=.15
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=.2,.7,.9,1.
            self.marker_pub.publish(marker)
    def images(self):
        global camera_preview_live
        # Both render products sample the same simulation tick. Publish a pair
        # only when both are available; calibration shares that exact stamp.
        pair = [(side, camera.get_rgb()) for side,camera,_ in cameras]
        if any(rgb is None or not rgb.size for _,rgb in pair):
            return
        for side,rgb in pair:
            if camera_preview is not None and camera_preview.visible:
                height, width = rgb.shape[:2]
                rgba = np.full((height, width, 4), 255, dtype=np.uint8)
                rgba[:, :, :3] = rgb[:, :, :3]
                camera_providers[side].set_bytes_data(rgba.ravel().tolist(), [width, height])
            msg=Image()
            msg.header.frame_id=f"{side}_camera_optical_frame"
            msg.header.stamp=stamp(self.t)
            msg.height,msg.width=rgb.shape[:2]
            msg.encoding="rgb8"
            msg.step=msg.width*3
            msg.data=np.ascontiguousarray(rgb[:,:,:3],dtype=np.uint8).tobytes()
            self.image_pubs[side].publish(msg)
            info=CameraInfo()
            info.header=msg.header
            info.width,info.height=msg.width,msg.height
            info.distortion_model="plumb_bob"
            info.d=[0.]*5
            fx=msg.width/(2*math.tan(math.radians(CONFIG["camera_horizontal_fov_deg"])/2))
            cx,cy=msg.width/2.,msg.height/2.
            info.k=[fx,0.,cx,0.,fx,cy,0.,0.,1.]
            info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
            tx=-fx*CONFIG["stereo_baseline"] if side=="right" else 0.
            info.p=[fx,0.,cx,tx,0.,fx,cy,0.,0.,0.,1.,0.]
            self.info_pubs[side].publish(info)
        if camera_preview is not None and camera_preview.visible:
            camera_preview_status.text = f"Live stereo cameras | simulation time {self.t:.1f} s"
            if not camera_preview_live:
                self.get_logger().info("Robot Cameras display is receiving both live camera feeds")
                camera_preview_live = True

rclpy.init()
# Start playback explicitly after scene and camera initialization. The main
# loop publishes clock, odometry and TF only while the timeline is playing.
world.play()
node=SimNode()
frame=0
try:
    while app.is_running():
        start=time.monotonic()
        rclpy.spin_once(node,timeout_sec=0.)
        if world.is_playing():
            node.step(1/30,frame)
            world.step(render=True)
            if frame%6==0:
                node.images()
            frame+=1
        else:
            world.step(render=True)
            node.command=(0.,0.,0.)
        time.sleep(max(0.,1/30-(time.monotonic()-start)))
finally:
    print(f"SIM_RESULT pose={node.pose} collision_guard_count={node.collisions}",flush=True)
    node.destroy_node()
    rclpy.shutdown()
    app.close()
