import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from oak_cone_detect_interfaces.msg import ConeArray3D
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.spatial import Delaunay
from std_msgs.msg import ColorRGBA, Float32
import random
import time

# QoS nur für den Subscriber: Best Effort, volatile, depth=1
qos_sub = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Pfad- und Marker-Parameter
PATH_LENGTH            = 20.0
MAX_ANGLE              = 30.0
FIRST_STEP_MAX_ANGLE   = 40.0
MIDPOINT_MARKER_SCALE  = 0.1
DEFAULT_CONE_SCALE     = (0.228, 0.228, 0.325)
LARGE_ORANGE_CONE_SCALE= (0.285, 0.285, 0.505)
CONE_POSITION_SCALE    = (1, 1, 1)
COLOR_MAP = {
    'blue':   [0.0, 0.0, 1.0, 1.0],
    'yellow': [1.0, 1.0, 0.0, 1.0],
    'orange': [1.0, 0.5, 0.0, 1.0]
}
MIDPOINT_COLOR = [0.5, 0.5, 0.5, 1.0]
SIDE_CHECK_RADIUS = 2.0
MAX_STEP_DIST      = 2.0
GEGENCHECK         = 1

class PathNode(Node):
    def __init__(self):
        super().__init__('midpoint_path_node')

        # Subscriber mit Best-Effort QoS
        self.sub = self.create_subscription(
            ConeArray3D,
            '/cone_detections_3d',
            self.callback,
            qos_profile=qos_sub
        )

        # Marker-Publisher nutzen Default-QoS (reliable, depth=10)
        self.marker_pub = self.create_publisher(MarkerArray, '/cone_markers', 10)
        self.path_pub   = self.create_publisher(MarkerArray, '/best_path_marker', 10)
        self.fps_pub    = self.create_publisher(Float32, '/path_inference_fps', 10)

        self._frame_times = []

    def callback(self, msg: ConeArray3D):
        t0 = time.time()

        # --- 1) Kegel sammeln ---
        cones = {col: [] for col in COLOR_MAP}
        for c in msg.cones:
            if c.y < 0:
                continue
            p = np.array([c.x, c.z, 0.0]) * np.array(CONE_POSITION_SCALE)
            cones[c.color].append(p)

        # --- 2) MarkerArray für Kegel ---
        markers = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for col, pts in cones.items():
            for idx, p in enumerate(pts):
                m = Marker()
                m.header = msg.header
                m.ns     = f'cone_{col}'
                m.id     = hash(col)%1000 + idx
                m.type   = Marker.CYLINDER
                m.action = Marker.ADD
                # hier casten wir p[0], p[1], p[2] auf Python-float
                m.pose.position = Point(
                    x=float(p[0]),
                    y=float(p[1]),
                    z=float(p[2])
                )
                scale = LARGE_ORANGE_CONE_SCALE if col=='orange' else DEFAULT_CONE_SCALE
                m.scale.x, m.scale.y, m.scale.z = scale
                r, g, b, a = COLOR_MAP[col]
                m.color = ColorRGBA(r=r, g=g, b=b, a=a)
                markers.markers.append(m)

        # --- 3) Mittelpunkte via Delaunay ---
        pts2d, cols2d = [], []
        for col, pts in cones.items():
            for p in pts:
                pts2d.append(p[:2])
                cols2d.append(col)

        mids_bg, mids_or = [], []
        if len(pts2d) >= 2:
            try:
                tri = Delaunay(np.array(pts2d))
                for s in tri.simplices:
                    for i in range(3):
                        a, b = s[i], s[(i+1)%3]
                        cs = {cols2d[a], cols2d[b]}
                        mid = tuple(((np.array(pts2d[a]) + np.array(pts2d[b]))/2).round(4))
                        if mid[1] < 0:
                            continue
                        if cs == {'blue','yellow'}:
                            mids_bg.append(mid)
                        if cs == {'orange'}:
                            mids_or.append(mid)
            except:
                pass

        mids_bg = sorted(set(mids_bg), key=lambda x: x[0])
        mids_or = sorted(set(mids_or), key=lambda x: x[0])

        # Mittelpunkte markieren
        for idx,(mx,my) in enumerate(mids_bg):
            m = Marker()
            m.header = msg.header
            m.ns     = 'midpoints_bg'
            m.id     = 10000+idx
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            r,g,b,a = MIDPOINT_COLOR
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            markers.markers.append(m)
        for idx,(mx,my) in enumerate(mids_or):
            m = Marker()
            m.header = msg.header
            m.ns     = 'midpoints_or'
            m.id     = 20000+idx
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            m.color   = ColorRGBA(r=1.0, g=0.6, b=0.3, a=1.0)
            markers.markers.append(m)

        # Vorbereiten für Seitentests
        blue_pts   = np.array([p[:2] for p in cones['blue']])   if cones['blue']   else np.empty((0,2))
        yellow_pts = np.array([p[:2] for p in cones['yellow']]) if cones['yellow'] else np.empty((0,2))
        orange_pts = np.array([p[:2] for p in cones['orange']]) if cones['orange'] else np.empty((0,2))

        def check_side_bg(mid, prev, nxt):
            v = np.array(nxt)-np.array(prev)
            if np.linalg.norm(v)<1e-3: return False
            v = v/np.linalg.norm(v)
            ln = np.array([-v[1], v[0]]); rn = np.array([v[1], -v[0]])
            def sides(pts):
                if pts.shape[0]==0: return False,False
                d = pts - mid
                dist = np.linalg.norm(d,axis=1)
                pl = np.dot(d, ln); pr = np.dot(d, rn)
                return np.any((pl>0)&(dist<SIDE_CHECK_RADIUS)), np.any((pr>0)&(dist<SIDE_CHECK_RADIUS))
            lb, rb = sides(blue_pts)
            ly, ry = sides(yellow_pts)
            both_b = lb and rb; both_y = ly and ry
            return (lb and ry or ly and rb) and not both_b and not both_y

        def check_side_or(mid, prev, nxt):
            v = np.array(nxt)-np.array(prev)
            if np.linalg.norm(v)<1e-3: return False
            v = v/np.linalg.norm(v)
            ln = np.array([-v[1], v[0]]); rn = np.array([v[1], -v[0]])
            if orange_pts.shape[0]==0: return False
            d = orange_pts - mid
            dist = np.linalg.norm(d,axis=1)
            pl = np.dot(d,ln); pr = np.dot(d,rn)
            return np.any((pl>0)&(dist<SIDE_CHECK_RADIUS)) and np.any((pr>0)&(dist<SIDE_CHECK_RADIUS))

        def find_greedy_path(mids, check_fn, start_pt, last_vec, max_len, max_step=MAX_STEP_DIST):
            if not mids:
                return [],0.0,last_vec
            arr = np.array(mids)
            path = [tuple(start_pt)]
            used = set()
            last = np.array(start_pt)
            total = 0.0
            y_axis = np.array([0.0,1.0])
            while total < max_len:
                dists = np.linalg.norm(arr-last,axis=1)
                cands = [(i,mids[i],dists[i]) for i in range(len(mids))
                         if i not in used
                         and dists[i]>0.1
                         and dists[i]<=max_step
                         and total+dists[i]<=max_len
                         and mids[i][1]>=0]
                if len(path)==1:
                    cands = [(i,p,d) for (i,p,d) in cands
                             if abs(np.degrees(np.arccos(np.clip(np.dot((np.array(p)-last)/np.linalg.norm(np.array(p)-last), y_axis),-1,1)))) <= FIRST_STEP_MAX_ANGLE]
                if not cands: break
                best = None; best_d=None
                for i,p,d in cands:
                    v = np.array(p)-last
                    n = np.linalg.norm(v)
                    if n<1e-3: continue
                    dirv = v/n
                    if len(path)>1:
                        ang = np.degrees(np.arccos(np.clip(np.dot(last_vec,dirv),-1,1)))
                        if ang<0 or ang>MAX_ANGLE: continue
                    if not check_fn(p,last,p): continue
                    if best is None or d<best_d:
                        best = (i,p,dirv,d); best_d=d
                if best is None: break
                i,p,dirv,d = best
                path.append(p)
                used.add(i)
                last = np.array(p)
                last_vec = dirv
                total += d
            return path, total, last_vec

        # Pfadsuche mit GEGENCHECK-Restarts
        best_bg, best_or = [], []
        max_pts, best_len, abort = 0, 0.0, ""
        for r in range(GEGENCHECK):
            ang = 0 if r==0 else random.uniform(0,2*np.pi)
            v0  = np.array([np.cos(ang), np.sin(ang)])
            p_bg, l_bg, v1 = find_greedy_path(mids_bg, check_side_bg, (0,0), v0, PATH_LENGTH)
            l_or_max = PATH_LENGTH - l_bg
            p_or, l_or = [], 0.0
            if l_or_max>0 and len(p_bg)>1:
                p_or, l_or, _ = find_greedy_path(mids_or, check_side_or, p_bg[-1], v1, l_or_max)
            pts_total = len(p_bg)+max(0,len(p_or)-1)
            if pts_total>max_pts:
                best_bg = p_bg
                best_or = p_or[1:] if len(p_or)>1 else []
                max_pts = pts_total
                best_len = l_bg + l_or
                if best_len<PATH_LENGTH:
                    if len(p_bg)<2:
                        abort = "Zu wenige Midpoints für blauen/gelben Pfad."
                    elif pts_total==0:
                        abort = "Kein gültiger Pfad gefunden."
                    else:
                        abort = f"Nur {best_len:.2f} m Pfadlänge erreicht."
                else:
                    abort = ""

        # --- Pfad in MarkerArray ---
        path_markers = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL
        clr.header = msg.header; clr.ns='best_path'; clr.id=40000
        path_markers.markers.append(clr)

        if len(best_bg)+len(best_or)>=2:
            pts = [Point(
                x=float(x), y=float(y), z=0.0
            ) for (x,y) in best_bg + best_or]
            n_bg = len(best_bg)
            m = Marker()
            m.header = msg.header
            m.ns     = 'best_path'
            m.id     = 30000
            m.type   = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.08
            m.points  = pts
            m.colors  = []
            for i in range(len(pts)):
                if i < n_bg:
                    m.colors.append(ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0))
                else:
                    m.colors.append(ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0))
            path_markers.markers.append(m)

        # Publisher
        self.marker_pub.publish(markers)
        self.path_pub.publish(path_markers)

        # FPS
        dt = time.time() - t0
        self._frame_times.append(dt)
        if len(self._frame_times)>10:
            self._frame_times.pop(0)
        avg = sum(self._frame_times)/len(self._frame_times)
        fps = Float32(data=(1.0/avg if avg>0 else 0.0))
        self.fps_pub.publish(fps)

        # Warnung
        if abort and best_len<PATH_LENGTH:
            self.get_logger().warn(f"Pfad < {PATH_LENGTH}m! Grund: {abort}")

def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
