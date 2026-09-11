#!/usr/bin/env python3
"""Remember observed local occupancy. No PCD, terrain fitting or map-file IO."""
import math
import time
import numpy as np


class OccupancyMemory:
    """Latest observed local state wins; absence of observation never clears."""
    def __init__(self, resolution=.05, initial_size_m=12., max_cells=4000000):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid resolution')
        self.resolution = resolution
        self.max_cells = max_cells
        side = int(math.ceil(initial_size_m/resolution))
        if side <= 0 or side*side > max_cells:
            raise ValueError('initial map exceeds cell budget')
        self.x0 = self.y0 = -(side//2)
        self.cells = np.full((side, side), -1, dtype=np.int8)
        self.generation = 0

    def integrate(self, x, y, values):
        x, y = np.asarray(x), np.asarray(y)
        values = np.asarray(values)
        if x.shape != y.shape or x.shape != values.shape:
            raise ValueError('malformed observation')
        if not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError('nonfinite coordinates')
        if not np.isin(values, [-1, 0, 100]).all():
            raise ValueError('expected uninflated ternary occupancy')
        known = values != -1
        if not known.any():
            return False
        ix = np.floor(x[known]/self.resolution).astype(np.int64)
        iy = np.floor(y[known]/self.resolution).astype(np.int64)
        values = values[known]
        h, w = self.cells.shape
        # Expand in two-metre blocks, preserving the world grid alignment.
        block = max(1, int(math.ceil(2./self.resolution)))
        nx = min(self.x0, (int(ix.min())//block)*block)
        ny = min(self.y0, (int(iy.min())//block)*block)
        right = max(self.x0+w, ((int(ix.max())//block)+1)*block)
        top = max(self.y0+h, ((int(iy.max())//block)+1)*block)
        if (right-nx)*(top-ny) > self.max_cells:
            raise ValueError('exploration memory cell budget exceeded')
        if (nx, ny, right, top) != (self.x0, self.y0, self.x0+w, self.y0+h):
            expanded = np.full((top-ny, right-nx), -1, dtype=np.int8)
            expanded[self.y0-ny:self.y0-ny+h, self.x0-nx:self.x0-nx+w] = self.cells
            self.cells, self.x0, self.y0 = expanded, nx, ny
        # Obstacle wins if rotated source cells land on the same world cell.
        for value in (0, 100):
            mask = values == value
            self.cells[iy[mask]-self.y0, ix[mask]-self.x0] = value
        self.generation += 1
        return True


class MemoryNode:
    def __init__(self):
        import rospy
        import tf2_ros
        from nav_msgs.msg import OccupancyGrid, MapMetaData
        from std_msgs.msg import String
        self.ros, self.Grid, self.String = rospy, OccupancyGrid, String
        self.frame = rospy.get_param('~global_frame', 'map')
        self.memory = OccupancyMemory(
            float(rospy.get_param('~resolution', .05)),
            float(rospy.get_param('~initial_size_m', 12.)),
            int(rospy.get_param('~max_cells', 4000000)))
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)
        self.last_stamp = rospy.Time(0)
        self.started = rospy.Time.now()
        self.map_pub = rospy.Publisher('/nav_static_map', OccupancyGrid, queue_size=1, latch=True)
        self.metadata_pub = rospy.Publisher('/nav_static_map_metadata', MapMetaData, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/exploration/map_status', String, queue_size=1, latch=True)
        # Bootstrap only: a fully unknown map lets move_base construct its local
        # costmap. It does not count as evidence or permit supervisor startup.
        self.publish(self.started)
        self.status_pub.publish(String(data='waiting: no local observation'))
        self.sub = rospy.Subscriber('/exploration/observed_local_map', OccupancyGrid,
                                   self.receive, queue_size=1)

    @staticmethod
    def planar_yaw(q):
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            raise ValueError('invalid orientation')
        if abs(q.x) > 1e-3 or abs(q.y) > 1e-3 or abs(q.z*q.z+q.w*q.w-1) > 1e-3:
            raise ValueError('occupancy map must be gravity aligned')
        return 2*math.atan2(q.z, q.w)

    def receive(self, message):
        started = time.monotonic()
        try:
            age = (self.ros.Time.now()-message.header.stamp).to_sec()
            if message.header.stamp <= self.last_stamp:
                return
            if not -.1 <= age <= 1.:
                raise ValueError('local observation stale')
            info = message.info
            if (info.width <= 0 or info.height <= 0
                    or len(message.data) != info.width*info.height
                    or abs(info.resolution-self.memory.resolution) > 1e-6
                    or not message.header.frame_id):
                raise ValueError('invalid local grid geometry')
            yaw = self.planar_yaw(info.origin.orientation)
            t = self.tf.lookup_transform(self.frame, message.header.frame_id,
                                         message.header.stamp, self.ros.Duration(.1)).transform
            rotation = self.planar_yaw(t.rotation)
            data = np.asarray(message.data, dtype=np.int8).reshape(info.height, info.width)
            yy, xx = np.nonzero(data != -1)
            lx, ly = (xx+.5)*info.resolution, (yy+.5)*info.resolution
            ox, oy = info.origin.position.x, info.origin.position.y
            x = ox+math.cos(yaw)*lx-math.sin(yaw)*ly
            y = oy+math.sin(yaw)*lx+math.cos(yaw)*ly
            wx = t.translation.x+math.cos(rotation)*x-math.sin(rotation)*y
            wy = t.translation.y+math.sin(rotation)*x+math.cos(rotation)*y
            if not self.memory.integrate(wx, wy, data[yy, xx]):
                self.status_pub.publish(self.String(data='refresh failed: no observed cells'))
                return
            self.last_stamp = message.header.stamp
            self.publish(message.header.stamp)
            self.status_pub.publish(self.String(data='ready: generation=%d processing_ms=%.2f' %
                (self.memory.generation, (time.monotonic()-started)*1000)))
        except Exception as error:
            self.status_pub.publish(self.String(data='refresh failed: '+str(error)))
            self.ros.logwarn_throttle(2., 'Observation memory: %s', error)

    def publish(self, stamp):
        output = self.Grid()
        output.header.frame_id, output.header.stamp = self.frame, stamp
        output.info.map_load_time = self.started
        output.info.resolution = self.memory.resolution
        output.info.height, output.info.width = self.memory.cells.shape
        output.info.origin.position.x = self.memory.x0*self.memory.resolution
        output.info.origin.position.y = self.memory.y0*self.memory.resolution
        output.info.origin.orientation.w = 1.
        output.data = self.memory.cells.ravel().tolist()
        self.map_pub.publish(output)
        self.metadata_pub.publish(output.info)


if __name__ == '__main__':
    import rospy
    rospy.init_node('wheeltec_observed_map_memory')
    MemoryNode()
    rospy.spin()
