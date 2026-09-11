#!/usr/bin/env python3

import collections
import math
import threading
import time

import rospy
import tf2_ros
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String


class FrontierCostmapAdapter:
    def __init__(self):
        input_topic = rospy.get_param(
            "~input_topic", "/move_base/global_costmap/costmap"
        )
        output_topic = rospy.get_param("~output_topic", "/exploration/frontier_map")
        updates_topic = rospy.get_param(
            "~updates_topic", "/move_base/global_costmap/costmap_updates"
        )
        self.lethal_threshold = rospy.get_param("~lethal_threshold", 99)
        self.min_frontier_cells = rospy.get_param("~min_frontier_cells", 5)
        self.min_frontier_size = rospy.get_param('/explore/min_frontier_size', 0.50)
        self.completion_hold_time = rospy.get_param("~completion_hold_time", 30.0)
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_link")
        self.map = None
        self.last_frontier_time = None
        self.map_lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            output_topic, OccupancyGrid, queue_size=1, latch=True
        )
        self.frontier_status = rospy.Publisher('/exploration/frontier_available', Bool, queue_size=1)
        self.validated_status = rospy.Publisher('/exploration/frontier_status', String, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, OccupancyGrid, self.map_callback, queue_size=1
        )
        self.update_subscriber = rospy.Subscriber(
            updates_topic, OccupancyGridUpdate, self.update_callback, queue_size=10
        )
        rospy.loginfo(
            "Frontier map adapter %s + %s -> %s "
            "(lethal >= %d, completion hold %.1f s)",
            input_topic,
            updates_topic,
            output_topic,
            self.lethal_threshold,
            self.completion_hold_time,
        )

    def map_callback(self, source):
        with self.map_lock:
            self.map = OccupancyGrid()
            self.map.header = source.header
            self.map.info = source.info
            self.map.data = list(source.data)
            self.publish_map()

    def update_callback(self, update):
        with self.map_lock:
            if self.map is None:
                return
            width = self.map.info.width
            height = self.map.info.height
            if (update.x + update.width > width or update.y + update.height > height
                    or update.width == 0 or update.height == 0
                    or len(update.data) != update.width*update.height
                    or (update.header.frame_id and update.header.frame_id != self.map.header.frame_id)):
                rospy.logwarn_throttle(2.0, "Ignoring out-of-bounds costmap update")
                self.validated_status.publish(String(data='INVALID'))
                self.frontier_status.publish(Bool(data=False))
                return
            for row in range(update.height):
                source_start = row * update.width
                target_start = (update.y + row) * width + update.x
                self.map.data[target_start : target_start + update.width] = update.data[
                    source_start : source_start + update.width
                ]
            self.map.header.stamp = update.header.stamp
            self.publish_map()

    def publish_map(self):
        if (self.map.info.resolution <= 0 or self.map.info.width <= 0
                or self.map.info.height <= 0
                or len(self.map.data) != self.map.info.width*self.map.info.height
                or not self.map.header.frame_id
                or abs((rospy.Time.now()-self.map.header.stamp).to_sec()) > 2.5):
            self.validated_status.publish(String(data='INVALID'))
            self.frontier_status.publish(Bool(data=False))
            return
        output = OccupancyGrid()
        output.header = self.map.header
        output.info = self.map.info
        output.data = [
            -1 if value < 0 else 100 if value >= self.lethal_threshold else 0
            for value in self.map.data
        ]
        has_frontier = self.has_reachable_frontier_cluster(output)
        if has_frontier is None:
            self.validated_status.publish(String(data='INVALID'))
            self.frontier_status.publish(Bool(data=False))
            return
        self.validated_status.publish(String(data='AVAILABLE' if has_frontier else 'EMPTY'))
        self.frontier_status.publish(Bool(data=has_frontier))
        now = time.monotonic()
        if has_frontier:
            self.last_frontier_time = now
        elif self.last_frontier_time is None:
            # A valid observed room can be covered before the first movement.
            pass
        # Publish the current map even when temporarily empty. Completion is
        # decided by the supervisor; retaining old geometry can send stale goals.
        self.publisher.publish(output)

    def has_reachable_frontier_cluster(self, grid):
        data = grid.data
        width = grid.info.width
        height = grid.info.height
        try:
            transform = self.tf_buffer.lookup_transform(
                grid.header.frame_id,
                self.robot_base_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(2.0, "Frontier reachability TF unavailable: %s", error)
            return None

        if abs((rospy.Time.now()-transform.header.stamp).to_sec()) > 0.5:
            return None

        robot_x = int(
            math.floor(
                (transform.transform.translation.x - grid.info.origin.position.x)
                / grid.info.resolution
            )
        )
        robot_y = int(
            math.floor(
                (transform.transform.translation.y - grid.info.origin.position.y)
                / grid.info.resolution
            )
        )
        if not (0 <= robot_x < width and 0 <= robot_y < height):
            return None

        start = robot_y * width + robot_x
        if data[start] != 0:
            start = self.nearest_free_cell(data, width, height, robot_x, robot_y)
            if start is None:
                return None

        reachable = {start}
        pending = collections.deque([start])
        while pending:
            index = pending.popleft()
            x = index % width
            y = index // width
            for neighbor_x, neighbor_y in (
                (x - 1, y),
                (x + 1, y),
                (x, y - 1),
                (x, y + 1),
            ):
                if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                    continue
                neighbor = neighbor_y * width + neighbor_x
                if neighbor not in reachable and data[neighbor] == 0:
                    reachable.add(neighbor)
                    pending.append(neighbor)

        # Match explore_lite: cluster UNKNOWN cells adjacent to reachable FREE
        # cells, not the free-side boundary (which can have a different length).
        frontier_cells = set()
        for index in reachable:
            x = index % width
            y = index // width
            if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                continue
            for neighbor in (index-1, index+1, index-width, index+width):
                if data[neighbor] < 0:
                    frontier_cells.add(neighbor)

        # Match the goal producer's physical threshold (0.50 m by default).
        required = max(self.min_frontier_cells,
                       int(math.ceil(self.min_frontier_size/grid.info.resolution-1e-9)))
        return self.has_large_cluster(frontier_cells, width, required)

    @staticmethod
    def nearest_free_cell(data, width, height, robot_x, robot_y):
        for radius in range(1, 11):
            for y in range(max(0, robot_y - radius), min(height, robot_y + radius + 1)):
                for x in range(max(0, robot_x - radius), min(width, robot_x + radius + 1)):
                    index = y * width + x
                    if data[index] == 0:
                        return index
        return None

    def has_large_cluster(self, frontier_cells, width, required=None):
        required = self.min_frontier_cells if required is None else required
        while frontier_cells:
            pending = [frontier_cells.pop()]
            cluster_size = 0
            while pending:
                index = pending.pop()
                cluster_size += 1
                if cluster_size >= required:
                    return True
                x = index % width
                y = index // width
                for neighbor_y in range(y - 1, y + 2):
                    for neighbor_x in range(x - 1, x + 2):
                        if not 0 <= neighbor_x < width or neighbor_y < 0:
                            continue
                        neighbor = neighbor_y * width + neighbor_x
                        if neighbor in frontier_cells:
                            frontier_cells.remove(neighbor)
                            pending.append(neighbor)
        return False


def main():
    rospy.init_node("wheeltec_frontier_costmap_adapter")
    FrontierCostmapAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
