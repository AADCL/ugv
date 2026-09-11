import importlib.util
from pathlib import Path
import unittest
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid
from wheeltec_safety.safety_core import DirectionalStopRegion

spec=importlib.util.spec_from_file_location('gate',str(Path(__file__).resolve().parents[1]/'scripts'/'cmd_vel_safety_gate.py'))
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class CostmapSemantics(unittest.TestCase):
    def test_small_forward_speeds_are_preserved(self):
        gate=module.CmdVelSafetyGate.__new__(module.CmdVelSafetyGate)
        gate.max_linear_speed=.2
        gate.max_angular_speed=.4
        gate.linear_deadband=.0001
        gate.angular_deadband=.01
        gate.allow_reverse=False
        for v in [.001,.005,.01]:
            command=Twist();command.linear.x=v;command.angular.z=.2
            out=gate.sanitized_command(command)
            self.assertEqual(out.linear.x,v)
            self.assertEqual(out.angular.z,.2)

    def test_full_footprint_uses_real_obstacles_not_inflated_cells(self):
        gate=module.CmdVelSafetyGate.__new__(module.CmdVelSafetyGate)
        gate.stop_region=DirectionalStopRegion(.25,.25,.2,.02,.18,.18,.2,.2,.4,1.5,12,.005,.01)
        gate.costmap_lethal_threshold=100
        gate.unknown_is_unsafe=True
        transform=TransformStamped()
        transform.transform.rotation.w=1
        grid=OccupancyGrid()
        grid.info.resolution=.05
        grid.info.width=grid.info.height=20
        grid.info.origin.position.x=grid.info.origin.position.y=-.5
        grid.info.origin.orientation.w=1
        command=Twist();command.linear.x=.1
        # x=.325, y=.025 is in the forward stop zone.
        for value,expected in [(0,False),(99,False),(100,True),(-1,True)]:
            grid.data=[0]*400
            grid.data[10*20+16]=value
            self.assertEqual(gate.costmap_stop_check(grid,transform,command)[0],expected)

if __name__=='__main__':unittest.main()
