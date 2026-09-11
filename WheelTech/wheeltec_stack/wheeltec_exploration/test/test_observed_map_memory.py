import importlib.util
from pathlib import Path
import unittest
import numpy as np

path = Path(__file__).resolve().parents[1]/'scripts/observed_map_memory.py'
spec = importlib.util.spec_from_file_location('observed_memory', str(path))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ObservationMemoryTests(unittest.TestCase):
    def setUp(self):
        self.memory = module.OccupancyMemory(.1, 2., 10000)

    def cell(self, x, y):
        return self.memory.cells[int(np.floor(y/.1))-self.memory.y0,
                                 int(np.floor(x/.1))-self.memory.x0]

    def test_unknown_does_not_erase_old_obstacle_or_free(self):
        self.memory.integrate([.05, .15], [.05, .05], [100, 0])
        self.assertFalse(self.memory.integrate([.05, .15], [.05, .05], [-1, -1]))
        self.assertEqual(self.cell(.05, .05), 100)
        self.assertEqual(self.cell(.15, .05), 0)
        self.assertEqual(self.cell(.25, .05), -1)

    def test_fresh_observation_clears_and_remarks_dynamic_obstacle(self):
        for value in (100, 0, 100):
            self.memory.integrate([.05], [.05], [value])
            self.assertEqual(self.cell(.05, .05), value)

    def test_rolling_window_moves_but_memory_stays(self):
        self.memory.integrate([.05], [.05], [100])
        self.memory.integrate([-2.05, 3.05], [-2.05, 3.05], [0, 100])
        self.assertEqual(self.cell(.05, .05), 100)
        self.assertEqual(self.cell(-2.05, -2.05), 0)
        self.assertEqual(self.cell(3.05, 3.05), 100)

    def test_collision_wins_when_cells_project_to_same_destination(self):
        self.memory.integrate([.01, .02], [.01, .02], [100, 0])
        self.assertEqual(self.cell(.05, .05), 100)

    def test_invalid_or_oversized_input_keeps_existing_map(self):
        original = self.memory.cells.copy()
        for x, y, value in [([0.], [0.], [99]), ([float('nan')], [0.], [0]),
                            ([10000.], [10000.], [0])]:
            with self.assertRaises(ValueError):
                self.memory.integrate(x, y, value)
            np.testing.assert_array_equal(self.memory.cells, original)


if __name__ == '__main__':
    unittest.main()
