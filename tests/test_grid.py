import unittest
from nidar_slam.occupancy_grid import OccupancyGrid


class OccupancyGridTests(unittest.TestCase):
    def test_world_to_cell_origin(self):
        grid = OccupancyGrid(4.0, 4.0, 0.1)
        self.assertIsNotNone(grid.world_to_cell(0.0, 0.0))

    def test_ray_marks_endpoint_occupied(self):
        grid = OccupancyGrid(4.0, 4.0, 0.1)
        grid.update_ray(0.0, 0.0, 0.5, 0.0, hit=True)
        end = grid.world_to_cell(0.5, 0.0)
        self.assertGreater(grid.log_odds[end], 0.0)


if __name__ == "__main__":
    unittest.main()
