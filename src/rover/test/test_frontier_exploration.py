"""Unit tests for FrontierExploration occupancy grid frontier detection."""

import numpy as np

from rover.frontier_exploration import FrontierExploration


def test_frontier_detection_on_partially_explored_grid():
    """Partially explored grid must yield ranked reachable frontier goals."""
    grid = np.full((60, 60), -1, dtype=np.int8)
    # Carve a 20x20 free room in the center
    grid[20:40, 20:40] = 0
    # Add one occupied wall on the left side
    grid[20:40, 20] = 100

    explorer = FrontierExploration(min_cluster_size=4, robot_radius_m=0.25)
    goals, _, stats = explorer.extract_frontiers(
        grid_data=grid,
        width=60,
        height=60,
        resolution=0.05,
        origin_x=-1.5,
        origin_y=-1.5,
        robot_x=0.0,
        robot_y=0.0,
    )
    assert stats['total_frontiers'] == len(goals)
    assert len(goals) >= 1
    assert stats['is_fully_mapped'] is False
    assert stats['explored_area_m2'] > 0.0


def test_fully_explored_grid_reports_mapped():
    """A grid with sufficient free cells and zero unknown cells is marked fully mapped."""
    grid = np.zeros((30, 30), dtype=np.int8)
    explorer = FrontierExploration()
    goals, _, stats = explorer.extract_frontiers(grid, 30, 30, 0.05, 0.0, 0.0)
    assert goals == []
    assert stats['is_fully_mapped'] is True
