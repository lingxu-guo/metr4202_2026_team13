"""Shared OccupancyGrid geometry helpers for navigation task providers."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple

from nav_msgs.msg import OccupancyGrid


GridIndex = int
GridPoint = Tuple[float, float]


@dataclass(frozen=True)
class GridMap:
    """Immutable, yaw-aware snapshot of a ROS OccupancyGrid."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    frame_id: str
    data: Tuple[int, ...]

    @classmethod
    def from_message(cls, message: OccupancyGrid) -> 'GridMap':
        """Validate and copy an OccupancyGrid into an immutable snapshot."""
        width = int(message.info.width)
        height = int(message.info.height)
        resolution = float(message.info.resolution)
        if width <= 0 or height <= 0:
            raise ValueError('OccupancyGrid dimensions must be positive.')
        if resolution <= 0.0:
            raise ValueError('OccupancyGrid resolution must be positive.')
        if len(message.data) != width * height:
            raise ValueError('OccupancyGrid data length does not match dimensions.')

        orientation = message.info.origin.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return cls(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            origin_yaw=math.atan2(sin_yaw, cos_yaw),
            frame_id=message.header.frame_id or 'map',
            data=tuple(int(value) for value in message.data),
        )

    def index_to_world(self, index: GridIndex) -> GridPoint:
        """Return the world position of a cell centre."""
        row, column = divmod(index, self.width)
        return self.local_to_world((
            (column + 0.5) * self.resolution,
            (row + 0.5) * self.resolution,
        ))

    def local_to_world(self, point: GridPoint) -> GridPoint:
        """Transform a point in grid-local metres into the world frame."""
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * point[0] - sine * point[1],
            self.origin_y + sine * point[0] + cosine * point[1],
        )

    def world_to_cell(self, point: GridPoint) -> Tuple[int, int] | None:
        """Return the cell containing a world point, or None outside the map."""
        delta_x = point[0] - self.origin_x
        delta_y = point[1] - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        column = math.floor(local_x / self.resolution)
        row = math.floor(local_y / self.resolution)
        if 0 <= row < self.height and 0 <= column < self.width:
            return row, column
        return None

    def world_to_index(self, point: GridPoint) -> GridIndex | None:
        """Return the cell index containing a world point."""
        cell = self.world_to_cell(point)
        if cell is None:
            return None
        return cell[0] * self.width + cell[1]


def classify_cells(
    grid: GridMap,
    occupied_threshold: int,
) -> Tuple[list[bool], list[bool], list[bool]]:
    """Return free, occupied, and unknown masks."""
    free = [0 <= value < occupied_threshold for value in grid.data]
    occupied = [value >= occupied_threshold for value in grid.data]
    unknown = [value < 0 for value in grid.data]
    return free, occupied, unknown


def neighbors4(index: GridIndex, width: int, height: int) -> Iterable[GridIndex]:
    """Yield four-connected neighbours inside the grid."""
    row, column = divmod(index, width)
    for row_delta, column_delta in ((-1, 0), (0, -1), (0, 1), (1, 0)):
        next_row = row + row_delta
        next_column = column + column_delta
        if 0 <= next_row < height and 0 <= next_column < width:
            yield next_row * width + next_column


def neighbors8(index: GridIndex, width: int, height: int) -> Iterable[GridIndex]:
    """Yield eight-connected neighbours inside the grid."""
    row, column = divmod(index, width)
    for row_delta in (-1, 0, 1):
        for column_delta in (-1, 0, 1):
            if row_delta == 0 and column_delta == 0:
                continue
            next_row = row + row_delta
            next_column = column + column_delta
            if 0 <= next_row < height and 0 <= next_column < width:
                yield next_row * width + next_column


def inflate_mask(
    blocked: Sequence[bool],
    width: int,
    height: int,
    resolution: float,
    radius_metres: float,
) -> list[bool]:
    """Inflate blocked cells by a circular world-space radius."""
    inflated = list(blocked)
    if radius_metres <= 0.0:
        return inflated
    radius_cells = int(math.ceil(radius_metres / resolution))
    offsets = [
        (row_delta, column_delta)
        for row_delta in range(-radius_cells, radius_cells + 1)
        for column_delta in range(-radius_cells, radius_cells + 1)
        if math.hypot(row_delta, column_delta) * resolution
        <= radius_metres + 1e-9
    ]
    for index, is_blocked in enumerate(blocked):
        if not is_blocked:
            continue
        row, column = divmod(index, width)
        for row_delta, column_delta in offsets:
            next_row = row + row_delta
            next_column = column + column_delta
            if 0 <= next_row < height and 0 <= next_column < width:
                inflated[next_row * width + next_column] = True
    return inflated


def connected_component(
    traversable: Sequence[bool],
    start: GridIndex,
    width: int,
    height: int,
) -> list[bool]:
    """Return the four-connected traversable component containing start."""
    connected = [False] * len(traversable)
    if start < 0 or start >= len(traversable) or not traversable[start]:
        return connected
    queue = deque([start])
    connected[start] = True
    while queue:
        index = queue.popleft()
        for neighbor in neighbors4(index, width, height):
            if traversable[neighbor] and not connected[neighbor]:
                connected[neighbor] = True
                queue.append(neighbor)
    return connected


def supercover_cells(
    grid: GridMap,
    start: GridPoint,
    end: GridPoint,
) -> list[GridIndex] | None:
    """Return all cells touched by a line, conservatively including corners."""
    start_cell = grid.world_to_cell(start)
    end_cell = grid.world_to_cell(end)
    if start_cell is None or end_cell is None:
        return None

    y0, x0 = start_cell
    y1, x1 = end_cell
    delta_x = x1 - x0
    delta_y = y1 - y0
    nx = abs(delta_x)
    ny = abs(delta_y)
    sign_x = 1 if delta_x > 0 else -1 if delta_x < 0 else 0
    sign_y = 1 if delta_y > 0 else -1 if delta_y < 0 else 0

    x = x0
    y = y0
    cells = [y * grid.width + x]
    ix = 0
    iy = 0
    while ix < nx or iy < ny:
        left = (1 + 2 * ix) * ny
        right = (1 + 2 * iy) * nx
        if left == right:
            if sign_x:
                cells.append(y * grid.width + x + sign_x)
            if sign_y:
                cells.append((y + sign_y) * grid.width + x)
            x += sign_x
            y += sign_y
            ix += 1
            iy += 1
        elif left < right:
            x += sign_x
            ix += 1
        else:
            y += sign_y
            iy += 1
        index = y * grid.width + x
        if index not in cells:
            cells.append(index)
    return cells
