import numpy as np


class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(
        self,
        dt: float,
        lookahead_distance: float = 0.5,
        logger=None,
        simulation: bool = False,
    ):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._logger = logger
        self._lookahead_distance: float = lookahead_distance
        self._path: list[tuple[float, float]] = []
        self._simulation: bool = simulation

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 4.11. Complete the function body with your code (i.e., compute v and w).

        if not self._path:
            return 0.0, 0.0

        _, closest_idx = self._find_closest_point(x, y)  # punto en la ruta más cercano, y su índice
        L = self._find_target_point((x, y), closest_idx)  # punto objetivo, al que hay que llegar

        dx = L[0] - x
        dy = L[1] - y
        l = np.linalg.norm([dx, dy])  # distancia entre el robot y el punto objetivo (l en la fórmula pure pursuit)

        if l == 0.0:
            return 0.0, 0.0

        beta = np.arctan2(dy, dx)  # calculo beta para sacar alpha después
        alpha = (beta - theta + np.pi) % (2 * np.pi) - np.pi  # Porque queremos el giro mínimo con signo (de ahi esa transformación)

        if abs(alpha) > np.pi / 2:
            return 0.0, 0.5 * np.sign(alpha)  # si alpha es muy grande, giro despacio

        v = 0.22
        w = 2 * v * np.sin(alpha) / l  # fórmula tal cual de las diapos pure pursuit

        return v, w


    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value

    def _find_closest_point(
        self, x: float, y: float
    ) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        # TODO: 4.9. Complete the function body (i.e., find closest_xy and closest_idx).
        path = np.array(self._path)

        dx = path[:, 0] - x
        dy = path[:, 1] - y
        squared_distances = dx**2 + dy**2
        closest_idx = int(np.argmin(squared_distances))
        closest_xy = (path[closest_idx][0], path[closest_idx][1])

        return closest_xy, closest_idx

    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        # TODO: 4.10. Complete the function body with your code (i.e., determine target_xy).
        if not self._path:
            return origin_xy

        for i in range(origin_idx, len(self._path)):
            point = self._path[i]

            if np.linalg.norm(np.array(point) - np.array(origin_xy)) >= self._lookahead_distance:
                return point

        return self._path[-1]

      