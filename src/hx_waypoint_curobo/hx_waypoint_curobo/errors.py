class WaypointError(RuntimeError):
    """Base class for user-visible waypoint pipeline failures."""


class ConfigurationError(WaypointError):
    """Environment or robot configuration is incomplete or inconsistent."""


class TrajectoryFormatError(WaypointError):
    """The source or compiled trajectory violates its schema."""


class CuroboUnavailableError(WaypointError):
    """The locked cuRobo runtime is unavailable or incompatible."""


class PlanningError(WaypointError):
    """cuRobo could not produce a complete safe trajectory."""


class ValidationError(WaypointError):
    """A trajectory or recording failed a mandatory safety gate."""
