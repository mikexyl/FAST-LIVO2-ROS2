# Local temporal-only rollback verification

Active code retains temporal submaps with 10-second windows and 5-second overlap.
Motion/overlap scheduling, parameters, analytics and batch selection were removed.
The motion experiment source and results remain in the adjacent motion-submaps archive.

The native library, ROS interfaces and test executables built successfully on this
machine in `.ros2/temporal-submaps-revert`. Both native CTest cases passed. The
Python suite passed 14 checks, with the optional DDS check subsequently run and
passed separately: 15 Python/integration checks in total. Native ROS tests ran
with localhost networking outside the sandbox after its socket restriction.

`source-verification.json` confirms exact historical temporal source hashes for
the mapping implementation, map buffer header, temporal scheduler/exporter,
analytics message, live recorder and bounded snapshot writer. The current
MapClosures/PCM/CBS integration and numerical correspondence guards remain.

No dataset replay or parameter sweep was launched for this code rollback.
