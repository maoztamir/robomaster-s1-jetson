# Contributing

Explain the problem, expected behavior, and validation in each pull request.
Separate hardware observations from simulation results. Never run driving tests
as part of CI, and do not weaken watchdogs, obstacle guards, or operator stops
to make a trial pass.

Run the Python unit tests, shell syntax checks, and C++ build/test commands from
the README. ROS integration checks require a sourced ROS installation; report
any skipped checks. Include the ROS distribution and relevant hardware details
when reporting a problem, and redact local addresses, usernames, and paths.

Keep local deployment settings, maps, camera captures, and generated assets out
of commits. Retain upstream license notices and document the origin and license
of any new third-party assets.
