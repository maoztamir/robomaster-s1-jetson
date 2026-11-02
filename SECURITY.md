# Security

Report suspected credential exposure or security vulnerabilities privately using
GitHub's private vulnerability reporting for this repository, if enabled. If it
is unavailable, open an issue requesting a private reporting channel without
including sensitive details, credentials, or exploit instructions.

Do not upload camera captures, room maps, calibration archives, local deployment
settings, SSH keys, or access tokens in issues or pull requests. Keep generated
data under the ignored `output/` and `maps/hardware/` directories.

ROS domain IDs and DDS discovery profiles do not authenticate publishers. Run
hardware control on a trusted, isolated network. Only one controller should own
`/cmd_vel`, and a physical power cutoff must remain accessible during trials.

This project is experimental. Supported hardware configurations and motion
limitations are described in the README and hardware guides.
