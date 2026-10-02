# Robot system document

LaTeX write-up of the whole robot: hardware bring-up (device tree, PRU firmware, librobotcontrol
changes), how the devices and computers connect, the ROS nodes, topics and frames, and the
navigation math (wheel odometry, gyro and compass heading, GPS, the robot_localization EKF,
gmapping and hector SLAM), plus the operating modes and known issues.

The built PDF is [`bbai-robot-system.pdf`](bbai-robot-system.pdf). Notation and the sensor error
models follow the TWIP thesis in [jstumfoll2/twip-dmso](https://github.com/jstumfoll2/twip-dmso).

Build with `make` (needs `pdflatex` and `bibtex`; on Ubuntu, `texlive-latex-extra texlive-science`).
