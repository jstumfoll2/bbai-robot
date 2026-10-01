# GPS

EM-506 (SiRF) on `/dev/ttyS2` (UART3), 4800 baud. Indoors it tracks poorly.

- `libgps-bbai-ttyS2-4800.patch`: changes to [wdalmut/libgps](https://github.com/wdalmut/libgps) at
  `ef90e80` so it opens `/dev/ttyS2` at 4800 baud.
- The ROS publisher is `ros/bbai_base/scripts/gps_node.py`.
