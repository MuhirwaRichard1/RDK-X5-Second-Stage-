# navbot_navigation
- `behaviour_manager` — SEEK/AVOID/APPROACH/STOP state machine; owns `/goal`.
- `local_planner` — free-space + setpoint → `/cmd_vel` (Twist, 20Hz). No safe direction → rotate-search.
