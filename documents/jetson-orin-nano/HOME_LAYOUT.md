# Jetson source and data layout

`/home/bee/Lv2_bee_Assignment` is the canonical checkout on branch `develop`. It contains the latest fetched upstream `develop`, the merged `sjh/feature/isaac-setup` work, the 2D map converter, and the GHJ Jetson tuning. The GHJ checkout was removed only after checking `/proc` and confirming no live process referenced it.

`/home/sjh/Lv2_bee_Assignment` is still active: a Claude process (PID 3944) and a shell (PID 11809) have that checkout as their working directory. Its Isaac experiment logs and Python environment are retained until that session closes. The generated Isaac loop `install`, `log`, `build`, and `__pycache__` directories were removed; `real_logs`, bag data, and `.venv` remain.

The stale ROS `build/` and colcon `log/` under canonical Bee were removed after confirming there was no active build process. `install/` remains but is from the older checkout and must be rebuilt from Bee before running the updated source. The robot was not restarted by this cleanup.

Jetson has no separate SSD mounted; `/home` uses the root NVMe. MCAP records, real experiment logs, and virtual environments were preserved because they contain useful diagnostics or dependencies.
