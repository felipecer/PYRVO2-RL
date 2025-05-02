# test_observation_bounds_robust.py

import itertools
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper, ObsMode

OUT_FILE = "observation_bounds_report.txt"

def stringify_mode(m):
    return "Cartesian" if m == ObsMode.Cartesian else "Polar"

def summarize_bounds(wrapper, params):
    """
    Calls get_observation_bounds() on wrapper, then returns
    a string block describing:
      - construction params
      - shapes of low/high
      - number of info lines
      - each info line
    """
    b = wrapper.get_observation_bounds()    
    low   = np.asarray(b["low"])
    high  = np.asarray(b["high"])
    info  = list(b["info"])
    mode  = b["mode"]
    # build summary text
    lines = []
    lines.append(f"---\nParameters: {params}")
    lines.append(f" mode reported     : {mode}")
    lines.append(f" array length      : {low.shape[0]}")
    lines.append(f" low.shape,high.shape: {low.shape},{high.shape}")
    lines.append(f" info lines ({len(info)}):")
    for idx, desc in enumerate(info):
        lines.append(f"   [{idx:02d}] {desc}")
    # sanity checks
    if low.shape != high.shape:
        lines.append("  !! ERROR: low/high shapes differ")
    if not np.all(low <= high):
        lines.append("  !! ERROR: some low > high")
    lines.append("")  # blank line
    return "\n".join(lines)

def main():
    # Define parameter grids
    modes         = [ObsMode.Cartesian, ObsMode.Polar]
    masks         = [False, True]
    lidars        = [False, True]
    max_neighbors = [0, 1, 3, 10]
    lidar_counts  = [4, 16]

    report_lines = []
    report_lines.append("Observation Bounds Robust Report")
    report_lines.append("="*36)
    report_lines.append("")

    # Iterate all combinations
    for mode, use_mask, use_lidar, max_n in itertools.product(modes, masks, lidars, max_neighbors):
        # if no lidar, skip varying lidar_count
        if not use_lidar:
            # single test with default lidar_count
            wrapper = RVO2RLWrapper(
                max_neighbors=max_n,
                mode=mode,
                use_obs_mask=use_mask,
                use_lidar=False
            )
            params = {
                "mode": stringify_mode(mode),
                "max_neighbors": max_n,
                "use_obs_mask": use_mask,
                "use_lidar": False
            }
            report_lines.append(summarize_bounds(wrapper, params))
        else:
            for lc in lidar_counts:
                wrapper = RVO2RLWrapper(
                    max_neighbors=max_n,
                    mode=mode,
                    use_obs_mask=use_mask,
                    use_lidar=True,
                    lidar_count=lc
                )
                params = {
                    "mode": stringify_mode(mode),
                    "max_neighbors": max_n,
                    "use_obs_mask": use_mask,
                    "use_lidar": True,
                    "lidar_count": lc
                }
                report_lines.append(summarize_bounds(wrapper, params))

    # Write to file
    with open(OUT_FILE, "w") as f:
        f.write("\n".join(report_lines))

    print(f"Finished. See {OUT_FILE} for full report.")

if __name__ == "__main__":
    main()
