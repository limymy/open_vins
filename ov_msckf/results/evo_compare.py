import argparse
from evo.tools import file_interface
from evo.tools import plot
import matplotlib.pyplot as plt
# from evo.tools import statistics
# from evo.tools import trajectory
from evo.core import sync
from evo.core import metrics

from evo.tools import log
log.configure_logging(verbose=True, debug=True, silent=False)

from evo.tools.settings import SETTINGS
# SETTINGS.plot_figsize = [6, 6]
SETTINGS.plot_split = True
SETTINGS.plot_usetex = False

import copy
import pprint

def main():
    aparser = argparse.ArgumentParser(description="evo tools")
    aparser.add_argument("--est", required=True)
    aparser.add_argument("--ref", required=True)

    args = aparser.parse_args()

    traj_ref = file_interface.read_tum_trajectory_file(args.ref)
    traj_est = file_interface.read_tum_trajectory_file(args.est)
    print(f'ref: {traj_ref}')
    print(f'traj_est: {traj_est}')

# 轨迹同步
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff=0.01)
    print(f'ref: {traj_ref}')
    print(f'traj_est: {traj_est}')

# 轨迹对齐
    traj_est_aligned = copy.deepcopy(traj_est)
    traj_est_aligned.align(traj_ref, correct_scale=False, correct_only_scale=False)
# traj_est.align(traj_ref, correct_scale=False, correct_only_scale=False)

    traj_by_label = {
    "estimate": traj_est_aligned,
    "reference": traj_ref
    }
    fig = plt.figure()
    plot.trajectories(fig, traj_by_label, plot.PlotMode.xyz)

## APE(absolute pose error) metric
# Create an instance of the APE class and process the data
    pose_relation = metrics.PoseRelation.translation_part
    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data([traj_ref, traj_est_aligned])
# Get all avalaible statistics at once in a dictionary
    ape_stats = ape_metric.get_all_statistics()
    pprint.pprint(ape_stats)
# Plot the APE values and statistics
    seconds_from_start = [t - traj_est_aligned.timestamps[0] for t in traj_est_aligned.timestamps]
    fig = plt.figure()
    plot.error_array(fig.gca(), ape_metric.error, x_array=seconds_from_start,
                    statistics={s:v for s,v in ape_stats.items() if s != "sse"},
                    name="APE", title="APE w.r.t. " + ape_metric.pose_relation.value, xlabel="$t$ (s)")

# Plot the trajectory with colormapping of the APE
    plot_mode = plot.PlotMode.xy
    fig = plt.figure()
    ax = plot.prepare_axis(fig, plot_mode)
    plot.traj(ax, plot_mode, traj_ref, style="--", color="gray", label="Reference")
    plot.traj_colormap(ax, traj_est_aligned, ape_metric.error, plot_mode, min_map=ape_stats["min"], max_map=ape_stats["max"])
    ax.legend()

## RPE(relative pose error) metric
# settings
    delta = 1
    delta_unit = metrics.Unit.frames
    all_pairs = False
# Create an instance of the RPE class and process the data
    rpe_metric = metrics.RPE(pose_relation, delta, delta_unit, all_pairs)
    rpe_metric.process_data([traj_ref, traj_est])
# Get all avalaible statistics at once in a dictionary
    rpe_stats = rpe_metric.get_all_statistics()
    pprint.pprint(rpe_stats)
# Plot the RPE values and statistics
    traj_ref_plot = copy.deepcopy(traj_ref)
    traj_est_plot = copy.deepcopy(traj_est)
    traj_ref_plot.reduce_to_ids(rpe_metric.delta_ids)
    traj_est_plot.reduce_to_ids(rpe_metric.delta_ids)
    seconds_from_start = [t - traj_est.timestamps[0] for t in traj_est.timestamps[1:]]
    fig = plt.figure()
    plot.error_array(fig.gca(), rpe_metric.error, x_array=seconds_from_start,
                    statistics={s:v for s,v in rpe_stats.items() if s != "sse"},
                    name="RPE", title="RPE w.r.t. " + rpe_metric.pose_relation.value, xlabel="$t$ (s)")
# Plot the trajectory with colormapping of the RPE
    plot_mode = plot.PlotMode.xy
    fig = plt.figure()
    ax = plot.prepare_axis(fig, plot_mode)
    plot.traj(ax, plot_mode, traj_ref_plot, style="--", color="gray", label="Reference")
    plot.traj_colormap(ax, traj_est_plot, rpe_metric.error, plot_mode, min_map=rpe_stats["min"], max_map=rpe_stats["max"])
    ax.legend()

    plt.show()

if __name__ == "__main__":
    main()
