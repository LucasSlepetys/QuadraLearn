"""Draw the positions exported by C++; no gait, IK or FK is calculated here."""

import csv
from pathlib import Path
from time import perf_counter

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import MaxNLocator


LEG_NAMES = ("FL", "FR", "RL", "RR")
POINT_NAMES = ("H", "P", "K", "F")  # Hip roll, hip pitch, knee, ball centre
LEG_COLOURS = ("#0077b6", "#e87924", "#009980", "#a24aa4")

# Locate the recording from this file, regardless of the terminal's folder.
CSV_PATH = Path(__file__).resolve().parent.parent / "simulation.csv"


def load_recording(csv_path):
    """1. Read time, world points, foot targets and phases from the CSV."""
    with csv_path.open(newline="", encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError("The CSV has no data rows. Run the C++ simulation first.")

    time_s = np.array([float(row["time_s"]) for row in rows])
    ground_z_W_m = float(rows[0]["ground_z_W_m"])

    # Index order: [frame, leg, point, coordinate]; coordinates are x, y, z.
    points_W = np.empty((len(rows), 4, 4, 3))
    targets_W = np.empty((len(rows), 4, 3))
    phases = np.empty((len(rows), 4), dtype=object)

    for frame, row in enumerate(rows):
        for leg, name in enumerate(LEG_NAMES):
            phases[frame, leg] = row[f"{name}_phase"]
            for point, point_name in enumerate(POINT_NAMES):
                for coordinate, axis in enumerate("xyz"):
                    points_W[frame, leg, point, coordinate] = float(
                        row[f"{name}_{point_name}_{axis}_W_m"]
                    )
            for coordinate, axis in enumerate("xyz"):
                targets_W[frame, leg, coordinate] = float(
                    row[f"{name}_foot_bottom_{axis}_W_m"]
                )

    if not (np.isfinite(time_s).all() and np.isfinite(points_W).all()
            and np.isfinite(targets_W).all() and np.isfinite(ground_z_W_m)):
        raise ValueError("The CSV contains invalid numbers. Check the C++ targets and IK results.")
    if np.any(np.diff(time_s) <= 0):
        raise ValueError("CSV times must increase from one row to the next.")

    return time_s, points_W, targets_W, phases, ground_z_W_m


def make_animation(time_s, points_W, targets_W, phases, ground_z_W_m):
    """2. Prepare the 3D graph, then animate the recorded points."""
    fig = plt.figure(figsize=(11, 8))
    fig.canvas.manager.set_window_title("Quadruped - C++ recording")
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.02, right=0.82, bottom=0.09, top=0.88)
    ax.set_xlabel("World x [m]", labelpad=10)
    ax.set_ylabel("World y [m]", labelpad=10)
    ax.set_zlabel("World z [m]", labelpad=10)
    ax.view_init(elev=25, azim=-55)
    ax.set_proj_type("ortho")

    # Keep the entire recording in view, without changing the axes each frame.
    all_points = np.concatenate((points_W.reshape(-1, 3), targets_W.reshape(-1, 3)))
    lower = all_points.min(axis=0) - 0.04
    upper = all_points.max(axis=0) + 0.04
    lower[2] = min(lower[2], ground_z_W_m - 0.02)
    ax.set_xlim(lower[0], upper[0])
    ax.set_ylim(lower[1], upper[1])
    ax.set_zlim(lower[2], upper[2])
    ax.set_box_aspect(upper - lower)  # One metre has the same scale on every axis.
    ax.zaxis.set_major_locator(MaxNLocator(nbins=4))

    # Flat ground at the height recorded by C++.
    ground_x, ground_y = np.meshgrid([lower[0], upper[0]], [lower[1], upper[1]])
    ax.plot_surface(ground_x, ground_y, np.full_like(ground_x, ground_z_W_m),
                    color="#b9c7ce", alpha=0.18, shade=False)

    body_line, = ax.plot([], [], [], color="#273747", linewidth=2.5, label="Body outline")
    front_line, = ax.plot([], [], [], color="#273747", linewidth=5, label="Front edge")

    leg_lines = []
    for name, colour in zip(LEG_NAMES, LEG_COLOURS):
        line, = ax.plot([], [], [], "o-", color=colour, linewidth=2.5,
                        markersize=4, label=name)
        leg_lines.append(line)

    target_marks, = ax.plot([], [], [], linestyle="None", marker="x",
                            color="#202020", markersize=7, label="Foot-bottom targets")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 0.92), frameon=False)
    title = fig.suptitle("", fontsize=14, y=0.96)
    phase_text = fig.text(0.5, 0.91, "", ha="center", fontsize=11)
    fig.text(0.5, 0.045, "Drag to rotate | Space: pause / resume | Recording repeats",
             ha="center", fontsize=10)
    fig.text(0.5, 0.02, "H-P-K-F lines end at the ball centres; crosses mark the ball-bottom targets.",
             ha="center", fontsize=9, color="#52616b")

    def draw_frame(frame):
        # Each leg is H -> P -> K -> F, using only the C++ world coordinates.
        for leg, line in enumerate(leg_lines):
            xyz = points_W[frame, leg]
            line.set_data_3d(xyz[:, 0], xyz[:, 1], xyz[:, 2])

        # Join the hip-roll origins around the body: FL -> FR -> RR -> RL -> FL.
        hips_W = points_W[frame, :, 0, :]
        outline = hips_W[[0, 1, 3, 2, 0]]
        body_line.set_data_3d(outline[:, 0], outline[:, 1], outline[:, 2])
        front_line.set_data_3d(hips_W[:2, 0], hips_W[:2, 1], hips_W[:2, 2])

        target = targets_W[frame]
        target_marks.set_data_3d(target[:, 0], target[:, 1], target[:, 2])
        title.set_text(f"Quadruped | t = {time_s[frame]:.2f} s | "
                       f"{'Paused' if paused else 'Playing'}")
        phase_text.set_text("     ".join(
            f"{name}: {phases[frame, leg].lower()}" for leg, name in enumerate(LEG_NAMES)
        ))

    # 3. Follow the CSV timestamps; rendering does not advance the robot model.
    duration_s = time_s[-1] - time_s[0]
    start_clock = perf_counter()
    pause_clock = 0.0
    paused = False
    current_frame = 0

    def initialise_animation():
        nonlocal start_clock
        start_clock = perf_counter()
        draw_frame(0)

    def update_animation(_tick):
        nonlocal current_frame
        # Hold the final pose for one second, then replay from the beginning.
        elapsed_s = (perf_counter() - start_clock) % (duration_s + 1.0)
        recording_time_s = time_s[0] + min(elapsed_s, duration_s)
        current_frame = max(0, int(np.searchsorted(time_s, recording_time_s, side="right") - 1))
        draw_frame(current_frame)

    animation = FuncAnimation(fig, update_animation, init_func=initialise_animation,
                              interval=25, blit=False, cache_frame_data=False)

    def on_key(event):
        nonlocal paused, pause_clock, start_clock
        if event.key != " ":
            return
        if paused:
            start_clock += perf_counter() - pause_clock  # Exclude the paused duration.
            animation.resume()
        else:
            pause_clock = perf_counter()
            animation.pause()
        paused = not paused
        draw_frame(current_frame)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", on_key)
    return fig, animation


def main():
    try:
        recording = load_recording(CSV_PATH)
    except (OSError, ValueError, KeyError) as error:
        print(f"Could not load {CSV_PATH}\n{error}")
        return 1

    time_s = recording[0]
    print(f"Loaded {len(time_s)} states from {CSV_PATH}")
    print("Space pauses/resumes. Close the graph window to finish.")
    fig, animation = make_animation(*recording)  # Keep animation alive while the window is open.
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
