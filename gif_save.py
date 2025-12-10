import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib import animation
from DynamicObstacle import *
from StaticObstacle import *

# -------------------------------
# Example data (replace with yours)
# -------------------------------

def show_gif(PBRRT_inst):
    

    #robot_traj = np.array([n.pos for n in PBRRT_inst.path_executed])
    robot_traj = PBRRT_inst.PBRRT_params["start"].pos
      # number of time steps
    path_exec = PBRRT_inst.path_executed
    dyn_trajs = []
    for i in range(len(PBRRT_inst.path_executed)):
        k_star = path_exec[i].k_star_exec
        if k_star == 0:
            k_star = 1
        for _ in range(k_star):
            robot_traj = np.vstack((robot_traj, path_exec[i].pos))
        
    
    robot_traj = robot_traj[1:]
    T = len(robot_traj)
    print("T = ", T)
        

    dyn_trajs = [np.array(obs.pos_list[0:T]) for obs in DynamicObstacle.all]
    dyn_radii = [obs.r for obs in DynamicObstacle.all]

    # Static rectangular obstacles: (x, y, width, height)
    static_rects = []
    for obs in StaticObstacle.all:
        static_rects.append((obs.x_min, obs.y_min, obs.width, obs.height))


    # -------------------------------
    # Set up the figure and axis
    # -------------------------------
    fig, ax = plt.subplots(figsize=(6, 4))

    # Compute bounds from all trajectories & obstacles to set limits nicely
    all_x = [robot_traj[:, 0]] + [d[:, 0] for d in dyn_trajs]
    all_y = [robot_traj[:, 1]] + [d[:, 1] for d in dyn_trajs]

    for (x, y, w, h) in static_rects:
        all_x.append(np.array([x, x + w]))
        all_y.append(np.array([y, y + h]))

    all_x = np.concatenate(all_x)
    all_y = np.concatenate(all_y)

    margin = 0.5
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_aspect('equal')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Robot and Dynamic Obstacles")

    # -------------------------------
    # Draw static obstacles (rectangles)
    # -------------------------------
    static_patches = []
    for (x_min, y_min, w, h) in static_rects:
        rect = patches.Rectangle((x_min, y_min), w, h,
                                fill=True, alpha=0.5, color="gray")
        ax.add_patch(rect)
        static_patches.append(rect)

    # -------------------------------
    # Prepare artists for animation
    # -------------------------------
    # Robot: point + trail line
    (robot_point,) = ax.plot([], [], marker='o', markersize=6)
    (robot_trail_line,) = ax.plot([], [], linewidth=1.5)

    # Dynamic obstacles as circles
    dyn_circles = []
    for radius in dyn_radii:
        circle = patches.Circle((0, 0), radius, alpha=0.6)
        ax.add_patch(circle)
        dyn_circles.append(circle)

    static_rects = [
    (obs.x_min, obs.y_min, obs.width, obs.height)
    for obs in StaticObstacle.all]
    # # Optional: plot full static paths of dynamic obstacles (faint lines)
    # for d in dyn_trajs:
    #     ax.plot(d[:, 0], d[:, 1], linestyle='--', linewidth=0.5, alpha=0.3)

    # -------------------------------
    # Init and update functions
    # -------------------------------
    def init():
        """Initialize animation (called once)."""
        robot_point.set_data([], [])
        robot_trail_line.set_data([], [])
        for circle in dyn_circles:
            circle.center = (-1000, -1000)  # offscreen
        return [robot_point, robot_trail_line, *dyn_circles, *static_patches]

    def update(frame):
        """Update animation for frame index."""
        # Robot
        x_r, y_r = robot_traj[frame]
        robot_point.set_data([x_r], [y_r])

        # Trail of robot (0..frame)
        robot_trail_line.set_data(robot_traj[:frame+1, 0],
                                robot_traj[:frame+1, 1])

        # Dynamic obstacles
        for circle, traj in zip(dyn_circles, dyn_trajs):
            x_o, y_o = traj[frame]
            circle.center = (x_o, y_o)


        return [robot_point, robot_trail_line, *dyn_circles]

    # -------------------------------
    # Create animation
    # -------------------------------
    anim = animation.FuncAnimation(
        fig,
        update,
        init_func=init,
        frames=T,
        interval=40,   # ms between frames (~25 fps)
        blit=True
    )

    # -------------------------------
    # Save to GIF
    # -------------------------------
    # Requires `pillow` installed: pip install pillow
    anim.save("map_1.gif", writer="pillow", fps=25)

    # OR save to MP4 (requires ffmpeg installed on your system)
    # anim.save("robot_obstacles.mp4", fps=25, extra_args=['-vcodec', 'libx264'])

    plt.close(fig)  # so it doesn't pop up if running as a script
    return robot_traj
