import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib import animation
from DynamicObstacle import *
from StaticObstacle import *

# -------------------------------
# Example data (replace with yours)
# -------------------------------

def show_gif(PBRRT_inst, gif_name):
    

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

    # ✅ Start/goal (works even if you don't store a "goal" in params)
    start_xy = PBRRT_inst.PBRRT_params["start"].pos
    goal_xy  = PBRRT_inst.PBRRT_params["goal"].pos

    # -------------------------------
    # Set up figure/axis
    # -------------------------------
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_aspect('equal')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    #ax.set_title("Map 1 - PBRRT (γ = 0.8)")

    # -------------------------------
    # Static obstacles (your existing code)
    # -------------------------------
    static_patches = []
    static_rects = [(obs.x_min, obs.y_min, obs.width, obs.height) for obs in StaticObstacle.all]
    for (x_min, y_min, w, h) in static_rects:
        rect = patches.Rectangle((x_min, y_min), w, h, fill=True, alpha=0.5, color="gray")
        ax.add_patch(rect)
        static_patches.append(rect)

    # -------------------------------
    # ✅ Draw start + goal (static)
    # -------------------------------
    start_artist, = ax.plot(start_xy[0], start_xy[1], marker='*', markersize=20, linestyle='None', color = 'red')
    goal_artist,  = ax.plot(goal_xy[0],  goal_xy[1],  marker='X', markersize=20, linestyle='None', color = 'purple')

    # Optional text labels
    #start_text = ax.text(start_xy[0], start_xy[1], " start", va="bottom", ha="left")
    #goal_text  = ax.text(goal_xy[0],  goal_xy[1],  " goal",  va="bottom", ha="left")

    # -------------------------------
    # Animation artists (your existing code)
    # -------------------------------
    (robot_point,) = ax.plot([], [], marker='o', markersize=6)
    (robot_trail_line,) = ax.plot([], [], linewidth=1.5)

    dyn_trajs = [np.array(obs.locations_list[0:T]) for obs in DynamicObstacle.all]
    dyn_radii = [obs.r for obs in DynamicObstacle.all]
    dyn_circles = []
    for radius in dyn_radii:
        circle = patches.Circle((0, 0), radius, alpha=0.6, color='green')
        ax.add_patch(circle)
        dyn_circles.append(circle)

    def init():
        robot_point.set_data([], [])
        robot_trail_line.set_data([], [])
        for circle in dyn_circles:
            circle.center = (-1000, -1000)

        # ✅ include static start/goal artists + text for blit=True
        return [robot_point, robot_trail_line,
                start_artist, goal_artist,
                *dyn_circles, *static_patches]

    def update(frame):
        x_r, y_r = robot_traj[frame]
        robot_point.set_data([x_r], [y_r])
        robot_trail_line.set_data(robot_traj[:frame+1, 0], robot_traj[:frame+1, 1])

        for circle, traj in zip(dyn_circles, dyn_trajs):
            x_o, y_o = traj[frame]
            circle.center = (x_o, y_o)

        # ✅ keep returning them so they stay drawn with blit
        return [robot_point, robot_trail_line,
                start_artist, goal_artist,
                *dyn_circles]

    anim = animation.FuncAnimation(
        fig, update, init_func=init, frames=T, interval=40, blit=True
    )
    anim.save(gif_name + ".gif", writer="pillow", fps=25)
    plt.close(fig)
    return robot_traj

