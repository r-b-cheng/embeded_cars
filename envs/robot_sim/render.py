"""Optional Matplotlib viewer; independent of simulation time and noise state."""
import math
import numpy as np


def render_environment(env, figure=None, save_path=None, show=False, estimated_pose=None):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle
    if figure is None:
        figure,ax=plt.subplots(figsize=(8,7))
    else:
        ax=figure.axes[0]
        ax.clear()
    world=env._world
    x0,y0,x1,y1=world.bounds
    ax.plot([x0,x1,x1,x0,x0],[y0,y0,y1,y1,y0],linewidth=2,label="Boundary")
    for p in world.polygons:
        ax.add_patch(Polygon(p,closed=True,alpha=0.35))
    for e in world.walls:
        ax.plot(e[:,0],e[:,1],linewidth=2)
    trajectory=np.asarray(env._trajectory)
    ax.plot(trajectory[:,0],trajectory[:,1],linewidth=1,label="Ground truth trail")
    x,y,theta=env._state.pose
    r=env.config.radius_m
    ax.add_patch(Circle((x,y),r,fill=False,linewidth=2))
    ax.arrow(x,y,r*math.cos(theta),r*math.sin(theta),head_width=r/2,length_includes_head=True)
    # Show sensor geometry at CURRENT state, not stale/latency-delayed measurements.
    # It is deliberately evaluator-only and labeled as such.
    for sensor in env._sensors.values():
        c=sensor.config
        if c["type"] not in ("ultrasonic","infrared","lidar"):
            continue
        ox,oy,phi=c["mount"]
        origin=(x+math.cos(theta)*ox-math.sin(theta)*oy,y+math.sin(theta)*ox+math.cos(theta)*oy)
        angles=c["angles_rad"] if c["type"]=="lidar" else [sensor.servo_angle]
        for a in angles:
            bearing=theta+phi+a
            distance=world.range(origin,bearing,c["max_range_m"])
            if not math.isfinite(distance):
                distance=c["max_range_m"]
            ax.plot([origin[0],origin[0]+distance*math.cos(bearing)],
                    [origin[1],origin[1]+distance*math.sin(bearing)],linestyle=":",alpha=0.35)
        if c["type"]=="ultrasonic":
            b=theta+phi+sensor.servo_angle
            for a in (b-math.radians(c["beam_width_deg"])/2,b+math.radians(c["beam_width_deg"])/2):
                d=min(c["max_range_m"],0.5)
                ax.plot([origin[0],origin[0]+d*math.cos(a)],[origin[1],origin[1]+d*math.sin(a)],linestyle="--",alpha=0.6)
    if estimated_pose is not None:
        ex,ey,et=estimated_pose
        ax.scatter([ex],[ey],marker="x",label="Algorithm estimate")
        ax.arrow(ex,ey,r*math.cos(et),r*math.sin(et),head_width=r/2)
    ax.set(xlim=(x0-0.15,x1+0.15),ylim=(y0-0.15,y1+0.15),xlabel="x (m)",ylabel="y (m)",
           title=f"{world.name} | t={env.time_s:.2f}s | evaluator geometry view")
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.legend(loc="upper right")
    figure.tight_layout()
    if save_path is not None:
        figure.savefig(save_path,dpi=140)
    if show:
        plt.show(block=False)
        plt.pause(0.001)
    return figure
