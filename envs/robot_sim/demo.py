"""Run a headless scripted drive or a keyboard-controlled Matplotlib playground."""
import argparse
import json
import math
from pathlib import Path
from .environment import RobotEnvironment,replay


class KeyboardController:
    def __init__(self,env):
        self.env=env
        self.servo=0.0
        self.paused=False
        self.closed=False

    def press(self,event):
        key=(event.key or '').lower()
        commands={'w':(45,45),'s':(-45,-45),'a':(-35,35),'d':(35,-35)}
        if key in commands:
            self.env.robot.set_motor_pwm(*commands[key])
        elif key==' ':
            self.env.robot.stop()
        elif key=='p':
            self.paused=not self.paused
        elif key in ('[',']') and 'ultrasonic' in self.env.robot.sensor_names:
            self.servo=max(-math.pi/2,min(math.pi/2,self.servo+(-.15 if key=='[' else .15)))
            self.env.robot.set_servo('ultrasonic',self.servo)
        elif key=='escape':
            self.closed=True
            self.env.close()

    def release(self,event):
        if (event.key or '').lower() in ('w','s','a','d'):
            self.env.robot.stop()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',default='maps/01_empty_room.json')
    parser.add_argument('--config',help='JSON containing sensors, robot, physics_dt_s, seed')
    parser.add_argument('--pose',nargs=3,type=float,metavar=('X','Y','YAW'))
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--steps',type=int,default=400,help='headless 10 ms steps')
    parser.add_argument('--interactive',action='store_true')
    parser.add_argument('--save',help='save a PNG at the final state')
    parser.add_argument('--record',help='write a reproducible JSONL command/observation log')
    parser.add_argument('--replay',help='re-simulate and verify an existing JSONL log')
    args=parser.parse_args()
    if args.steps<0:
        parser.error('--steps must be nonnegative')
    if not args.interactive:
        import matplotlib
        matplotlib.use('Agg')
    if args.replay:
        env=replay(args.replay)
        if args.save: env.render(save_path=args.save)
        print(json.dumps(env.ground_truth(),indent=2))
        env.close(); return
    options=json.loads(Path(args.config).read_text()) if args.config else {}
    options['seed']=args.seed
    env=RobotEnvironment(args.map,**options)
    env.reset(pose=args.pose,random_spawn=args.pose is None)
    if args.record: env.start_recording(args.record)
    if args.interactive:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        figure=env.render()
        keys=KeyboardController(env)
        figure.canvas.mpl_connect('key_press_event',keys.press)
        figure.canvas.mpl_connect('key_release_event',keys.release)
        figure.canvas.mpl_connect('close_event',lambda event: env.close())
        print('W/S forward/back | A/D turn | SPACE stop | [/] servo | P pause | ESC close')
        def update(_):
            if keys.closed: return
            if not keys.paused: env.step(5*env.dt)
            env.render()
        animation=FuncAnimation(figure,update,interval=50,cache_frame_data=False)
        # Keep a reference for the life of the window.
        figure._robot_animation=animation
        plt.show()
    else:
        for k in range(args.steps):
            # A simple scripted test, deliberately not a navigation algorithm.
            left,right=(40,40) if (k//100)%2==0 else (-30,30)
            env.robot.set_motor_pwm(left,right)
            env.step()
    if args.save: env.render(save_path=args.save)
    print(json.dumps(env.ground_truth(),indent=2))
    env.close()


if __name__=='__main__':
    main()
