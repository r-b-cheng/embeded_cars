"""Time-aware stop-and-scan with one servo-mounted sonar through the public API."""
import math
from robot_sim import RobotEnvironment


def main():
    with RobotEnvironment('maps/08_irregular.json',sensors={'ultrasonic':{}}) as env:
        env.reset((.5,.5,0)); bot=env.robot
        for degrees in [-60,-40,-20,0,20,40,60]:
            bot.stop(); command_time=bot.time_s
            bot.set_servo('ultrasonic',math.radians(degrees))
            for _ in range(300):
                env.step()
                frame=bot.read_sensor('ultrasonic')
                if frame is not None and frame['timestamp_s']>command_time and frame['data']['status']!='servo_unsettled':
                    print(degrees,frame['data']); break
            else:
                raise RuntimeError('scan timed out')
        env.render(save_path='stationary_scan.png')


if __name__=='__main__':
    main()
