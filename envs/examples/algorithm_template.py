"""A controller depends ONLY on RobotPort. The experiment runner owns the world."""
from robot_sim import RobotEnvironment,RobotPort


class ReactiveController:
    """Minimal obstacle-stop example, not SLAM or guaranteed collision avoidance."""
    def update(self,robot:RobotPort):
        observation=robot.read_sensor('ultrasonic')
        if observation is None:
            robot.stop(); return
        data=observation['data']
        if robot.time_s-observation['timestamp_s']>.25 or data['status']!='hit':
            robot.stop(); return
        robot.set_motor_pwm(35,35) if data['distance_m']>.5 else robot.stop()


def main():
    controller=ReactiveController()
    with RobotEnvironment('maps/02_central_box.json',sensors={'ultrasonic':{}}) as env:
        env.reset((.6,.7,0))
        for _ in range(300):
            controller.update(env.robot)
            env.step()
        env.render(save_path='controller_example.png')
        # Ground truth is read only by the evaluator, outside the controller.
        print(env.ground_truth())


if __name__=='__main__':
    main()
