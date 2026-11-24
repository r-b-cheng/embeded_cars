"""Same commands/map/seed with three sensor bundles; no claimed localization scores.

Replace the open-loop runner with your own controller/estimator to evaluate actual
localization quality using robot_sim.pose_errors and occupancy_iou.
"""
import json
from pathlib import Path
from robot_sim import RobotEnvironment


def main():
    folder=Path('ablation_logs'); folder.mkdir(exist_ok=True)
    variants={'sonar':{'ultrasonic':{'noise_std_m':.01}},
              'sonar_imu':{'ultrasonic':{'noise_std_m':.01},'imu':{'gyro_noise_std_rad_s':.005}},
              'sonar_encoder':{'ultrasonic':{'noise_std_m':.01},'encoder':{}}}
    for name,sensors in variants.items():
        with RobotEnvironment('maps/01_empty_room.json',sensors=sensors,seed=42) as env:
            env.reset((1,1,0)); env.start_recording(folder/(name+'.jsonl'))
            for i in range(20):
                env.robot.set_motor_pwm(*( (40,40) if i%4<3 else (-30,30) ))
                env.step(.2)
            print(name,json.dumps(env.ground_truth()))


if __name__=='__main__':
    main()
