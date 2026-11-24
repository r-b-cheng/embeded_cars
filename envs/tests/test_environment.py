import math
import numpy as np
import pytest
from robot_sim import RobotEnvironment,WorldMap,replay,pose_errors,occupancy_iou


def test_commands_drive_without_teleport(room):
    e=RobotEnvironment(room,sensors={}); e.reset((1,1,0))
    assert not hasattr(e.robot,'ground_truth') and not hasattr(e.robot,'load_map')
    e.robot.set_motor_pwm(50,50)
    assert e.ground_truth()['pose']==[1,1,0]
    e.step(1)
    np.testing.assert_allclose(e.ground_truth()['pose'],[1.15,1,0],atol=1e-12)
    e.robot.stop(); e.step(1)
    np.testing.assert_allclose(e.ground_truth()['pose'],[1.15,1,0],atol=1e-12)


def test_turn_in_place(room):
    e=RobotEnvironment(room); e.reset((2,2,0)); e.robot.set_motor_pwm(-40,40); e.step(.3)
    p=e.ground_truth()['pose']
    np.testing.assert_allclose(p,[2,2,.24/.135*.3],atol=1e-12)


def test_first_order_motor_lag(room):
    e=RobotEnvironment(room,sensors={},robot={'motor_tau_s':.2})
    e.reset((2,2,0)); e.robot.set_motor_pwm(100,100); e.step(.1)
    v=.3*(1-math.exp(-.1/.2))
    assert e.ground_truth()['wheel_speeds_m_s']==pytest.approx([v,v],abs=1e-12)
    expected_dx=sum(.3*(1-math.exp(-k*.01/.2))*.01 for k in range(1,11))
    assert e.ground_truth()['pose'][0]==pytest.approx(2+expected_dx,abs=1e-12)


def test_thin_wall_continuous_collision_and_encoder_spin():
    w=WorldMap({'version':1,'bounds':[0,0,5,5],'walls':[[[2,0],[2,5]]]})
    e=RobotEnvironment(w,robot={'max_wheel_speed_m_s':200},sensors={'encoder':{},'bumper':{}})
    e.reset((1,2,0)); e.robot.set_motor_pwm(100,100); e.step(.02)
    assert e.ground_truth()['pose']==[1,2,0]
    assert e.ground_truth()['collided']
    assert e.robot.read_sensor('bumper')['data']['contact']
    assert e.robot.read_sensor('encoder')['data']['left_ticks']>0


def test_swept_arc_not_only_endpoints():
    # A one-tick half-circle has clear endpoints but crosses a horizontal thin wall.
    w=WorldMap({'version':1,'bounds':[0,0,5,5],'walls':[[[2.2,2.5],[3,2.5]]]})
    omega=math.pi/.01; v=.5*omega
    maxspeed=v+omega*.135/2
    left=(v-omega*.135/2)/maxspeed*100
    e=RobotEnvironment(w,robot={'max_wheel_speed_m_s':maxspeed},sensors={})
    e.reset((2,2,0)); e.robot.set_motor_pwm(left,100); e.step()
    assert e.ground_truth()['collided']
    assert e.ground_truth()['pose']==[2,2,0]


def test_step_chunking_is_identical(room):
    a=RobotEnvironment(room,seed=99); b=RobotEnvironment(room,seed=99)
    a.reset((2,2,.1)); b.reset((2,2,.1))
    a.robot.set_motor_pwm(30,60); b.robot.set_motor_pwm(30,60)
    a.step(.5)
    for _ in range(50): b.step()
    assert a.ground_truth()==b.ground_truth()
    assert a.robot.get_observations()==b.robot.get_observations()


def test_replay_noisy_commands_and_servo(tmp_path,room):
    e=RobotEnvironment(room,sensors={'ultrasonic':{'noise_std_m':.01,'dropout_probability':.1,'latency_s':.02},
                                    'imu':{'gyro_noise_std_rad_s':.01,'gyro_bias_walk_rad_s_sqrt_s':.002},'encoder':{}},seed=51)
    e.reset((2,2,0)); path=tmp_path/'episode.jsonl'; e.start_recording(path)
    for i in range(20):
        e.robot.set_motor_pwm(40-i,30+i)
        e.robot.set_servo('ultrasonic',(-1)**i*.2)
        e.step(.2)
    e.stop_recording(); b=replay(path)
    assert b.ground_truth()==e.ground_truth()
    assert b.robot.get_observations()==e.robot.get_observations()
    text=path.read_text(); path.write_text(text.replace('"map_sha256": "','"map_sha256": "x',1))
    with pytest.raises(ValueError): replay(path)


def test_random_spawn_and_invalid_placement(room):
    e=RobotEnvironment(room,sensors={},seed=4)
    e.reset(random_spawn=True); p=e.ground_truth()['pose']
    e.reset(random_spawn=True); assert e.ground_truth()['pose']==p
    e.reset(random_spawn=True,seed=5); assert e.ground_truth()['pose']!=p
    assert room.is_free(e.ground_truth()['pose'][:2],.1)
    for p in [(0,0,0),(6,3,0),(.05,1,0),(1,1,float('nan'))]:
        with pytest.raises(ValueError): e.reset(p)
    with pytest.raises(ValueError): e.reset((1,1,0),random_spawn=True)


def test_lifecycle_validation(tmp_path,room):
    e=RobotEnvironment(sensors={})
    with pytest.raises(RuntimeError): e.step()
    with pytest.raises(RuntimeError): e.reset((1,1,0))
    e.load_map(room)
    with pytest.raises(ValueError): e.reset()
    e.reset((1,1,0))
    for value in [101,-101,float('nan'),float('inf')]:
        with pytest.raises(ValueError): e.robot.set_motor_pwm(value,0)
    for value in [.015,-.1,float('nan')]:
        with pytest.raises(ValueError): e.step(value)
    e.start_recording(tmp_path/'log.jsonl')
    with pytest.raises(RuntimeError): e.reset((1,1,0))
    with pytest.raises(RuntimeError): e.load_map(room)
    e.close(); e.load_map(room)
    with pytest.raises(RuntimeError): e.robot.read_sensor('imu')


def test_json_grid_roundtrip(tmp_path):
    cells=np.zeros((10,12),int); cells[5,7]=100
    w=WorldMap.from_occupancy(cells,.2,origin_m=(-1,-2))
    path=tmp_path/'map.json'; w.save(path); loaded=WorldMap.load(path)
    assert loaded.to_dict()==w.to_dict() and loaded.sha256==w.sha256
    assert not loaded.is_free((.5,-.9))
    assert loaded.range((.1,-.9),0,1)==pytest.approx(.3)
    cells[1,1]=-1
    with pytest.raises(ValueError): WorldMap.from_occupancy(cells,.2)
    blocked=WorldMap.from_occupancy(cells,.2,unknown_policy='occupied')
    assert not blocked.is_free((.3,.3))
    free=WorldMap.from_occupancy(cells,.2,unknown_policy='free')
    assert free.is_free((.3,.3))
    assert free.to_dict()['occupancy']['cells'][1][1]==-1


def test_metrics():
    m=pose_errors([[0,0,math.pi-.01]],[[.03,.04,-math.pi+.01]])
    assert m['position_rmse_m']==pytest.approx(.05)
    assert m['yaw_rmse_rad']==pytest.approx(.02)
    assert occupancy_iou([[100,0]],[[100,100]])['iou']==.5
    assert occupancy_iou([[-1]],[[-1]])['iou'] is None
    with pytest.raises(ValueError): pose_errors([],[])


def test_render_is_side_effect_free(tmp_path,room):
    import matplotlib
    matplotlib.use('Agg')
    e=RobotEnvironment(room); e.reset((2,2,0)); before=e.robot.get_observations()
    path=tmp_path/'preview.png'
    e.render(save_path=path,estimated_pose=(2.1,2,.1))
    assert path.stat().st_size>1000
    assert e.robot.get_observations()==before
    assert e.time_s==0
    e.close()
