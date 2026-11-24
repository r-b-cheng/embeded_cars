import copy
import math
import numpy as np
import pytest
from robot_sim import RobotEnvironment,RobotConfig


def make(room,sensors,robot=None,pose=(2,2,0),**kwargs):
    e=RobotEnvironment(room,sensors=sensors,robot=robot,**kwargs)
    e.reset(pose)
    return e


def test_initial_sensor_values(room):
    e=make(room,None)
    obs=e.robot.get_observations()
    assert obs['ultrasonic']['data']['distance_m']==pytest.approx(3)
    assert obs['imu']['data']['angular_velocity_rad_s']==[0,0,0]
    assert obs['imu']['data']['specific_force_m_s2']==[0,0,9.80665]
    assert obs['encoder']['data']==dict(left_ticks=0,right_ticks=0,left_delta=0,right_delta=0)
    assert obs['bumper']['data']=={'contact':False}
    assert all(v['timestamp_s']==0 for v in obs.values())


def test_cache_does_not_resample_and_is_defensive_copy(room):
    e=make(room,{'imu':{'gyro_noise_std_rad_s':.1}})
    first=e.robot.read_sensor('imu')
    assert e.robot.read_sensor('imu')==first
    changed=copy.deepcopy(first)
    changed['data']['angular_velocity_rad_s'][2]=999
    returned=e.robot.read_sensor('imu'); returned['data'].clear()
    assert e.robot.read_sensor('imu')==first
    e.step()
    assert e.robot.read_sensor('imu')['sequence']==1
    assert e.robot.read_sensor('imu')!=first


def test_latency_and_sample_clock(room):
    e=make(room,{'ultrasonic':{'latency_s':.03}})
    assert e.robot.read_sensor('ultrasonic') is None
    e.step(.02); assert e.robot.read_sensor('ultrasonic') is None
    e.step(.01); f=e.robot.read_sensor('ultrasonic')
    assert f['timestamp_s']==0 and f['available_at_s']==.03
    e.step(.07); assert e.robot.read_sensor('ultrasonic')['sequence']==0
    e.step(.03); f=e.robot.read_sensor('ultrasonic')
    assert f['timestamp_s']==.1 and f['sequence']==1
    assert f['available_at_s']==.13


def test_gyro_specific_force_encoder_curve(room):
    offset=(.03,-.02,.4)
    e=make(room,{'imu':{'mount':offset},'encoder':{'ticks_per_rev':1000,'frequency_hz':100}})
    e.robot.set_motor_pwm(20,60); e.step()
    v=.12; w=.12/.135; ax=v/.01; alpha=w/.01
    x,y,phi=offset
    bx=ax-alpha*y-w*w*x; by=v*w+alpha*x-w*w*y
    im=e.robot.read_sensor('imu')['data']
    np.testing.assert_allclose(im['specific_force_m_s2'],[math.cos(phi)*bx+math.sin(phi)*by,-math.sin(phi)*bx+math.cos(phi)*by,9.80665],atol=1e-12)
    assert im['angular_velocity_rad_s'][2]==pytest.approx(w)
    e.step(.19)
    enc=e.robot.read_sensor('encoder')['data']
    assert enc['left_ticks']==round(.06*.2/(2*math.pi*.0325)*1000)
    assert enc['right_ticks']==round(.18*.2/(2*math.pi*.0325)*1000)
    e.robot.set_motor_pwm(-20,-60); e.step(.2)
    enc=e.robot.read_sensor('encoder')['data']
    assert enc['left_ticks']==0 and enc['right_ticks']==0


def test_encoder_reports_slip_not_body_displacement(room):
    e=make(room,{'encoder':{}},robot={'left_slip':1,'right_slip':1})
    before=e.ground_truth()['pose']
    e.robot.set_motor_pwm(100,100); e.step(1)
    assert e.ground_truth()['pose']==before
    assert e.robot.read_sensor('encoder')['data']['left_ticks']>0


def test_servo_slew_and_settling(room):
    e=make(room,{'ultrasonic':{}})
    e.robot.set_servo('ultrasonic',math.pi/2)
    e.step(.2); assert e.robot.read_sensor('ultrasonic')['data']['status']=='servo_unsettled'
    e.step(.3); assert e.robot.read_sensor('ultrasonic')['data']['status']=='servo_unsettled'
    e.step(.1); frame=e.robot.read_sensor('ultrasonic')['data']
    assert frame['status']=='hit'
    assert frame['distance_m']==pytest.approx(3)
    assert frame['bearing_rad']==pytest.approx(math.pi/2)
    with pytest.raises(ValueError): e.robot.set_servo('ultrasonic',4)


def test_range_limits_and_no_false_max_range_hit(room):
    e=make(room,{'far':{'type':'ultrasonic','max_range_m':1},
                 'blind':{'type':'infrared','min_range_m':.2,'max_range_m':1}},pose=(4.85,2,0))
    assert e.robot.read_sensor('blind')['data']['status']=='below_min'
    assert e.robot.read_sensor('blind')['data']['distance_m'] is None
    e.reset((2,2,0))
    assert e.robot.read_sensor('far')['data']['status']=='no_echo'
    assert e.robot.read_sensor('far')['data']['distance_m'] is None


def test_mounted_sensor_rotation_and_lidar(room):
    e=make(room,{'left':{'type':'infrared','mount':[.08,0,math.pi/2],'max_range_m':10},
                 'lidar':{'angles_rad':[0,math.pi/2,math.pi,-math.pi/2]}},pose=(1,2,math.pi/2))
    assert e.robot.read_sensor('left')['data']['distance_m']==pytest.approx(1)
    np.testing.assert_allclose(e.robot.read_sensor('lidar')['data']['ranges_m'],[3,1,2,4],atol=1e-12)


def test_enabling_other_sensor_does_not_change_existing_noise(room):
    spec={'imu':{'gyro_noise_std_rad_s':.05,'gyro_bias_walk_rad_s_sqrt_s':.003}}
    a=make(room,spec,seed=12)
    b=make(room,{**spec,'ultrasonic':{'noise_std_m':.1},'lidar':{}},seed=12)
    for _ in range(30):
        a.step(.02); b.step(.02)
        assert a.robot.read_sensor('imu')==b.robot.read_sensor('imu')


def test_noise_statistics_and_seed(room):
    e=make(room,{'imu':{'gyro_noise_std_rad_s':.02,'gyro_bias_rad_s':.04}},seed=123)
    values=[]
    for _ in range(4000):
        e.step(); values.append(e.robot.read_sensor('imu')['data']['angular_velocity_rad_s'][2])
    assert abs(np.mean(values)-.04)<.0015
    assert .019<np.std(values)<.021
    e.reset((2,2,0))
    again=[]
    for _ in range(10):
        e.step(); again.append(e.robot.read_sensor('imu')['data']['angular_velocity_rad_s'][2])
    assert again==values[:10]


def test_range_noise_and_dropout_statistics(room):
    e=make(room,{'ultrasonic':{'noise_std_m':.01,'dropout_probability':.2}},seed=33)
    distances=[]; drops=0
    for _ in range(1200):
        e.step(.1)
        d=e.robot.read_sensor('ultrasonic')['data']
        if d['status']=='dropout': drops+=1
        else: distances.append(d['distance_m'])
    assert .15<drops/1200<.25
    assert abs(np.mean(distances)-3)<.002
    assert .008<np.std(distances)<.012


@pytest.mark.parametrize('spec',[
    {'a':{'type':'ultrasonic','max_range_m':0}},
    {'imu':{'frequency_hz':30}},
    {'imu':{'frequency_hz':200}},
    {'imu':{'latency_s':.015}},
    {'ultrasonic':{'beam_width_deg':180}},
    {'ultrasonic':{'noise_std_m':-.1}},
    {'ultrasonic':{'dropout_probability':1.1}},
    {'ultrasonic':{'mount':[.5,0,0]}},
    {'ultrasonic':{'servo_speed_rad_s':0}},
    {'imu':{'fake_field':1}},
    {'imu':{'gyro_noise_std_rad_s':float('nan')}},
    {'lidar':{'angles_rad':[]}},
    {'encoder':{'ticks_per_rev':2.5}},
    {'encoder':{'mount':[.01,0,0]}},
    {'camera':{}},
])
def test_invalid_configuration(room,spec):
    with pytest.raises((ValueError,TypeError)):
        RobotEnvironment(room,sensors=spec)


def test_disabled_and_unknown_sensor(room):
    e=make(room,{'imu':{},'lidar':{'enabled':False}})
    assert e.robot.sensor_names==('imu',)
    with pytest.raises(KeyError): e.robot.read_sensor('lidar')
    with pytest.raises(ValueError): e.robot.set_servo('imu',0)
    e=make(room,{})
    assert e.robot.get_observations()=={}
