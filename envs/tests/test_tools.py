import json
import math
from types import SimpleNamespace
import numpy as np
import pytest
from PIL import Image
from robot_sim import RobotEnvironment,WorldMap,replay
from robot_sim.convert import convert_grid
from robot_sim.demo import KeyboardController


def test_image_conversion_flips_rows_and_preserves_unknown(tmp_path):
    image=np.array([[0,255],[255,205]],dtype=np.uint8)
    p=tmp_path/'source.png'; Image.fromarray(image).save(p)
    output=tmp_path/'out.json'
    with pytest.raises(ValueError): convert_grid(p,output,resolution_m=.5)
    w=convert_grid(p,output,resolution_m=.5,unknown_policy='occupied')
    assert w.to_dict()['occupancy']['cells']==[[0,-1],[100,0]]
    assert w.is_free((.25,.25))
    assert not w.is_free((.25,.75))
    assert not w.is_free((.75,.25))


@pytest.mark.parametrize('suffix',['.npy','.csv'])
def test_array_conversion(tmp_path,suffix):
    a=np.array([[0,0,100],[0,0,0]])
    path=tmp_path/('input'+suffix)
    if suffix=='.npy': np.save(path,a)
    else: np.savetxt(path,a,delimiter=',',fmt='%d')
    w=convert_grid(path,tmp_path/'map.json',resolution_m=.2,top_to_bottom=True)
    assert w.to_dict()['occupancy']['cells']==[[0,0,0],[0,0,100]]
    assert not w.is_free((.5,.3))


def test_keyboard_callbacks(room):
    e=RobotEnvironment(room); e.reset((2,2,0)); k=KeyboardController(e)
    k.press(SimpleNamespace(key='w')); e.step(.1)
    assert e.ground_truth()['pose'][0]>2
    k.release(SimpleNamespace(key='w')); e.step()
    assert e.ground_truth()['v_m_s']==0
    k.press(SimpleNamespace(key=']')); e.step(.2)
    assert e.robot.read_sensor('ultrasonic')['data']['bearing_rad']==pytest.approx(.15)
    k.press(SimpleNamespace(key='p')); assert k.paused
    k.press(SimpleNamespace(key='p')); assert not k.paused
    k.press(SimpleNamespace(key=' ')); k.press(SimpleNamespace(key='escape')); assert k.closed


def test_replay_detects_output_tampering(tmp_path,room):
    p=tmp_path/'record.jsonl'
    e=RobotEnvironment(room,sensors={}); e.reset((2,2,0)); e.start_recording(p)
    e.robot.set_motor_pwm(20,20); e.step(.1); e.stop_recording()
    lines=p.read_text().splitlines(); obj=json.loads(lines[-1]); obj['truth']['pose'][0]+=1
    lines[-1]=json.dumps(obj); p.write_text('\n'.join(lines)+'\n')
    with pytest.raises(AssertionError): replay(p)


@pytest.mark.parametrize('key,value',[('radius_m',0),('wheel_base_m',-1),('wheel_radius_m',0),
                                     ('motor_tau_s',-1),('left_slip',1.1),('right_gain',0),
                                     ('max_wheel_speed_m_s',float('inf'))])
def test_bad_robot_settings(room,key,value):
    with pytest.raises(ValueError): RobotEnvironment(room,robot={key:value})


def test_same_seed_reset_changes_nothing(room):
    config={'imu':{'gyro_noise_std_rad_s':.03},'ultrasonic':{'noise_std_m':.01}}
    e=RobotEnvironment(room,sensors=config,seed=8)
    def run():
        e.reset((2,2,.2)); e.robot.set_motor_pwm(40,45); return e.step(.7),e.ground_truth()
    a=run(); b=run(); assert a==b


def test_close_and_context_manager(tmp_path,room):
    path=tmp_path/'log.jsonl'
    with RobotEnvironment(room,sensors={}) as e:
        e.reset((2,2,0)); e.start_recording(path); e.step()
    assert e._record is None
    assert replay(path).time_s==.01

@pytest.mark.parametrize('encoding,a,expected',[
    ('binary',[[0,1],[1,0]],[[0,100],[100,0]]),
    ('probability',[[0,.8],[.25,-1]],[[0,80],[25,-1]]),
])
def test_conversion_encodings(tmp_path,encoding,a,expected):
    p=tmp_path/'input.npy'; np.save(p,a)
    w=convert_grid(p,tmp_path/'map.json',resolution_m=.1,encoding=encoding,unknown_policy='free')
    assert w.to_dict()['occupancy']['cells']==expected


def test_bumper_latches_between_samples():
    e=RobotEnvironment({'version':1,'bounds':[0,0,2,2]},sensors={'bumper':{'frequency_hz':10}},robot={'max_wheel_speed_m_s':2})
    e.reset((1.89,1,0)); e.robot.set_motor_pwm(100,100); e.step(.01)
    assert e.ground_truth()['collided']
    e.robot.set_motor_pwm(-30,-30); e.step(.09)
    assert not e.ground_truth()['collided']
    assert e.robot.read_sensor('bumper')['data']['contact']
    e.step(.1); assert not e.robot.read_sensor('bumper')['data']['contact']


def test_truncated_replay_rejected(tmp_path,room):
    p=tmp_path/'truncated.jsonl'
    e=RobotEnvironment(room,sensors={}); e.reset((1,1,0)); e.start_recording(p); e.step(.1); e.stop_recording()
    lines=p.read_text().splitlines(); p.write_text('\n'.join(lines[:-1])+'\n')
    with pytest.raises(ValueError,match='truncated'): replay(p)


def test_demo_cli_and_replay(monkeypatch,tmp_path,room):
    from robot_sim.demo import main
    m=tmp_path/'room.json'; room.save(m)
    log=tmp_path/'demo.jsonl'; pic=tmp_path/'demo.png'
    monkeypatch.setattr('sys.argv',['demo','--map',str(m),'--pose','1','1','0','--steps','5','--record',str(log),'--save',str(pic)])
    main(); assert pic.exists()
    monkeypatch.setattr('sys.argv',['demo','--replay',str(log),'--save',str(pic)])
    main(); assert replay(log).time_s==.05


def test_converter_cli(monkeypatch,tmp_path):
    from robot_sim.convert import main
    source=tmp_path/'raw.npy'; np.save(source,[[0,1],[1,0]])
    dest=tmp_path/'map.json'
    monkeypatch.setattr('sys.argv',['convert',str(source),str(dest),'--resolution','.2','--encoding','binary'])
    main()
    assert WorldMap.load(dest).to_dict()['occupancy']['cells']==[[0,100],[100,0]]
