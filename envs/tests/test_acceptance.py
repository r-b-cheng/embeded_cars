"""10 independently generated maps x 10 starts x 20 nonzero motion commands.

No sensor oracle calls production raycasting or reads sensor truth fields. Expected
poses come from a separate complex-plane integrator. Expected geometry uses GEOS.
"""
from pathlib import Path
import json
import gzip
import math
import random
import platform
import time
import numpy as np
import pytest
import shapely
from robot_sim import RobotEnvironment,WorldMap
from oracle import Oracle,motion

ROOT=Path(__file__).resolve().parents[1]
SENSORS={
    'front':{'type':'ultrasonic','mount':[.06,0,0],'beam_width_deg':15,'servo':False},
    'left':{'type':'ultrasonic','mount':[0,.04,math.pi/2],'beam_width_deg':15,'servo':False},
    'right':{'type':'ultrasonic','mount':[0,-.04,-math.pi/2],'beam_width_deg':15,'servo':False},
    'imu':{'mount':[.02,-.02,.2]},
    'encoder':{'ticks_per_rev':1440},
    'lidar':{'max_range_m':4,'angles_rad':[math.radians(a) for a in range(-160,161,40)]},
    'infrared':{'mount':[.08,0,.1]},
    'bumper':{},
}
TICKS_PER_MOTION=50
DURATION=TICKS_PER_MOTION*.01
COMMANDS=[(45,45),(35,55),(55,35),(-40,40),(40,40),(50,25),(25,50),(-40,-40),
          (40,-40),(45,45),(30,50),(50,30),(-35,-35),(-40,40),(40,40),
          (35,55),(55,35),(40,-40),(-40,-40),(45,45)]


def choose_starts(oracle,seed):
    rng=random.Random(seed); starts=[]
    x0,y0,x1,y1=oracle.bounds
    for _ in range(50000):
        p=(rng.uniform(x0+.3,x1-.3),rng.uniform(y0+.3,y1-.3),rng.uniform(-math.pi,math.pi))
        if oracle.clearance(p[:2])>.30 and all(math.dist(p[:2],q[:2])>.24 for q in starts):
            starts.append(p)
        if len(starts)==10:
            return starts
    raise AssertionError('failed to generate ten separated valid starts')


def verify_distance(data,expected,config,stats):
    stats['range_comparisons']+=1
    if math.isinf(expected):
        assert data['status']=='no_echo' and data['distance_m'] is None
        stats['no_echo_checks']+=1
    elif expected<config['min_range_m']-1e-10:
        assert data['status']=='below_min' and data['distance_m'] is None
        stats['blind_zone_checks']+=1
    else:
        assert data['status']=='hit'
        error=abs(data['distance_m']-expected)
        stats['max_range_error_m']=max(stats['max_range_error_m'],error)
        assert error<=1e-9
        stats['valid_range_checks']+=1


@pytest.mark.acceptance
def test_acceptance_campaign():
    started=time.perf_counter()
    reports=ROOT/'reports'; reports.mkdir(exist_ok=True)
    paths=sorted((ROOT/'maps').glob('*.json'))
    assert len(paths)==10
    stats={k:0 for k in ['episodes','motion_commands','physics_ticks','range_comparisons','valid_range_checks',
        'no_echo_checks','blind_zone_checks','imu_frames_checked','encoder_frames_checked','bumper_frames_checked',
        'max_range_error_m','max_pose_error_m','max_yaw_error_rad','max_gyro_error_rad_s','max_accel_error_m_s2']}
    episodes=[]; starts_output={}
    trace_path=reports/'acceptance_trace.jsonl.gz'
    with gzip.open(trace_path,'wt',encoding='utf-8') as trace:
        for map_index,path in enumerate(paths):
            spec=json.loads(path.read_text()); oracle=Oracle(spec)
            starts=choose_starts(oracle,2000+map_index)
            starts_output[path.name]=[list(p) for p in starts]
            for start_index,start in enumerate(starts):
                env=RobotEnvironment(spec,sensors=SENSORS,seed=map_index*100+start_index)
                env.reset(start)
                expected=tuple(start)
                # Normalize the initial heading independently.
                expected=(*expected[:2],math.atan2(math.sin(expected[2]),math.cos(expected[2])))
                angles=np.array([0.,0.]); old_counts=(0,0); prev_v=prev_w=0
                seen={name:0 for name in env.robot.sensor_names}
                moved=translated=0; length=0.0; adjustments=0
                for command_index,nominal in enumerate(COMMANDS):
                    choices=[nominal,(-nominal[0],-nominal[1]),(35,35),(-35,-35),(-40,40)]
                    for left,right in choices:
                        pred=[expected]
                        for _ in range(TICKS_PER_MOTION): pred.append(motion(pred[-1],left,right,.01))
                        if abs(left+right)<1e-9 or oracle.path_free(pred,.1002):
                            break
                    if (left,right)!=nominal: adjustments+=1
                    before=expected
                    env.robot.set_motor_pwm(left,right)
                    v=(left+right)/200*.3; w=(right-left)/100*.3/.135
                    for _ in range(TICKS_PER_MOTION):
                        expected=motion(expected,left,right,.01)
                        angles+=np.array([left,right])/100*.3/.0325*.01
                        ax=(v-prev_v)/.01; alpha=(w-prev_w)/.01
                        prev_v,prev_w=v,w
                        obs=env.step()
                        stats['physics_ticks']+=1
                        truth=env.ground_truth()
                        err=math.dist(truth['pose'][:2],expected[:2])
                        da=truth['pose'][2]-expected[2]
                        yawerr=abs(math.atan2(math.sin(da),math.cos(da)))
                        stats['max_pose_error_m']=max(stats['max_pose_error_m'],err)
                        stats['max_yaw_error_rad']=max(stats['max_yaw_error_rad'],yawerr)
                        assert err<1e-9 and yawerr<1e-9 and not truth['collided']
                        for name,frame in obs.items():
                            if frame['sequence']==seen[name]: continue
                            seen[name]=frame['sequence']
                            c=env.sensor_configs[name]; kind=c['type']; data=frame['data']
                            assert frame['timestamp_s']==pytest.approx(env.time_s,abs=1e-12)
                            x,y,a=expected; ox,oy,phi=c['mount']
                            origin=(x+math.cos(a)*ox-math.sin(a)*oy,y+math.sin(a)*ox+math.cos(a)*oy)
                            if kind in ('ultrasonic','infrared'):
                                d=oracle.range(origin,a+phi,c['max_range_m'],math.radians(c['beam_width_deg']))
                                verify_distance(data,d,c,stats)
                                if kind=='ultrasonic' and data['status']=='hit':
                                    assert data['echo_time_s']==pytest.approx(2*d/343,abs=1e-11)
                            elif kind=='lidar':
                                for bearing,d,status in zip(c['angles_rad'],data['ranges_m'],data['statuses']):
                                    expected_d=oracle.range(origin,a+phi+bearing,c['max_range_m'])
                                    verify_distance({'distance_m':d,'status':status},expected_d,c,stats)
                            elif kind=='imu':
                                bx=ax-alpha*oy-w*w*ox; by=v*w+alpha*ox-w*w*oy
                                target=[math.cos(phi)*bx+math.sin(phi)*by,-math.sin(phi)*bx+math.cos(phi)*by,9.80665]
                                ge=abs(data['angular_velocity_rad_s'][2]-w)
                                ae=max(abs(a-b) for a,b in zip(data['specific_force_m_s2'],target))
                                stats['max_gyro_error_rad_s']=max(stats['max_gyro_error_rad_s'],ge)
                                stats['max_accel_error_m_s2']=max(stats['max_accel_error_m_s2'],ae)
                                assert ge<1e-10 and ae<1e-9
                                stats['imu_frames_checked']+=1
                            elif kind=='encoder':
                                counts=tuple(int(math.copysign(math.floor(abs(a)*1440/(2*math.pi)+.5),a)) for a in angles)
                                assert (data['left_ticks'],data['right_ticks'])==counts
                                assert (data['left_delta'],data['right_delta'])==tuple(a-b for a,b in zip(counts,old_counts))
                                old_counts=counts
                                stats['encoder_frames_checked']+=1
                            else:
                                assert not data['contact']; stats['bumper_frames_checked']+=1
                    displacement=math.dist(before[:2],expected[:2])
                    angular=abs(math.atan2(math.sin(expected[2]-before[2]),math.cos(expected[2]-before[2])))
                    assert displacement>1e-7 or angular>1e-7
                    moved+=1; translated+=int(displacement>1e-7); length+=abs(v)*DURATION
                    trace.write(json.dumps({'map':path.name,'spawn':start_index,'motion':command_index,
                                            'pwm':[left,right],'duration_s':DURATION,'expected_pose':expected,
                                            'truth':env.ground_truth(),'observations':env.robot.get_observations()},allow_nan=False)+'\n')
                assert moved==20 and translated>=15
                episodes.append({'map':path.name,'map_sha256':WorldMap(spec).sha256,'spawn':start_index,
                    'initial_pose':list(start),'motions':moved,'translation_segments':translated,
                    'travel_m':length,'collision_avoidance_substitutions':adjustments,'status':'passed'})
                stats['episodes']+=1; stats['motion_commands']+=moved
                env.close()
    stats.update({'maps':len(paths),'spawns_per_map':10,'motions_per_episode':20,'motion_duration_s':DURATION,'simulated_seconds':stats['physics_ticks']*.01,'status':'passed',
                  'duration_s':round(time.perf_counter()-started,3),
                  'tolerances':{'range_m':1e-9,'position_m':1e-9,'yaw_rad':1e-9,'imu_accel_m_s2':1e-9},
                  'python':platform.python_version(),'numpy':np.__version__,'shapely':shapely.__version__,
                  'oracle':'Independent GEOS geometry + complex-plane kinematics; no shared raycast/integrator',
                  'scope':'Ideal deterministic planar sensor model; not a real HC-SR04 acoustic validation',
                  'episodes_detail':episodes})
    (reports/'acceptance.json').write_text(json.dumps(stats,indent=2,allow_nan=False)+'\n')
    (ROOT/'configs'/'acceptance_spawns.json').write_text(json.dumps(starts_output,indent=2)+'\n')
