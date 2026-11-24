import json
import math
import random
from pathlib import Path
import numpy as np
import pytest
from robot_sim import WorldMap
from robot_sim.geometry import integrate
from oracle import Oracle,motion

MAPS=sorted((Path(__file__).resolve().parents[1]/"maps").glob("*.json"))

@pytest.mark.parametrize("map_path",MAPS,ids=lambda p:p.stem)
def test_random_geometry_against_geos(map_path):
    spec=json.loads(map_path.read_text()); world=WorldMap(spec); oracle=Oracle(spec)
    rng=random.Random(1729)
    compared=0
    while compared<80:
        x0,y0,x1,y1=world.bounds
        p=(rng.uniform(x0,x1),rng.uniform(y0,y1))
        assert world.clearance(p)==pytest.approx(oracle.clearance(p),abs=1e-9)
        if oracle.clearance(p)<.03:
            continue
        a=rng.uniform(-math.pi,math.pi)
        for width in (0,math.radians(15),math.radians(65)):
            actual=world.range(p,a,4,width)
            expected=oracle.range(p,a,4,width)
            assert actual==pytest.approx(expected,abs=1e-9)
        compared+=1

@pytest.mark.parametrize("angle,expected",[(0,4),(math.pi/2,4),(math.pi,1),(-math.pi/2,1),(math.pi/4,4*math.sqrt(2))])
def test_analytic_room(room,angle,expected):
    assert room.range((1,1),angle,10)==pytest.approx(expected,abs=1e-10)


def test_cone_detects_between_sparse_rays():
    # A small reflector away from center and sector boundary rays.
    a=math.radians(3.33)
    x,y=2+math.cos(a),2+math.sin(a)
    spec={"version":1,"bounds":[0,0,8,8],"walls":[[[x,y-.0001],[x,y+.0001]]]}
    w=WorldMap(spec)
    assert w.range((2,2),0,4)==math.inf
    assert w.range((2,2),0,4,math.radians(15))==pytest.approx(Oracle(spec).range((2,2),0,4,math.radians(15)),abs=1e-10)
    assert w.range((2,2),0,4,math.radians(15))<1.001


def test_collinear_segment_and_occlusion():
    w=WorldMap({"version":1,"bounds":[0,0,5,5],"walls":[[[2,1],[3,1]],[[4,.5],[4,1.5]]]})
    assert w.range((1,1),0,4)==pytest.approx(1)
    assert w.range((3.5,1),math.pi,4)==pytest.approx(.5)
    assert w.range((2.5,1),0,4)==0

@pytest.mark.parametrize("v,w",[(.15,0),(.1,1),(-.1,1),(.1,-2),(0,3),(.1,1e-11)])
def test_integration_constant_twist(v,w):
    left=(v-.135*w/2)/.3*100
    right=(v+.135*w/2)/.3*100
    result=integrate(1,2,.7,v,w,.4)
    expected=motion((1,2,.7),left,right,.4)
    np.testing.assert_allclose(result,expected,atol=1e-10,rtol=0)

@pytest.mark.parametrize("bad",[
    {"version":2,"bounds":[0,0,1,1]},
    {"version":1,"bounds":[0,0,-1,1]},
    {"version":1,"bounds":[0,0,2,2],"polygons":[[[0,0],[1,1],[0,1],[1,0]]]},
    {"version":1,"bounds":[0,0,2,2],"walls":[[[1,1],[1,1]]]},
    {"version":1,"bounds":[0,0,2,2],"polygons":[[[0,0],[1,0],[1,0],[0,1]]]},
    {"version":1,"bounds":[0,0,float("nan"),2]},
    {"version":1,"bounds":[0,0,2,2],"misspelled":5},
])
def test_invalid_geometry(bad):
    with pytest.raises(ValueError):
        WorldMap(bad)
