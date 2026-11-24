"""Rebuild the ten bundled deterministic acceptance maps."""
from pathlib import Path
import json
import numpy as np
from robot_sim import WorldMap


def box(x0,y0,x1,y1):
    return [[x0,y0],[x1,y0],[x1,y1],[x0,y1]]


def generate(folder):
    folder=Path(folder); folder.mkdir(parents=True,exist_ok=True)
    specifications=[
        ("01_empty_room",[0,0,4,4],[]),
        ("02_central_box",[0,0,4,4],[box(1.5,1.4,2.3,2.5)]),
        ("03_l_corridor",[0,0,5,5],[box(2,2,5,5)]),
        ("04_t_junction",[0,0,5,5],[box(0,0,1.8,2),box(3.2,0,5,2),box(0,3.2,5,5)]),
        ("05_slanted_walls",[0,0,5,4],[[[0,2.5],[0,4],[1.8,4]],[[5,0],[3.4,0],[5,1.2]],[[2.2,1.3],[2.7,2.2],[1.8,2.4]]]),
        ("06_doorway",[0,0,5,4],[box(2.4,0,2.6,1.5),box(2.4,2.4,2.6,4)]),
        ("07_zigzag_maze",[0,0,6,5],[box(1.4,0,1.6,3.6),box(2.9,1.4,3.1,5),box(4.4,0,4.6,3.6)]),
        ("08_irregular",[0,0,5,5],[[[1,1],[2.3,1.2],[2.4,1.7],[1.6,1.8],[1.5,2.8],[.8,2.4]],[[3.2,3],[4.2,2.7],[4.6,4],[3.5,4.4]]]),
        ("09_clutter",[0,0,5,5],[box(.7,.8,1.1,1.3),box(2.2,.6,2.7,1.2),box(3.7,1.2,4.2,1.6),box(1.7,2.3,2.1,2.8),box(.8,3.6,1.4,4.1),box(3.3,3.2,4.1,3.8)]),
    ]
    for name,bounds,polygons in specifications:
        spec={"version":1,"units":"m","name":name,"bounds":bounds,"polygons":polygons,"walls":[],
              "metadata":{"purpose":"deterministic simulator acceptance","coordinates":"x right, y up; radians CCW"}}
        if name=="08_irregular":
            spec["walls"]=[[[2.8,.2],[3.5,1.6]]]
        WorldMap(spec).save(folder/(name+".json"))
    grid=np.zeros((25,25),dtype=int)
    grid[3:9,5:8]=100
    grid[13:21,13:16]=100
    grid[6:9,17:22]=100
    grid[18:21,3:8]=100
    WorldMap.from_occupancy(grid,.2,name="10_occupancy_import").save(folder/"10_occupancy_import.json")
    return sorted(folder.glob("*.json"))


if __name__=="__main__":
    generate(Path(__file__).resolve().parents[1]/"maps")
