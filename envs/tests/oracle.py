"""Independent geometry oracle: GEOS/Shapely, built from raw JSON, NOT runtime edges."""
import cmath
import math
from shapely.geometry import Point,LineString,Polygon,box
from shapely.ops import unary_union


class Oracle:
    def __init__(self,spec):
        self.bounds=spec["bounds"]
        b=box(*self.bounds)
        self.solids=[Polygon(p) for p in spec.get("polygons",[])]
        if "occupancy" in spec:
            g=spec["occupancy"]; r=g["resolution_m"]; x,y=g.get("origin_m",self.bounds[:2])
            for row,cells in enumerate(g["cells"]):
                for col,v in enumerate(cells):
                    if v>=g.get("occupied_threshold",65) or (v==-1 and g.get("unknown_policy")=="occupied"):
                        self.solids.append(box(x+col*r,y+row*r,x+(col+1)*r,y+(row+1)*r))
        parts=[b.boundary]+self.solids+[LineString(w) for w in spec.get("walls",[])]
        self.geometry=unary_union(parts)

    def clearance(self,p):
        x,y=p; x0,y0,x1,y1=self.bounds
        if not x0<x<x1 or not y0<y<y1:
            return 0.0
        return Point(p).distance(self.geometry)

    def range(self,p,angle,maximum,width=0):
        x,y=p
        if width<=1e-12:
            beam=LineString([p,(x+maximum*math.cos(angle),y+maximum*math.sin(angle))])
        else:
            r=maximum/math.cos(width/2)*1.01
            beam=Polygon([p,(x+r*math.cos(angle-width/2),y+r*math.sin(angle-width/2)),
                            (x+r*math.cos(angle+width/2),y+r*math.sin(angle+width/2))])
        intersection=self.geometry.intersection(beam)
        if intersection.is_empty:
            return math.inf
        distance=Point(p).distance(intersection)
        return distance if distance<=maximum+1e-9 else math.inf

    def path_free(self,poses,radius):
        if min(self.clearance(p[:2]) for p in poses)<=radius+1e-7:
            return False
        return LineString([p[:2] for p in poses]).distance(self.geometry)>radius+1e-6


def motion(pose,left,right,dt,base=.135,maxspeed=.3):
    """Complex-plane closed form; no calls to production integration routines."""
    x,y,theta=pose
    v=(left+right)/200*maxspeed
    w=(right-left)/100*maxspeed/base
    heading=cmath.exp(1j*theta)
    dz=v*heading*dt if abs(w)<1e-10 else v*heading*(cmath.exp(1j*w*dt)-1)/(1j*w)
    theta=math.atan2(math.sin(theta+w*dt),math.cos(theta+w*dt))
    return (x+dz.real,y+dz.imag,theta)
