"""Evaluator utilities. Controllers should never receive ground-truth arguments."""
import math
import numpy as np


def pose_errors(truth,estimate):
    a,b=np.asarray(truth,float),np.asarray(estimate,float)
    if a.ndim!=2 or a.shape[1]!=3 or a.shape!=b.shape or len(a)==0 or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("truth/estimate need equal finite nonempty (N,3) arrays in a shared frame")
    xy=np.linalg.norm(a[:,:2]-b[:,:2],axis=1)
    yaw=(a[:,2]-b[:,2]+math.pi)%(2*math.pi)-math.pi
    return {"position_rmse_m":float(np.sqrt(np.mean(xy*xy))),"position_max_m":float(xy.max()),
            "yaw_rmse_rad":float(np.sqrt(np.mean(yaw*yaw)))}


def occupancy_iou(truth,estimate,threshold=65):
    a,b=np.asarray(truth),np.asarray(estimate)
    if a.shape!=b.shape or a.ndim!=2 or a.size==0:
        raise ValueError("aligned grids must have equal nonempty 2D shapes")
    if not 1<=threshold<=100 or np.any(~np.isfinite(a)) or np.any(~np.isfinite(b)) or np.any((a < -1)|(a>100)) or np.any((b < -1)|(b>100)):
        raise ValueError("invalid occupancy values/threshold")
    valid=(a>=0)&(b>=0)
    if not np.any(valid):
        return {"iou":None,"known_fraction":0.0}
    union=valid&((a>=threshold)|(b>=threshold))
    intersection=valid&(a>=threshold)&(b>=threshold)
    return {"iou":float(intersection.sum()/union.sum()) if union.any() else 1.0,
            "known_fraction":float(valid.mean())}
