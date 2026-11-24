"""Convert collected occupancy arrays / grayscale PNG or PGM to map JSON.

Raw ultrasound readings need a mapping algorithm first. This utility converts an
already spatially registered occupancy grid; it does not infer a map from ranges.
"""
from pathlib import Path
import argparse
import numpy as np
from .world import WorldMap


def convert_grid(source,output,*,resolution_m,origin_m=(0,0),unknown_policy='reject',
                 image_occupied_threshold=.65,image_free_threshold=.196,top_to_bottom=False,encoding='percent'):
    if encoding not in ('percent','binary','probability'):
        raise ValueError('encoding must be percent/binary/probability')
    source=Path(source)
    if source.suffix.lower()=='.npy':
        cells=np.load(source,allow_pickle=False)
    elif source.suffix.lower()=='.csv':
        cells=np.loadtxt(source,delimiter=',',ndmin=2)
    elif source.suffix.lower() in ('.png','.pgm','.bmp'):
        from PIL import Image
        if not 0<=image_free_threshold<image_occupied_threshold<=1:
            raise ValueError('image thresholds must satisfy 0 <= free < occupied <= 1')
        with Image.open(source) as image:
            gray=np.asarray(image.convert('L'),dtype=float)
        probability=1-gray/255
        cells=np.full(gray.shape,-1,dtype=int)
        cells[probability<=image_free_threshold]=0
        cells[probability>=image_occupied_threshold]=100
        # Pixel row zero is top, while world grid row zero is bottom.
        cells=np.flipud(cells)
        top_to_bottom=False
        encoding='percent'
    else:
        raise ValueError('supported inputs: .npy, .csv, .png, .pgm, .bmp')
    if encoding in ('binary','probability'):
        a=np.asarray(cells,float)
        if not np.isfinite(a).all() or np.any((a!=-1)&((a<0)|(a>1))):
            raise ValueError('binary/probability values must be -1 or 0..1')
        if encoding=='binary' and np.any((a!=-1)&(a!=0)&(a!=1)):
            raise ValueError('binary values must be -1,0,1')
        cells=np.where(a==-1,-1,np.rint(a*100)).astype(int)
    if top_to_bottom:
        cells=np.flipud(cells)
    world=WorldMap.from_occupancy(cells,resolution_m,origin_m,unknown_policy=unknown_policy,name=source.stem)
    world.save(output)
    return world


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source'); parser.add_argument('output')
    parser.add_argument('--resolution',type=float,required=True,help='metres per grid cell')
    parser.add_argument('--origin',nargs=2,type=float,default=(0,0),metavar=('X','Y'))
    parser.add_argument('--unknown-policy',choices=['reject','occupied','free'],default='reject')
    parser.add_argument('--top-to-bottom',action='store_true',help='flip CSV/NPY rows from image order')
    parser.add_argument('--encoding',choices=['percent','binary','probability'],default='percent')
    args=parser.parse_args()
    convert_grid(args.source,args.output,resolution_m=args.resolution,origin_m=args.origin,
                 unknown_policy=args.unknown_policy,top_to_bottom=args.top_to_bottom,encoding=args.encoding)
    print(args.output)


if __name__=='__main__':
    main()
