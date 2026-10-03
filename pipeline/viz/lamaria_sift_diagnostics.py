#!/usr/bin/env python3
"""Show the actual native-image SIFT matches used by batch refinement."""
import argparse
import json
from pathlib import Path
import sqlite3
import cv2
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--work',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args(); a.out.mkdir(parents=True,exist_ok=True)
    con=sqlite3.connect(f'file:{a.work / "database.db"}?mode=ro',uri=True,timeout=60)
    images={n:i for i,n in con.execute('select image_id,name from images')}
    stamps=np.array(sorted(int(Path(n).stem) for n in images if n.startswith('cam0/')),dtype=np.int64)
    report=[]
    for target in (80.,300.,529.,800.):
        i=min(int(np.argmin(abs(stamps/1e9-target))),len(stamps)-21)
        for cam in (0,1):
            names=[f'cam{cam}/{stamps[k]}.png' for k in (i,i+20)]
            ids=[images[n] for n in names]; lo,hi=sorted(ids)
            row=con.execute('select rows,data from two_view_geometries where pair_id=?',(lo*2147483647+hi,)).fetchone()
            matches=np.frombuffer(row[1],np.uint32).reshape(-1,2).copy() if row and row[0] else np.empty((0,2),np.uint32)
            if ids[0]>ids[1]: matches=matches[:,::-1]
            kpts=[]; pictures=[]
            for iid,n in zip(ids,names):
                nr,nc,blob=con.execute('select rows,cols,data from keypoints where image_id=?',(iid,)).fetchone()
                kpts.append(np.frombuffer(blob,np.float32).reshape(nr,nc)[:,:2]-.5)
                im=cv2.imread(str(a.work/'images'/n)); assert im is not None
                pictures.append(im)
            h,w=pictures[0].shape[:2]; canvas=np.vstack([np.zeros((60,2*w,3),np.uint8),np.hstack(pictures)])
            # Distinct grid cells retain central and peripheral evidence without
            # hiding the image under every orientation-duplicate SIFT match.
            chosen=[]; bins=set()
            for pair in matches:
                x,y=kpts[0][pair[0]]; cell=(int(x/60),int(y/60))
                if cell not in bins:
                    bins.add(cell); chosen.append(pair)
            for j,pair in enumerate(chosen):
                color=tuple(map(int,cv2.cvtColor(np.uint8([[[j*37%180,200,255]]]),cv2.COLOR_HSV2BGR)[0,0]))
                q0=np.rint(kpts[0][pair[0]]+[0,60]).astype(int)
                q1=np.rint(kpts[1][pair[1]]+[w,60]).astype(int)
                cv2.line(canvas,tuple(q0),tuple(q1),color,1,cv2.LINE_AA)
                cv2.circle(canvas,tuple(q0),3,color,1,cv2.LINE_AA)
                cv2.circle(canvas,tuple(q1),3,color,1,cv2.LINE_AA)
            line=f'cam{cam}   {stamps[i]/1e9:.3f} s -> {stamps[i+20]/1e9:.3f} s    verified SIFT: {len(matches)}, drawn: {len(chosen)}'
            cv2.putText(canvas,line,(12,23),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(canvas,'Native sensor orientation. Geometric 2D matching; landmark support is tested separately.',(12,47),cv2.FONT_HERSHEY_SIMPLEX,.48,(200,200,200),1,cv2.LINE_AA)
            name=f'matches_{int(target):04d}s_cam{cam}.png';cv2.imwrite(str(a.out/name),canvas)
            report.append(dict(camera=cam,first_timestamp_ns=int(stamps[i]),second_timestamp_ns=int(stamps[i+20]),
                extracted_features=list(map(len,kpts)),geometric_matches=len(matches),drawn_matches=len(chosen),file=name))
    (a.out/'matches.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
