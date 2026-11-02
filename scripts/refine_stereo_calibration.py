"""Recover a calibration from saved boards without changing the original archives.

Run with OpenCV and PyYAML available. Fits only training views; every fifth
detected pair per archive is reserved for validation before any residual filtering.
"""
import json
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
from pathlib import Path
import tarfile
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output/calibration/refined-20260913'
OUT.mkdir(parents=True, exist_ok=True)
SOURCES = [
    ('output/calibration/stereo-20260913-165101.tar.gz', True),
    ('output/calibration/session-YrPIZKYq/calibrationdata.tar.gz', False),
    ('output/calibration/stereo-native-20260913-173118.tar.gz', False),
    ('output/calibration/stereo-native-20260913-173736.tar.gz', False),
]
SIZE = (640, 360)
OBJ = np.zeros((48, 3), np.float32)
OBJ[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * .025
cv2.setNumThreads(4)

def detect():
    cache = OUT / 'corners.json'
    if cache.exists():
        return json.loads(cache.read_text())
    records = []
    for source, swap in SOURCES:
        with tarfile.open(ROOT / source) as tar:
            names = sorted(n for n in tar.getnames() if n.startswith('left-') and n.endswith('.png'))
            for index, name in enumerate(names):
                points = []
                for side in (('right', 'left') if swap else ('left', 'right')):
                    image = cv2.imdecode(np.frombuffer(tar.extractfile(name.replace('left-', side+'-')).read(), np.uint8), 0)
                    if image.shape[::-1] != SIZE:
                        raise ValueError('Mixed image dimensions')
                    ok, corners = cv2.findChessboardCornersSB(image, (8, 6), cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
                    if not ok:
                        break
                    c = corners.reshape(-1, 2)
                    if c[-1, 0] < c[0, 0]:
                        c = c[::-1].copy()
                    points.append(c.tolist())
                if len(points) == 2:
                    records.append(dict(source=source, index=index, swapped=swap, points=points, validation=index % 5 == 0))
            print(f'Detected {sum(r["source"] == source for r in records)}/{len(names)} pairs: {source}', flush=True)
    cache.write_text(json.dumps(records))
    return records

def pts(records, side):
    return [np.asarray(r['points'][side], np.float32).reshape(-1, 1, 2) for r in records]

def mono_fit(records, side, flags):
    active = records[:]
    for iteration in range(3):
        rms,k,d,rvecs,tvecs = cv2.calibrateCamera([OBJ]*len(active), pts(active,side), SIZE, None, None, flags=flags)
        errors=[]
        for p,rv,tv in zip(pts(active,side),rvecs,tvecs):
            pred,_=cv2.projectPoints(OBJ,rv,tv,k,d)
            errors.append(float(np.sqrt(np.mean((pred-p)**2)*2)))
        cutoff=max(.5, float(np.median(errors)+2.5*np.median(np.abs(np.array(errors)-np.median(errors)))))
        kept=[r for r,e in zip(active,errors) if e<=cutoff]
        if len(kept)<15 or len(kept)==len(active) or iteration==2: break
        active=kept
    return k,d

def errors(records,k1,d1,k2,d2,r,t):
    r1,r2,p1,p2,*_=cv2.stereoRectify(k1,d1,k2,d2,SIZE,r,t,alpha=0)
    result=[]
    for a,b in zip(pts(records,0),pts(records,1)):
        x=cv2.undistortPoints(a,k1,d1,R=r1,P=p1).reshape(-1,2)
        y=cv2.undistortPoints(b,k2,d2,R=r2,P=p2).reshape(-1,2)
        result.append(x[:,1]-y[:,1])
    return np.array(result), (r1,r2,p1,p2)

def stats(values):
    return dict(rms=float(np.sqrt(np.mean(values**2))),p95_abs=float(np.percentile(np.abs(values),95)),max_abs=float(np.max(np.abs(values))))

def fit(records, flags):
    train=[r for r in records if not r['validation']]
    valid=[r for r in records if r['validation']]
    k1,d1=mono_fit(train,0,flags)
    k2,d2=mono_fit(train,1,flags)
    active=train[:]
    for iteration in range(5):
        result=cv2.stereoCalibrate([OBJ]*len(active),pts(active,0),pts(active,1),k1,d1,k2,d2,SIZE,
            flags=cv2.CALIB_FIX_INTRINSIC,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,100,1e-7))
        rms,k1,d1,k2,d2,r,t,e,f=result
        residual,_=errors(active,k1,d1,k2,d2,r,t)
        pair_rms=np.sqrt(np.mean(residual**2,axis=1))
        cutoff=max(.5, float(np.median(pair_rms)+2.5*np.median(np.abs(pair_rms-np.median(pair_rms)))))
        kept=[a for a,v in zip(active,pair_rms) if v<=cutoff]
        print(f'Fit {flags} iteration {iteration}: {len(active)} pairs, stereo RMS {rms:.3f}, baseline {np.linalg.norm(t)*1000:.2f} mm, cutoff {cutoff:.3f}',flush=True)
        if len(kept)<15 or len(kept)==len(active) or iteration==4: break
        active=kept
    er,rect=errors(valid,k1,d1,k2,d2,r,t)
    tr,_=errors(active,k1,d1,k2,d2,r,t)
    summary=dict(flags=int(flags),training_pairs=len(active),validation_pairs=len(valid),baseline_m=float(np.linalg.norm(t)),
        training_vertical_px=stats(tr),validation_vertical_px=stats(er),K_left=k1.tolist(),K_right=k2.tolist(),
        validation_per_pair=[dict(source=a['source'],index=a['index'],rms=float(np.sqrt(np.mean(v*v)))) for a,v in zip(valid,er)],
        training_used=[dict(source=a['source'],index=a['index']) for a in active])
    summary['validation_by_source']={source:stats(np.array([v for a,v in zip(valid,er) if a['source']==source]))
        for source in sorted({a['source'] for a in valid})}
    return summary,(k1,d1,k2,d2,r,t,rect)

if __name__=='__main__':
    records=detect()
    candidates=[]
    for flags in (cv2.CALIB_FIX_K3, cv2.CALIB_FIX_K3|cv2.CALIB_ZERO_TANGENT_DIST):
        summary,model=fit(records,flags)
        candidates.append((summary,model))
        (OUT/f'candidate-{flags}.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps({k:v for k,v in summary.items() if k not in ('validation_per_pair','training_used')},indent=2),flush=True)
    # Candidate selection uses reserved validation data; report that explicitly.
    summary,model=min(candidates,key=lambda c:c[0]['validation_vertical_px']['rms'])
    k1,d1,k2,d2,r,t,(r1,r2,p1,p2)=model
    for side,k,d,rr,p in [('left',k1,d1,r1,p1),('right',k2,d2,r2,p2)]:
        def mat(a): return dict(rows=a.shape[0],cols=a.shape[1],data=a.ravel().tolist())
        cfg=dict(image_width=640,image_height=360,camera_name='imx219_'+side,camera_matrix=mat(k),
                 distortion_model='plumb_bob',distortion_coefficients=mat(d.reshape(1,-1)),rectification_matrix=mat(rr),projection_matrix=mat(p))
        (OUT/(side+'.yaml')).write_text(yaml.safe_dump(cfg,sort_keys=False))
    summary.update(R_left_to_right=r.tolist(),T_left_to_right_m=t.ravel().tolist(),
        validation_note='Reserved views excluded from fitting and outlier rejection; used to choose between two models. Existing captures, not a new independent session. No hardware deployment.')
    (OUT/'review.json').write_text(json.dumps(summary,indent=2)+'\n')
