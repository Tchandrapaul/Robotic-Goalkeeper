"""Experimental color/outline tracker and single-camera goalie simulator. No GPIO."""
import argparse
import json
import threading
import time
from collections import Counter
from pathlib import Path
import cv2
import numpy as np
from ball_tracking import BallTracker
from ball_geometry import BallGeometry, preview_command
from camera_setup import configure_camera_timing
from tracking_area import select_tracking_area, area_mask


class CameraStream:
    def __init__(self,cap):
        self.cap=cap;self.lock=threading.Lock();self.stop=threading.Event()
        self.frame=None;self.timestamp=0.;self.sequence=0;self.failed=False
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        while not self.stop.is_set():
            ok,frame=self.cap.read()
            if not ok:self.failed=True;return
            with self.lock:
                self.frame=frame;self.timestamp=time.monotonic();self.sequence+=1
    def get(self):
        with self.lock:return self.frame,self.timestamp,self.sequence
    def close(self):
        self.stop.set();self.thread.join(timeout=2)
        if not self.thread.is_alive():self.cap.release()


def draw_simulation(position,target,angle,command,settings,status):
    canvas=np.zeros((620,1100,3),np.uint8)
    def label(text,x,y,color=(230,230,230),size=.48):
        cv2.putText(canvas,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,1)
    label('EXPERIMENTAL SINGLE-CAMERA 3D - size/outline errors affect depth',12,25)
    label(status[:145],12,50)
    label('Provisional paddle geometry. No physical motor commands.',12,75)
    # Top-down uses one metric scale, y positive away from the goal.
    def top(x,y):return int(270+np.clip(x,-3,3)*110),int(555-np.clip(y,-.1,4)*110)
    for y in range(5):
        cv2.line(canvas,top(-2,y),top(2,y),(45,45,45),1);label(f'{y}m',15,top(0,y)[1])
    half=settings['goal_width_m']/2
    cv2.line(canvas,top(-half,0),top(half,0),(255,255,255),5)
    label('TOP DOWN',210,110)
    if position is not None:
        cv2.circle(canvas,top(position[0],position[1]),6,(0,255,0),-1)
    if target is not None:
        cv2.drawMarker(canvas,top(target[0],0),(0,200,255),cv2.MARKER_CROSS,18,2)
        if position is not None:cv2.line(canvas,top(position[0],position[1]),top(target[0],0),(0,200,255),2)
    # Front view uses the same scale in x and z; paddle rectangle includes lower end.
    def front(x,z):return int(820+np.clip(x,-2,2)*330),int(555-np.clip(z,-.1,2)*330)
    cv2.line(canvas,front(-.8,0),front(.8,0),(150,150,150),2)
    for x in (-half,half):cv2.line(canvas,front(x,0),front(x,settings['goal_height_m']),(255,255,255),2)
    cv2.line(canvas,front(-half,settings['goal_height_m']),front(half,settings['goal_height_m']),(255,255,255),2)
    pivot=settings['pivot_height_m'];lower=settings['paddle_below_pivot_m'];upper=settings['paddle_length_m']-lower;width=settings['paddle_width_m']
    a=np.radians(angle);corners=[]
    for along,across in [(-lower,-width/2),(upper,-width/2),(upper,width/2),(-lower,width/2)]:
        corners.append(front(along*np.cos(a)-across*np.sin(a),pivot+along*np.sin(a)+across*np.cos(a)))
    cv2.polylines(canvas,[np.int32(corners)],True,(0,255,0),2)
    cv2.circle(canvas,front(0,pivot),5,(255,255,255),-1)
    if target is None and position is not None:
        cv2.circle(canvas,front(position[0],position[2]),max(2,round(settings['ball_diameter_m']/2*330)),(255,255,0),2)
    if target is not None:cv2.circle(canvas,front(target[0],target[2]),max(2,round(settings['ball_diameter_m']/2*330)),(0,200,255),2)
    label('FRONT: orange intercept | cyan current position',600,110)
    label(f'Paddle {angle:.1f} deg | command '+('hold' if command is None else f'{command:.1f}'),600,140)
    label('Floor overlap in drawing = unverified shaft offset',600,165,(0,170,255))
    if position is not None:label(f'Est. x={position[0]:+.2f} y={position[1]:.2f} z={position[2]:.2f} m',30,600)
    return canvas


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=root/'config/hsv_config.json',help='Ball HSV calibration JSON')
    parser.add_argument('--settings',type=Path,default=root/'config/live_settings.json',help='Measured geometry settings JSON')
    parser.add_argument('--calibration',type=Path,default=root/'config/floor_calibration.json',help='Floor calibration JSON')
    parser.add_argument('--camera',type=int,default=0,help='Linux camera index (default: 0)')
    parser.add_argument('--calibrate',action='store_true',help='Run notebook calibration before tracking')
    args=parser.parse_args()
    try:
        config=json.loads(args.config.read_text())
        settings=json.loads(args.settings.read_text())
        if args.calibrate:
            from calibrate_floor import main as calibrate_notebook
            print('If camera height or position relative to the blocking line changed, update your geometry settings too.')
            if not calibrate_notebook(camera=args.camera, output=args.calibration):
                print('Calibration cancelled; tracking not started. Existing calibration kept.')
                return
        calibration=json.loads(args.calibration.read_text())
    except (OSError,json.JSONDecodeError) as exc:
        parser.error(f'{exc}. Copy config/live_settings.example.json to config/live_settings.json, edit measurements, and run src/calibrate_floor.py. See README.')
    tracker=BallTracker(config);geometry=BallGeometry(calibration,settings)
    cv2.setNumThreads(2)
    cap=cv2.VideoCapture(args.camera,cv2.CAP_V4L2)
    if not cap.isOpened():raise SystemExit(f'Camera {args.camera} unavailable. Reconnect and restart.')
    stream=None
    try:
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
        cap.set(cv2.CAP_PROP_FPS,60);cap.set(cv2.CAP_PROP_BUFFERSIZE,4)
        configure_camera_timing(args.camera)
        ok,setup=cap.read()
        if not ok:raise RuntimeError('Camera read failed')
        if (setup.shape[1],setup.shape[0])!=tuple(calibration['image_size']):raise RuntimeError('Resolution differs from floor calibration')
        print('Select the entire ball corridor INCLUDING air above the floor. Exclude furniture where possible.')
        region=select_tracking_area(setup)
        if region is None:return
        allowed=area_mask(setup.shape,region)
        stream=CameraStream(cap)
        angle=90.;last_update=time.monotonic();last_log=last_update;last_seq=-1
        counters=Counter();processing=[];show_mask=False;fps=0.
        while True:
            frame,stamp,seq=stream.get()
            if stream.failed:raise RuntimeError('Camera disconnected/read failed')
            if frame is None or seq==last_seq:
                if cv2.waitKey(1)&255==ord('q'):break
                time.sleep(.001);continue
            last_seq=seq
            if frame.shape[:2]!=allowed.shape:raise RuntimeError('Camera resolution changed; restart')
            start=time.monotonic()
            detection,mask=tracker.update(frame,allowed,stamp)
            position,target,arrival=geometry.update(detection,frame.shape,stamp)
            now=time.monotonic();dt=min(.05,max(0,now-last_update));last_update=now
            command,note=preview_command(position,target,angle,settings,arrival)
            if command is not None:
                angle+=float(np.clip(command-angle,-settings['preview_speed_deg_s']*dt,settings['preview_speed_deg_s']*dt))
            canvas=frame.copy()
            if detection is not None:
                color=(0,255,0) if detection['measured'] else (0,180,255)
                cv2.circle(canvas,(round(detection['x']),round(detection['y'])),round(detection['radius']),color,2)
            counters['frames']+=1
            counters['measured' if detection is not None and detection['measured'] else 'predicted' if detection is not None else 'missing']+=1
            rx,ry,rw,rh=region;cv2.rectangle(canvas,(rx,ry),(rx+rw,ry+rh),(255,180,0),1)
            labels=[f'{fps:.1f} processed FPS | '+tracker.status,geometry.reason,note,
                    'Q quit | R reselect area | S snapshot | M mask']
            for i,text in enumerate(labels):cv2.putText(canvas,text,(10,25+i*25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
            sim=draw_simulation(position,target,angle,command,settings,note)
            processing.append((time.monotonic()-start)*1000)
            cv2.imshow('Ball tracking - outline',canvas);cv2.imshow('Goalie simulation',sim)
            if show_mask:cv2.imshow('Color seeds',mask)
            if now-last_log>=1:
                fps=counters['frames']/(now-last_log)
                crossing='none' if target is None else f'x={target[0]:+.3f} z={target[2]:.3f} arrival={arrival:.3f}s'
                print(f'TRACK | fps={fps:.1f} measured={counters["measured"]} predicted={counters["predicted"]} missing={counters["missing"]} median_process={np.median(processing):.1f}ms | {tracker.status}',flush=True)
                print(f'AIM | {crossing} | {geometry.reason} | {note}',flush=True)
                counters.clear();processing.clear();last_log=now
            key=cv2.waitKey(1)&255
            if key==ord('q'):break
            if key==ord('r'):
                selected=select_tracking_area(frame)
                if selected is not None:region=selected;allowed=area_mask(frame.shape,region)
                tracker.reset();geometry.reset()
            elif key==ord('m'):
                show_mask=not show_mask
                if not show_mask:cv2.destroyWindow('Color seeds')
            elif key==ord('s'):
                folder=root/'snapshots'/str(time.time_ns());folder.mkdir(parents=True)
                for name,image in [('camera',frame),('overlay',canvas),('mask',mask),('simulation',sim)]:cv2.imwrite(str(folder/(name+'.png')),image)
                (folder/'state.json').write_text(json.dumps({'detection':detection,'tracking':tracker.status,'mode':note,'geometry':geometry.reason,'position':None if position is None else position.tolist(),'target':None if target is None else target.tolist(),'arrival':arrival,'angle':angle},indent=2))
                print('Saved snapshot:',folder,flush=True)
    finally:
        if stream is not None:stream.close()
        else:cap.release()
        cv2.destroyAllWindows()


if __name__=='__main__':main()
