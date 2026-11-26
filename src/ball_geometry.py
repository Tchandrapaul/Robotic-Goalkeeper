"""Experimental monocular sphere estimate from floor homography and camera center.
Uses a small-sphere projection approximation; outline errors directly affect depth.
"""
from collections import deque
import numpy as np


class BallGeometry:
    def __init__(self, calibration, settings):
        self.h=np.asarray(calibration['image_to_floor'],float)
        self.image_size=tuple(calibration['image_size'])
        self.radius=settings['ball_diameter_m']/2
        self.camera=np.asarray(settings['camera_center_m'],float)
        if self.h.shape!=(3,3) or not np.isfinite(self.h).all() or self.camera.shape!=(3,) or not np.isfinite(self.camera).all() or self.camera[2]<=self.radius or self.radius<=0:
            raise ValueError('Invalid camera/ball geometry')
        plane=np.linalg.inv(self.h)
        vertical=-(plane[:,0]*self.camera[0]+plane[:,1]*self.camera[1]+plane[:,2])/self.camera[2]
        self.p=np.column_stack((plane[:,:2],vertical,plane[:,2]))
        reference=np.mean(np.asarray(calibration['floor_points_m']),axis=0)
        if (self.p@np.array([*reference,0,1]))[2]<0:self.p=-self.p
        self.history=deque(maxlen=10)
        self.last=None
        self.reason='Waiting for measured outline'

    def reset(self):
        self.history.clear();self.last=None

    def estimate(self, detection, frame_shape):
        if (frame_shape[1],frame_shape[0])!=self.image_size:
            self.reason='Resolution mismatch - recalibrate';return None
        u,v,r=detection['x'],detection['y'],detection['radius']
        if r<8:return None
        q=self.h@np.array([u,v,1.])
        if abs(q[2])<1e-8:return None
        ground=np.array([q[0]/q[2],q[1]/q[2],0.])
        image=self.p@np.r_[ground,1.]
        if abs(image[2])<1e-8:return None
        # Local image scale of a sphere at the ray's floor intersection.
        jacobian=(self.p[:2,:3]*image[2]-image[:2,None]*self.p[2,:3])/image[2]**2
        singular=np.linalg.svd(jacobian,compute_uv=False)
        floor_radius=self.radius*np.sqrt(singular[0]*singular[1])
        scale=np.sign(image[2])*floor_radius/r
        position=self.camera+scale*(ground-self.camera)
        if not np.isfinite(position).all() or abs(position[0])>3 or not .05<position[1]<6 or not self.radius-.08<position[2]<3:
            self.reason='Size/depth inconsistent - holding';return None
        position[2]=max(self.radius,position[2])
        return position

    def update(self,detection,frame_shape,now):
        if self.history and now-self.history[-1][0]>.25:self.reset()
        if detection is not None and detection.get('measured'):
            position=self.estimate(detection,frame_shape)
            if position is None:
                self.reset();return None,None,None
            if detection.get('motion_reset'):self.reset()
            if self.history:
                dt=now-self.history[-1][0]
                if dt<=0:return None,None,None
                if np.linalg.norm(position-self.history[-1][1])>max(.20,15*dt):
                    self.reset() # New motion/outline; require fresh velocity rather than extrapolate.
            self.history.append((now,position))
            self.last=position
            self.reason=('Optical-flow size -> estimated 3D' if detection.get('source')=='optical_flow' else 'Measured outline -> estimated 3D')
        elif self.history and now-self.history[-1][0]<=.10:
            self.reason='Predicted 3D gap'
        else:
            self.reason='No fresh outline - holding';return None,None,None
        times=np.array([a[0] for a in self.history]);positions=np.array([a[1] for a in self.history])
        latest=positions[-1].copy();age=now-times[-1]
        if len(times)<4 or times[-1]-times[0]<.06:return latest,None,None
        t=times-times[-1]
        air=bool(np.median(positions[-3:,2])>self.radius+.06)
        gravity=9.81 if air else 0.
        adjusted=positions.copy();adjusted[:,2]+=.5*gravity*t*t
        design=np.column_stack((np.ones(len(t)),t))
        coeff=np.linalg.lstsq(design,adjusted,rcond=None)[0]
        residual=np.sqrt(np.mean((design@coeff-adjusted)**2))
        if residual>.07:
            self.reason='Trajectory inconsistent - collecting fresh motion'
            self.history=deque(list(self.history)[-2:],maxlen=10)
            return latest,None,None
        pos,velocity=coeff
        pos=pos+velocity*age;pos[2]-=.5*gravity*age*age
        velocity=velocity.copy();velocity[2]-=gravity*age
        if not air:pos[2]=self.radius;velocity[2]=0
        if pos[2]<self.radius-.02:
            self.reason='Possible bounce - waiting for fresh observations';return latest,None,None
        if velocity[1]>=-.08:
            self.reason+=' | not approaching';return pos,None,None
        arrival=-pos[1]/velocity[1]
        if not .02<arrival<=2:
            self.reason+=' | outside prediction horizon';return pos,None,None
        target=pos+velocity*arrival;target[2]-=.5*gravity*arrival*arrival
        if target[2]<self.radius:
            self.reason='Bounce before goal - no extrapolation through bounce';return pos,None,None
        self.reason+= ' | AIR estimate' if air else ' | rolling estimate'
        return pos,target,float(arrival)


def paddle_target(target, current, settings, arrival=None):
    """Find overlap with a provisional rectangular paddle in the blocking plane."""
    if target is None:return None,'No interception estimate'
    x,_,z=target
    radius=settings['ball_diameter_m']/2
    if abs(x)>settings['goal_width_m']/2+radius or z>settings['goal_height_m']+radius:
        return None,'Outside modeled goal'
    angles=np.arange(settings['angle_min_deg'],settings['angle_max_deg']+.01,.5)
    a=np.radians(angles);dz=z-settings['pivot_height_m']
    along=x*np.cos(a)+dz*np.sin(a)
    across=-x*np.sin(a)+dz*np.cos(a)
    lower=settings['paddle_below_pivot_m'];upper=settings['paddle_length_m']-lower
    distance=np.hypot(np.maximum(np.maximum(-lower-along,along-upper),0),np.maximum(abs(across)-settings['paddle_width_m']/2,0))
    valid=distance<=radius*.85
    if not valid.any():return None,'Outside provisional paddle reach'
    candidates=angles[valid];clearance=distance[valid]
    score=abs(candidates-current)+clearance*100
    selected=float(candidates[np.argmin(score)])
    return selected,'Provisional overlap; floor clearance/stopping unverified'


def preview_command(position, target, current, settings, arrival=None):
    """Follow is visibly distinct from an estimated interception, simulation only."""
    if target is not None:
        command,note=paddle_target(target,current,settings,arrival)
        return command,'INTERCEPT | '+note
    if position is None:
        return None,'HOLD | no usable ball position'
    x,_,z=position
    if not np.isfinite(position).all():return None,'HOLD | invalid position'
    dz=z-settings['pivot_height_m']
    if np.hypot(x,dz)<1e-6:return None,'HOLD | target at pivot'
    desired=np.degrees(np.arctan2(dz,x))
    if not 0<=desired<=180:return None,'HOLD | position below permitted pointing range'
    command=float(np.clip(desired,settings['angle_min_deg'],settings['angle_max_deg']))
    return command,'FOLLOW | current estimated position; NOT an interception'
