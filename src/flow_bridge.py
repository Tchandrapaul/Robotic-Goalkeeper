"""Short, image-measured optical-flow bridge; never runs indefinitely without outlines."""
import cv2
import numpy as np


class FlowBridge:
    def __init__(self):
        self.reset()

    def reset(self):
        self.gray=None;self.ball=None;self.time=None;self.outline_time=None

    def remember(self,gray,ball,now):
        self.gray=gray.copy();self.ball=dict(ball);self.time=now
        if ball['source']=='outline':self.outline_time=now

    def estimate(self,gray,color_mask,allowed,now):
        if self.gray is None or self.outline_time is None or not 0<now-self.time<=.15 or now-self.outline_time>.20:
            return None
        b=self.ball;x,y,r=b['x'],b['y'],b['radius']
        feature_mask=np.zeros(self.gray.shape,np.uint8)
        cv2.circle(feature_mask,(round(x),round(y)),max(4,round(.8*r)),255,-1)
        points=cv2.goodFeaturesToTrack(self.gray,30,.015,4,mask=feature_mask,blockSize=5)
        if points is None or len(points)<6:return None
        params=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,25,.02))
        next_points,valid,error=cv2.calcOpticalFlowPyrLK(self.gray,gray,points,None,**params)
        if next_points is None:return None
        back,valid_back,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,next_points,None,**params)
        if back is None:return None
        keep=(valid.ravel()>0)&(valid_back.ravel()>0)&(error.ravel()<25)&(np.linalg.norm(points-back,axis=2).ravel()<1.2)
        old=points[keep].reshape(-1,2);new=next_points[keep].reshape(-1,2)
        if len(old)<6:return None
        transform,inliers=cv2.estimateAffinePartial2D(old,new,method=cv2.RANSAC,ransacReprojThreshold=2,maxIters=200)
        if transform is None or not np.isfinite(transform).all() or inliers.sum()<6 or inliers.mean()<.65:return None
        scale=float(np.hypot(transform[0,0],transform[1,0]))
        if not .85<scale<1.18:return None
        center=transform@np.array([x,y,1.]);radius=r*scale
        xx,yy=center;h,w=gray.shape
        if xx-radius<0 or xx+radius>=w or yy-radius<0 or yy+radius>=h:return None
        disk=np.zeros(gray.shape,np.uint8);cv2.circle(disk,(round(xx),round(yy)),max(4,round(radius*.8)),255,-1)
        inside=disk>0
        if np.mean(allowed[inside]>0)<.98 or np.mean(color_mask[inside]>0)<.06:return None
        return dict(x=float(xx),y=float(yy),radius=float(radius),quality=.4,
                    source='optical_flow',measured=True,flow_age_s=now-self.outline_time)
