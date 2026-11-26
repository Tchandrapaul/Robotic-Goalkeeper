"""Color-seeded outline detection with bounded temporal association."""
from collections import deque
from flow_bridge import FlowBridge
import cv2
import numpy as np


class BallTracker:
    def __init__(self, config):
        self.lower = np.uint8([config['h_min'],config['s_min'],config['v_min']])
        self.upper = np.uint8([config['h_max'],config['s_max'],config['v_max']])
        self.history = deque(maxlen=6)
        self.pending = None
        self.pending_count = 0
        self.flow = FlowBridge()
        self.status = 'Searching'
        self.last_radius = None

    def reset(self):
        self.flow.reset()
        self.history.clear()
        self.pending = None
        self.pending_count = 0
        self.last_radius = None

    def velocity(self):
        if len(self.history)<2:
            return np.zeros(2)
        a=np.asarray(self.history);t=a[:,0]-a[-1,0];t-=t.mean()
        den=t@t
        return np.zeros(2) if den<1e-8 else (t@a[:,1:3])/den

    def candidates(self, frame, allowed):
        blurred=cv2.GaussianBlur(frame,(5,5),0)
        hsv=cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,self.lower,self.upper)
        mask=cv2.bitwise_and(mask,allowed)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        mask=cv2.bitwise_and(mask,allowed)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        seeds=[]
        for c in sorted(contours,key=cv2.contourArea,reverse=True)[:8]:
            if cv2.contourArea(c)<45:continue
            (x,y),r=cv2.minEnclosingCircle(c)
            if r<5 or r>180:continue
            if not any(np.hypot(x-s[0],y-s[1])<max(r,s[2]) for s in seeds):
                seeds.append((x,y,r,c.reshape(-1,2)))
        gray=cv2.cvtColor(blurred,cv2.COLOR_BGR2GRAY)
        found=[]
        h,w=mask.shape
        for sx,sy,sr,seed_contour in seeds:
            reach=int(min(220,max(45,2.8*sr)))
            x0,y0=max(0,int(sx)-reach),max(0,int(sy)-reach)
            x1,y1=min(w,int(sx)+reach),min(h,int(sy)+reach)
            tile=gray[y0:y1,x0:x1]
            minr=max(7,int(sr*.65));maxr=min(180,int(sr*2.5+12),min(tile.shape)//2)
            if maxr<=minr:continue
            circles=cv2.HoughCircles(tile,cv2.HOUGH_GRADIENT,1.0,8,param1=90,param2=20,minRadius=minr,maxRadius=maxr)
            if circles is None:continue
            gx=cv2.Sobel(tile,cv2.CV_32F,1,0,ksize=3)
            gy=cv2.Sobel(tile,cv2.CV_32F,0,1,ksize=3)
            for cx,cy,r in circles[0][:40]:
                x,y=float(cx+x0),float(cy+y0);r=float(r)
                if np.hypot(x-sx,y-sy)>r*1.1:continue
                containment=np.mean(np.sum((seed_contour-[x,y])**2,axis=1)<(r*1.08)**2)
                if containment<.85:continue
                # Reject circles around an interior pattern when ball-colored pixels
                # belonging to the same local object extend beyond its outline.
                bx0,by0=max(0,int(x-2.0*r)),max(0,int(y-2.0*r))
                bx1,by1=min(w,int(x+2.0*r)+1),min(h,int(y+2.0*r)+1)
                yy_local,xx_local=np.ogrid[by0:by1,bx0:bx1]
                distance2=(xx_local-x)**2+(yy_local-y)**2
                local_color=(mask[by0:by1,bx0:bx1]>0)&(distance2<(2.0*r)**2)
                total=int(local_color.sum())
                outside=int((local_color&(distance2>(1.08*r)**2)).sum())
                if total and outside/total>.06:continue
                if x-r<1 or x+r>=w-1 or y-r<1 or y+r>=h-1:continue
                xx,yy=np.meshgrid(np.arange(max(0,int(x-r)),min(w,int(x+r)+1)),np.arange(max(0,int(y-r)),min(h,int(y+r)+1)))
                disk=(xx-x)**2+(yy-y)**2 <= (.9*r)**2
                if disk.sum()==0:continue
                if np.mean(allowed[yy[disk],xx[disk]]>0)<.98:continue
                color=np.mean(mask[yy[disk],xx[disk]]>0)
                if color<.10:continue
                angles=np.linspace(0,2*np.pi,64,endpoint=False)
                ux,uy=np.cos(angles),np.sin(angles)
                support=np.zeros(64,dtype=bool)
                for offset in (-3,-1,1,3):
                    ex=np.rint(cx+(r+offset)*ux).astype(int);ey=np.rint(cy+(r+offset)*uy).astype(int)
                    valid=(ex>=0)&(ey>=0)&(ex<tile.shape[1])&(ey<tile.shape[0])
                    indices=np.flatnonzero(valid)
                    dx,dy=gx[ey[valid],ex[valid]],gy[ey[valid],ex[valid]]
                    magnitude=np.hypot(dx,dy)
                    alignment=abs(dx*ux[valid]+dy*uy[valid])/np.maximum(magnitude,1)
                    support[indices]|=(magnitude>45)&(alignment>.80)
                coverage=float(support.mean())
                if coverage<.48 or sum(np.mean(g)>.30 for g in np.split(support,4))<3:continue
                score=.80*coverage+.20*min(1,color/.4)
                found.append({'x':x,'y':y,'radius':r,'quality':score,'source':'outline','measured':True})
        return found,mask

    def update(self, frame, allowed, now):
        candidates,mask=self.candidates(frame,allowed)
        if self.history and now-self.history[-1][0]>.30:self.reset()
        chosen=None
        if self.history:
            last=np.asarray(self.history[-1][1:]);dt=now-self.history[-1][0]
            expected=last+self.velocity()*dt
            ranked=[]
            for c in candidates:
                ratio=c['radius']/self.last_radius
                if not .50<ratio<1.9:continue
                distance=np.linalg.norm(np.array([c['x'],c['y']])-expected)
                # Allows fast kicks while still bounding association by time and ball size.
                gate=max(70,self.last_radius*3)+5500*dt
                if distance<=gate:
                    ranked.append((c['quality']-distance/max(gate,1)*.55-.25*abs(np.log(ratio)),c))
            if ranked:chosen=max(ranked,key=lambda a:a[0])[1]
        elif candidates:
            c=max(candidates,key=lambda a:a['quality'])
            if self.pending is not None and np.hypot(c['x']-self.pending['x'],c['y']-self.pending['y'])<max(70,3*c['radius']):
                self.pending_count+=1
            else:self.pending_count=1
            self.pending=c
            if self.pending_count>=3:chosen=c
            self.status=f'Confirming outline {self.pending_count}/3'
        else:
            self.pending=None;self.pending_count=0
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        if chosen is None and self.history:
            chosen=self.flow.estimate(gray,mask,allowed,now)
        if chosen is not None:
            if self.history:
                dt=now-self.history[-1][0]
                expected=np.asarray(self.history[-1][1:])+self.velocity()*dt
                if np.linalg.norm(np.array([chosen['x'],chosen['y']])-expected)>max(20,chosen['radius']*.7):
                    self.history.clear() # Do not average old velocity through a kick/bounce.
                    chosen['motion_reset']=True
            chosen['raw_radius']=chosen['radius']
            if self.history:
                # Smooth small outline-fit noise; keep large/fast movements responsive.
                previous=np.asarray(self.history[-1][1:])
                measured=np.array([chosen['x'],chosen['y']])
                alpha=.45 if np.linalg.norm(measured-previous)<chosen['radius']*.25 else .90
                filtered=previous+alpha*(measured-previous)
                chosen['x'],chosen['y']=map(float,filtered)
                radius_alpha=.35 if abs(chosen['radius']/self.last_radius-1)<.15 else .85
                chosen['radius']=float(self.last_radius+radius_alpha*(chosen['radius']-self.last_radius))
            self.history.append((now,chosen['x'],chosen['y']))
            self.last_radius=chosen['radius']
            self.flow.remember(gray,chosen,now)
            self.status='Measured color + outline' if chosen['source']=='outline' else 'Optical-flow bridge (max 200 ms)'
            return chosen,mask
        if self.history:
            age=now-self.history[-1][0]
            if 0<=age<=.10:
                predicted=np.asarray(self.history[-1][1:])+self.velocity()*age
                x,y=predicted
                if 0<=x<frame.shape[1] and 0<=y<frame.shape[0] and allowed[int(y),int(x)]:
                    self.status=f'Predicted gap {age*1000:.0f} ms'
                    return {'x':float(x),'y':float(y),'radius':self.last_radius,'quality':0,'source':'prediction','measured':False},mask
        if not candidates:self.status='No supported ball outline'
        elif self.history:self.status='Candidate rejected by association'
        return None,mask
