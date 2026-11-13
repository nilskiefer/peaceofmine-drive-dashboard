"""Simple throttle PI and two-wheel speed feedback, independent of ROS I/O."""
from collections import deque
import math

DEFAULTS = dict(mode='open_loop', kp=20., ki=5., max_speed_mps=.8, max_output_percent=50.)
FEEDBACK_TIMEOUT = .5
SPEED_WINDOW = .2
MAX_WHEEL_SPEED = 5.  # Above the vehicle's physical speed, even in high gear.
NO_MOTION_TIMEOUT = 3.


def validate(value):
    if not isinstance(value, dict) or value.get('mode') not in ('open_loop','closed_loop'):
        raise ValueError('Choose open loop or closed loop (PI)')
    for key, low, high in (('kp',0.,1000.),('ki',0.,1000.),('max_speed_mps',.01,1.7),('max_output_percent',1.,90.)):
        number=value.get(key)
        if type(number) not in (int,float) or not math.isfinite(number) or not low <= number <= high:
            raise ValueError(f'{key} must be a finite number from {low:g} to {high:g}')
    return dict(mode=value['mode'],**{k:float(value[k]) for k in DEFAULTS if k!='mode'})


class WheelFeedback:
    """Differentiate cumulative metres over a short window; use the higher wheel."""
    def __init__(self):
        self.history=deque()
        self.last_stamp=None
        self.received=None
        self.speeds=[None,None]
        self.issue=None

    def distance(self,stamp_ns,values,now,age_s=0.):
        if not isinstance(stamp_ns,int) or stamp_ns <= 0 or not math.isfinite(age_s) or not -.05 <= age_s < FEEDBACK_TIMEOUT:
            self.issue='Encoder timestamp is invalid or stale'
            return
        if stamp_ns == self.last_stamp:
            return  # Replayed packets must not renew freshness.
        if self.last_stamp is not None and stamp_ns < self.last_stamp:
            self.history.clear()
            self.speeds=[None,None]
        self.last_stamp=stamp_ns
        distances=[float(values[i]) if i<len(values) and math.isfinite(values[i]) else None for i in range(2)]
        self.received=now-max(0.,age_s)
        self.history.append((stamp_ns,distances))
        while len(self.history)>2 and (stamp_ns-self.history[1][0])/1e9 >= SPEED_WINDOW:
            self.history.popleft()
        first,baseline=self.history[0]
        dt=(stamp_ns-first)/1e9
        self.speeds=[None,None]
        self.issue='Waiting for a second encoder sample'
        if not .05 <= dt <= FEEDBACK_TIMEOUT:
            if dt>FEEDBACK_TIMEOUT:
                self.history.clear();self.history.append((stamp_ns,distances))
            return
        self.issue=None
        for wheel,(before,after) in enumerate(zip(baseline,distances)):
            if before is not None and after is not None:
                speed=abs(after-before)/dt
                if math.isfinite(speed) and speed <= MAX_WHEEL_SPEED:
                    self.speeds[wheel]=speed

    def snapshot(self,now):
        age=None if self.received is None else max(0.,now-self.received)
        left,right=self.speeds
        valid=[(speed,index) for index,speed in enumerate(self.speeds) if speed is not None]
        selected=max(valid,key=lambda item:item[0])[1] if valid else None
        fresh=age is not None and age<FEEDBACK_TIMEOUT and selected is not None and self.issue is None
        if age is None:
            health,reason='missing','No wheel encoder messages'
        elif age>=FEEDBACK_TIMEOUT:
            health,reason='stale','Wheel encoder messages are stale'
        elif not fresh:
            health,reason='invalid',self.issue or 'No usable wheel speeds; check encoder counters'
        elif len(valid)<2:
            health,reason='disagreement','One wheel speed is unavailable; using the other wheel'
        elif abs(left-right)>max(.05,max(left,right)*.5):
            health,reason='disagreement','Wheel speeds disagree; using the higher reading'
        else:
            health,reason='ok','Wheel speeds agree'
        return dict(fresh=fresh,health=health,reason=reason,left_mps=left if fresh else None,
            right_mps=right if fresh else None,selected=('left' if selected==0 else 'right') if fresh else None,
            measured_mps=self.speeds[selected] if fresh else None,age_s=age)


class SpeedController:
    def __init__(self,options=None):
        self.options=validate(DEFAULTS if options is None else options)
        self.feedback=WheelFeedback()
        self.integral=0.
        self.last_time=None
        self.direction=0
        self.neutral_required=False
        self.fault=None
        self.no_motion_since=None

    def configure(self,options):
        options=validate(options)
        if options != self.options:
            self.options=options
            self.integral=0.
            self.neutral_required=True
            self.fault=None
            self.no_motion_since=None

    def step(self,target,now,enabled=True,reason='',speed_scale=1.7):
        encoder=self.feedback.snapshot(now)
        dt=0. if self.last_time is None else max(0.,min(.2,now-self.last_time))
        self.last_time=now
        target=max(-self.options['max_speed_mps'],min(self.options['max_speed_mps'],target))
        state='open_loop' if self.options['mode']=='open_loop' else 'closed_loop'
        throttle=p=0.
        limited=False
        if not enabled or abs(target)<1e-6:
            self.integral=0.
            self.direction=0
            self.no_motion_since=None
            if enabled:
                self.fault=None
                self.neutral_required=False
            state='stopped' if enabled else reason or 'disarmed'
        elif self.neutral_required:
            state,reason='neutral_required','Release drive to neutral before starting again'
        else:
            direction=1 if target>0 else -1
            if direction!=self.direction:
                self.integral=0.
                self.no_motion_since=None
            self.direction=direction
            limit=self.options['max_output_percent']
            base=abs(target)/speed_scale*100.
            if self.options['mode']=='closed_loop':
                if not encoder['fresh']:
                    self.fault=encoder['reason']
                elif self.fault is None:
                    if encoder['measured_mps']<.005:
                        if self.no_motion_since is None:self.no_motion_since=now
                        if now-self.no_motion_since>=NO_MOTION_TIMEOUT:
                            self.fault='No encoder movement for 3 s; release drive and check wheels/encoders'
                            encoder.update(health='no_motion',reason=self.fault)
                    else:
                        self.no_motion_since=None
                if self.fault is None:
                    error=abs(target)-encoder['measured_mps']
                    p=self.options['kp']*error
                    candidate=max(-limit,min(limit,self.integral+self.options['ki']*error*dt))
                    raw=base+p+candidate
                    # Anti-windup: don't accumulate error further into saturation.
                    if not (raw>limit and error>0 or raw<0 and error<0):
                        self.integral=candidate
                    raw=base+p+self.integral
                    throttle=direction*max(0.,min(limit,raw))
                    limited=raw<0 or raw>limit
                else:
                    self.integral=0.
                    state,reason='encoder_fault',self.fault+'; release drive to retry or select open loop'
                    if self.fault.startswith('No encoder movement'):
                        encoder.update(health='no_motion',reason=self.fault)
            else:
                self.integral=0.
                throttle=direction*min(limit,base)
                limited=base>limit
        return dict(available=True,mode=self.options['mode'],state=state,reason=reason,
            target_mps=target,measured_mps=encoder['measured_mps'],throttle_percent=throttle,
            p_percent=p,i_percent=self.integral,output_limited=limited,encoder=encoder)
