import json, math, bisect, subprocess
from pathlib import Path
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
R=Path(__file__).parent
LP='/fmu/out/vehicle_local_position_v1'; SP='/fmu/in/trajectory_setpoint'
COLORS={'ROUTE':'#246ce0','ACTIVE_RECOVERY':'#009981','RESERVE_BRAKE':'#dc851c','CORRECTION_BRAKE':'#aa56b7','YAW_ALIGN':'#909097'}
def xy(m):return np.array([m['x'],m['y']],float)
def pose(m):return np.array([m['pose']['position'][k] for k in ['x','y']],float)
def stamp(m):return m['header']['stamp']['sec']*1000000000+m['header']['stamp']['nanosec']
def yaw(m):
 q=m['pose']['orientation'];return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def rot(a):return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
def speed(m):return math.hypot(m['vx'],m['vy'])
def sd(v,a=.3):return v*.25+.5*a*.25**2+(v+a*.25)**2/.2
class Flight:
 def __init__(self,f):
  self.f=f;self.d=json.loads((R/f'{f}.json').read_text());self.ts={k:[x[0] for x in v] for k,v in self.d.items()}
  self.pairs=[];body={};cor={}
  for t,k,m in sorted([(t,k,m) for k in ['/rtabmap/body_pose','/rtabmap/odom_correction'] for t,m in self.d[k]]):
   s=stamp(m);(body if k.endswith('body_pose') else cor)[s]=m
   if s in body and s in cor:self.pairs.append((t,(body[s],cor[s])))
  self.pair_ts=[t for t,_ in self.pairs]
 def at(self,k,t):
  i=bisect.bisect_right(self.ts[k],t)-1
  return None if i<0 else self.d[k][i][1]
 def pair(self,t):return self.pairs[bisect.bisect_right(self.pair_ts,t)-1][1]
 def frame(self,t):
  b,c=self.pair(t);return pose(b),rot(yaw(c))
 def phases(self):
  events=[];land=None;start=None;kill=None
  for t,m in self.d['/rosout']:
   if m['name']!='cpp_flight_adapter':continue
   s=m['msg'];state=None
   if s=='-> ROUTE':start=t;state='ROUTE'
   elif s.startswith('reserve recovery: starting'):state='ACTIVE_RECOVERY'
   elif 'braking until measured capture' in s:state='CORRECTION_BRAKE' if s.startswith('material correction') else 'RESERVE_BRAKE'
   elif s.startswith('active centring stopped:'):state='CORRECTION_BRAKE' if events[-1][1]=='CORRECTION_BRAKE' else 'RESERVE_BRAKE'
   elif 'rejoining route slowly' in s:state='ROUTE'
   elif s.startswith('YAW_ALIGN '):state='YAW_ALIGN'
   elif s.startswith('ROUTE valid'):state='ROUTE'
   elif '-> AUTO.LAND' in s:land=t
   elif s.startswith('KILL KEY'):kill=t
   if state is not None and start is not None and (not events or events[-1][1]!=state):events.append([t,state])
  end=land if land is not None else min(self.ts[LP][-1],self.ts[SP][-1])
  return start,end,land,kill,[[a,events[i+1][0] if i+1<len(events) else end,k] for i,(a,k) in enumerate(events) if a<end]
