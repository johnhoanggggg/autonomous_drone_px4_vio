import sys,tempfile,math
from pathlib import Path
sys.path.insert(0,str(Path.cwd()/'ros2_ws/src/px4_vio_bridge/test'))
from test_fault_hold_integration import *
a=active_adapter(Path(tempfile.mkdtemp(prefix='entry1337_')), command_acceleration=.15)
try:
 a.run(.6)
 a.vn=0.;a.path_y=.35
 a.wait(lambda:a.status[-1].data.startswith('ROUTE valid'),timeout=2.)
 old=a.sp[-1]; print('old',list(old.position[:2]),list(old.velocity[:2]),a.status[-1].data)
 start=len(a.status);i=len(a.sp)
 a.vn=.08
 a.wait(lambda:any(m.data.startswith('ACTIVE_RECOVERY') for m in a.status[start:]),timeout=1.)
 for m in a.sp[max(0,i-1):]:print('sp',list(m.position[:2]),list(m.velocity[:2]))
 print(a.log_path.read_text())
finally:a.close()
