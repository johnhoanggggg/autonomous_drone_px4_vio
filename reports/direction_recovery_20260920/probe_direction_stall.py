import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path.cwd()/'ros2_ws/src/px4_vio_bridge/test'))
from test_fault_hold_integration import *
a=Adapter(Path(tempfile.mkdtemp(prefix='direction_stall_')),{'active_recovery_enabled':True,'fault_stop_deceleration':.1,'route_governor_response_acceleration':.3,'route_correction_uncertainty_reserve':.05,'route_governor_overspeed_tolerance':.01,'command_acceleration':.15})
try:
 a.north=.40;a.path_y=.49;a.displacement_north=0.;a.displacement_east=.5
 a.wait(lambda:any(m.data.startswith('ROUTE') for m in a.status))
 a.armed=True
 a.run(2.5)
 print(a.log_path.read_text())
 print('\n'.join(m.data for m in a.status[-16:]))
finally:a.close()
