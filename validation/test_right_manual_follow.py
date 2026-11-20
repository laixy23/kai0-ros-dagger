"""Actual-feedback manual-follow fake-plant tests; no SDK/CAN interface created."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).parent))
import run_right_manual_follow as module
from check_right_alignment import REQUIRED_IDS


class Plant:
    def __init__(self):
        self.t=1000.0
        self.master=[-9000,7000,-1300,-1000,30000,7000]
        self.slave=[-5000,7200,-1400,-1900,34500,7600]
        self.master_mode=2
        self.master_enabled=True
        self.calls=[]
        self.owners=[]
        self.stuck=False
        self.stale=False
        self.writes=0

    def clock(self): return self.t
    def pause(self,span): self.t+=span

    def read(self,channel):
        self.t+=.03
        ages={f"0x{k:03X}":.001 for k in REQUIRED_IDS}
        if self.stale and self.writes and channel==module.MASTER: ages['0x2A6']=.5
        return {'ctrl_mode':self.master_mode if channel==module.MASTER else 1,
                'arm_status':0,'err_code':0,'gripper_mode':0,
                'joint_mdeg':list(self.master if channel==module.MASTER else self.slave),
                'driver_enabled':[self.master_enabled if channel==module.MASTER else True]*6,
                'timestamp_sec':self.t,'component_age_s':ages}

    def factory(self,channel):
        if channel!=module.SLAVE: raise AssertionError('Master SDK must never be constructed')
        self.calls.append(('construct',channel))
        return self
    def ConnectPort(self,**kwargs): self.calls.append(('ConnectPort',kwargs))
    def DisconnectPort(self): self.calls.append(('DisconnectPort',))
    def MotionCtrl_2(self,**kwargs): self.calls.append(('MotionCtrl_2',kwargs))
    def JointCtrl(self,*target):
        self.calls.append(('JointCtrl',target)); self.writes+=1
        if not self.stuck: self.slave[:]=target
    def run(self,**kwargs):
        return module.follow(duration=2,reader=self.read,sdk_factory=self.factory,
                             controller_reader=lambda:self.owners,clock=self.clock,
                             wall=self.clock,pause=self.pause,**kwargs)


class ActualFeedbackFollowTests(unittest.TestCase):
    def test_default_passively_reads_both_and_never_constructs_sdk(self):
        p=Plant(); r=p.run()
        self.assertTrue(r['passed']); self.assertEqual(p.calls,[])

    def test_outside_j3_rejects_before_any_sdk_or_preparation(self):
        p=Plant();p.master[2]=1371
        r=p.run(apply=True,feedback_master=True,confirm_supported=True)
        self.assertFalse(r['passed']);self.assertIn('J3=1371',r['error']);self.assertEqual(p.calls,[])

    def test_master_must_already_be_manual_and_enabled_no_mode_repair(self):
        for mode,enabled in ((0,False),(1,True),(2,False)):
            p=Plant();p.master_mode=mode;p.master_enabled=enabled
            r=p.run(apply=True,feedback_master=True,confirm_supported=True)
            self.assertFalse(r['passed']);self.assertEqual(p.calls,[])

    def test_only_slave_sdk_writes_with_bounded_targets_and_actual_errors(self):
        p=Plant();rows=[]
        r=p.run(apply=True,feedback_master=True,confirm_supported=True,emit=rows.append)
        self.assertTrue(r['passed'],r.get('error'))
        self.assertEqual(p.calls[0],('construct',module.SLAVE))
        self.assertTrue(all(x[0] in ('construct','ConnectPort','DisconnectPort','MotionCtrl_2','JointCtrl') for x in p.calls))
        commands=[c for c in r['commands'] if c['stage']=='follow']
        self.assertTrue(commands)
        for before,after in zip(commands,commands[1:]):
            dt=after['monotonic_sec']-before['monotonic_sec']
            delta=max(abs(a-b) for a,b in zip(after['target_mdeg'],before['target_mdeg']))
            self.assertLessEqual(delta,1000);self.assertLessEqual(delta/dt,5000+1e-6)
        self.assertTrue(r['all_six_aligned']);self.assertTrue(r['exit_slave_hold_requested'])
        self.assertEqual(r['master_control_writes'],0)
        self.assertEqual(len(r['final_sample']['master_minus_slave_deg']),6)
        modes=[x[1] for x in p.calls if x[0]=='MotionCtrl_2']
        self.assertTrue(all(x==dict(ctrl_mode=1,move_mode=1,move_spd_rate_ctrl=10,is_mit_mode=0) for x in modes))

    def test_stuck_slave_stops_on_measured_published_tracking_error(self):
        p=Plant();p.stuck=True
        r=p.run(apply=True,feedback_master=True,confirm_supported=True)
        self.assertFalse(r['passed']);self.assertIn('tracking error',r['error'])
        self.assertTrue(r['exit_slave_hold_requested'])
        for c in r['commands']:
            self.assertLessEqual(max(abs(a-b) for a,b in zip(c['target_mdeg'],[-5000,7200,-1400,-1900,34500,7600])),5000)

    def test_stale_master_group_stops_and_holds_only_fresh_slave_actual(self):
        p=Plant();p.stale=True
        r=p.run(apply=True,feedback_master=True,confirm_supported=True)
        self.assertFalse(r['passed']);self.assertIn('raw CAN component',r['error'])
        self.assertTrue(r['exit_slave_hold_requested'])
        self.assertEqual(r['commands'][-1]['stage'],'exit_measured_hold')

    def test_existing_controller_and_stop_request_never_construct_sdk(self):
        p=Plant();p.owners=[{'pid':10}]
        r=p.run(apply=True,feedback_master=True,confirm_supported=True)
        self.assertFalse(r['passed']);self.assertEqual(p.calls,[])
        p.owners=[]
        r=p.run(apply=True,feedback_master=True,confirm_supported=True,stop_requested=lambda:True)
        self.assertFalse(r['passed']);self.assertEqual(p.calls,[])

    def test_support_and_feedback_selection_required(self):
        for supported,feedback in ((False,False),(True,False),(False,True)):
            p=Plant();r=p.run(apply=True,confirm_supported=supported,feedback_master=feedback)
            self.assertFalse(r['passed']);self.assertEqual(p.calls,[])

    def test_stop_request_only_finishes_with_slave_measured_hold(self):
        p=Plant()
        r=p.run(apply=True,feedback_master=True,confirm_supported=True,stop_requested=lambda:p.writes>=3)
        self.assertTrue(r['passed']);self.assertEqual(r['stop_reason'],'operator_stop')
        self.assertEqual(r['commands'][-1]['stage'],'exit_measured_hold')
        self.assertEqual(r['master_control_writes'],0)

    def test_initial_45_degree_limit_blocks_large_catchup(self):
        p=Plant();p.master[2]=-50000
        r=p.run(apply=True,feedback_master=True,confirm_supported=True)
        self.assertFalse(r['passed']);self.assertIn('45 degrees',r['error']);self.assertEqual(p.calls,[])


if __name__=='__main__':unittest.main()
