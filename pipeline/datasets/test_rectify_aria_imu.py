"""Physical-signal tests for channel calibration, timestamp sign and resampling."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from rectify_aria_imu import extract_affine, load_imu_csv, rectify_and_align, write_csv


class RectifyAriaImuTest(unittest.TestCase):
    def test_recovers_physical_signals_with_separate_sensor_delays(self):
        # Delayed sensors read a physical signal at an earlier time. Manufacture
        # raw measurements through independent forward models, then recover the
        # known physical functions at the output timestamp.
        ns = 1_700_000_000_000_000_123 + np.arange(25,dtype=np.int64)*1_000_000
        elapsed = np.arange(25)*.001
        gyro_offset, accel_offset, rate = .00425, .0015, 1000.
        g_delay, a_delay = gyro_offset+.0005, accel_offset+.0005
        gyro_physical = lambda t: np.column_stack((1+2*t, -2+3*t, .5-t))
        accel_physical = lambda t: np.column_stack((4+20*t, 7-15*t, 9.81+5*t))
        matrix_g = np.array([[1.03,.02,0],[.01,.98,.015],[0,-.01,1.02]])
        matrix_a = np.array([[.97,-.03,.01],[0,1.04,.025],[0,0,1.01]])
        bias_g, bias_a = np.array([.02,-.01,.03]), np.array([.4,-.3,.2])
        raw_g = gyro_physical(elapsed-g_delay)@matrix_g.T+bias_g
        raw_a = accel_physical(elapsed-a_delay)@matrix_a.T+bias_a
        rectify_g = lambda value: np.linalg.solve(matrix_g,value-bias_g)
        rectify_a = lambda value: np.linalg.solve(matrix_a,value-bias_a)
        ag,bg,eg = extract_affine(rectify_g,raw_g)
        aa,ba,ea = extract_affine(rectify_a,raw_a)
        out_t,out_g,out_a,keep = rectify_and_align(ns,raw_g,raw_a,(ag,bg),(aa,ba),gyro_offset,accel_offset,rate)
        self.assertLess(max(eg,ea),1e-12)
        np.testing.assert_array_equal(keep,np.arange(3,25))
        np.testing.assert_array_equal(out_t,ns[keep]-4_750_000)
        physical_t=(out_t-ns[0])*1e-9
        np.testing.assert_allclose(out_g,gyro_physical(physical_t),atol=1e-12,rtol=0)
        np.testing.assert_allclose(out_a,accel_physical(physical_t),atol=1e-12,rtol=0)

    def test_drops_end_when_accel_lags_more_and_rejects_backward_input(self):
        ns=np.arange(10,dtype=np.int64)*1_000_000
        values=np.ones((10,3));identity=(np.eye(3),np.zeros(3))
        _,_,_,keep=rectify_and_align(ns,values,values,identity,identity,0.,.003,1000.)
        np.testing.assert_array_equal(keep,np.arange(7))
        ns[5]=ns[4]
        with self.assertRaisesRegex(ValueError,"strictly increasing"):
            rectify_and_align(ns,values,values,identity,identity,0.,0.,1000.)

    def test_integer_timestamp_io_and_overwrite_refusal(self):
        ns=np.array([1_700_000_000_000_000_123,1_700_000_000_001_000_124],dtype=np.int64)
        w=np.array([[.1,.2,.3],[.4,.5,.6]]);a=w+9
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"imu.csv"
            write_csv(path,ns,w,a)
            rt,rw,ra=load_imu_csv(path)
            np.testing.assert_array_equal(rt,ns)
            np.testing.assert_allclose(rw,w);np.testing.assert_allclose(ra,a)
            original=path.read_bytes()
            with self.assertRaises(FileExistsError):
                write_csv(path,ns,w*0,a*0)
            self.assertEqual(path.read_bytes(),original)


if __name__=="__main__":
    unittest.main()
