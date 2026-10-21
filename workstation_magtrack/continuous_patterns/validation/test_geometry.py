"""Regression fixtures for frame composition, C3 joins and vector bounds."""
import copy
import json
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import generate
from src import trajectory, clearance


class GeometryContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT/"config/contract_v1.json").read_text())
        cls.geometry = generate.read_geometry()
        cls.base = generate.new_geometry("woven_lissajous", cls.config["seed"]+1000, 0,
            "fixture", {"mode":"sweep", "lower_m":.075,"upper_m":.2})
        generate.select_cycles(cls.base, cls.config)
        cls.meta = generate.timed_variant(cls.base, cls.config,"fixture","V2","fixed")

    def test_controller_base_is_composed_with_urdf_base_link(self):
        t,p,q,_ = generate.arrays(self.meta,self.config)
        result = clearance.precheck(self.meta,p,q,t,self.geometry,self.config)
        self.assertEqual(result["status"],"passed_rod_subset_only")
        # Omitting the fixed Rz(pi) gives a reproducible false wall rejection.
        wrong = copy.deepcopy(self.geometry)
        wrong["base_link_to_base"]["rpy_rad"] = [0.,0.,0.]
        bad = clearance.precheck(self.meta,p,q,t,wrong,self.config)
        self.assertTrue(any(x["status"]=="rejected" for x in bad["external_cuboids"]))

    def test_70mm_rejected_without_raising_its_height(self):
        meta = copy.deepcopy(self.meta)
        meta["height_profile"]["lower_m"] = meta["min_height_m"] = .070
        result = clearance.rod_board_envelope(meta,self.geometry,self.config)
        self.assertEqual(result["status"],"rejected")
        self.assertEqual(meta["min_height_m"],.070)

    def test_height_knots_have_c3_joins(self):
        knots,_ = trajectory.height_segments(self.meta)
        for d in (1,2,3):
            values = trajectory.height_derivatives(self.meta,knots)[d]
            np.testing.assert_allclose(values,0.,atol=1e-6)

    def test_position_chain_rule_orders_1_to_4(self):
        s = np.array([.1234567,.3123456,.4567891,.6789123,.8543219])
        delta = 1e-7
        p = trajectory.position_derivatives(self.meta,s)
        plus = trajectory.position_derivatives(self.meta,s+delta)
        minus = trajectory.position_derivatives(self.meta,s-delta)
        for d in range(1,5):
            err = np.max(np.abs((plus[d-1]-minus[d-1])/(2*delta)-p[d]))
            self.assertLess(err/max(1.,np.max(np.abs(p[d]))),1e-6)

    def test_analytic_rest_and_time_reversal(self):
        s = np.array([0.,1.])
        for value in trajectory.position_derivatives(self.meta,s)[1:4]:
            np.testing.assert_array_equal(value,0.)
        reverse = copy.deepcopy(self.meta)
        reverse["direction"] = "reverse"
        probe = np.linspace(.1,.9,17)
        p = trajectory.position_derivatives(self.meta,1.-probe)
        r = trajectory.position_derivatives(reverse,probe)
        for d in range(5):
            np.testing.assert_allclose(r[d],(-1.)**d*p[d],atol=1e-10)


if __name__ == "__main__":
    unittest.main()
