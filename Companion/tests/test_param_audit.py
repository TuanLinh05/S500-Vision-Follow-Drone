import unittest

from s500_companion.param_audit import audit_summary, audit_yaw_test_params


class ParamAuditTests(unittest.TestCase):
    def test_known_safe_profile_has_no_failures(self) -> None:
        values = {
            "COM_RC_OVERRIDE": 3,
            "COM_OBL_RC_ACT": 0,
            "COM_OF_LOSS_T": 1,
            "COM_RCL_EXCEPT": 0,
            "GF_MAX_HOR_DIST": 40,
            "GF_MAX_VER_DIST": 25,
            "GF_ACTION": 2,
            "MPC_XY_VEL_MAX": 2,
            "RC_MAP_RETURN_SW": 6,
            "RC_MAP_KILL_SW": 8,
            "MAV_2_CONFIG": 0,
        }
        result = audit_summary(audit_yaw_test_params(values))
        self.assertTrue(result["ready_for_p7"])
        self.assertEqual(result["counts"]["FAIL"], 0)

    def test_current_style_profile_catches_missing_fence_and_override(self) -> None:
        values = {
            "COM_RC_OVERRIDE": 1,
            "COM_OBL_RC_ACT": 0,
            "COM_OF_LOSS_T": 1,
            "COM_RCL_EXCEPT": 0,
            "GF_MAX_HOR_DIST": 0,
            "GF_MAX_VER_DIST": 0,
            "GF_ACTION": 2,
            "MPC_XY_VEL_MAX": 12,
            "RC_MAP_RETURN_SW": 6,
            "RC_MAP_KILL_SW": 0,
            "MAV_2_CONFIG": 0,
        }
        result = audit_summary(audit_yaw_test_params(values))
        self.assertFalse(result["ready_for_p7"])
        self.assertEqual(result["counts"]["FAIL"], 4)


if __name__ == "__main__":
    unittest.main()

