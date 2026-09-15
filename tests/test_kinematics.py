import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from hardware.rb5_realsense.kinematics import RB5Kinematics, URDFChain
from hardware.rb5_realsense.devices import RobotReader
from hardware.rb5_realsense.__main__ import require_layout

CONFIG = json.loads(Path("hardware/rb5.example.json").read_text())


class FKTests(unittest.TestCase):
    def test_official_zero_pose_and_base_rotation(self):
        fk = RB5Kinematics(CONFIG["robot"]["kinematics"])
        zero = fk.forward_degrees([0] * 6)
        np.testing.assert_allclose(zero[:3, 3], [0, -.2074, 1.0969], atol=1e-12)
        np.testing.assert_allclose(zero[:3, :3], np.eye(3), atol=1e-12)
        turned = fk.forward_degrees([90, 0, 0, 0, 0, 0])
        np.testing.assert_allclose(turned[:3, 3], [.2074, 0, 1.0969], atol=1e-12)
        np.testing.assert_allclose(turned[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)
        self.assertEqual(fk.metadata["urdf_sha256"], "38c357aaca3ea0ee2284eeb673920579b40ffaed1a40b5b77f02087516487598")

    def test_encoder_mapping_sign_offset_and_invalid_inputs(self):
        config = copy.deepcopy(CONFIG["robot"]["kinematics"])
        baseline = RB5Kinematics(config)
        config["joint_names"] = list(reversed(config["joint_names"]))
        config["joint_signs"] = [-1] * 6
        config["joint_offsets_deg"] = [10] * 6
        fk = RB5Kinematics(config)
        q = np.arange(6) * 10.
        np.testing.assert_allclose(fk.forward_degrees(q), baseline.forward_degrees((-q+10)[::-1]), atol=1e-12)
        for bad in ([1, 2], [0]*5 + [np.nan]):
            with self.assertRaises(ValueError):
                fk.forward_degrees(bad)
        config["joint_names"][0] = "unknown"
        with self.assertRaises(ValueError):
            RB5Kinematics(config)

    def test_urdf_origin_axis_fixed_tip_and_rejected_unsupported_models(self):
        xml = '''<robot name="test"><link name="base"/><link name="arm"/><link name="tip"/>
        <joint name="q" type="revolute"><parent link="base"/><child link="arm"/>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="0 1 0"/></joint>
        <joint name="tool" type="fixed"><parent link="arm"/><child link="tip"/>
        <origin xyz="1 0 0"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "robot.urdf"
            path.write_text(xml)
            chain = URDFChain(path, "base", "tip")
            np.testing.assert_allclose(chain.forward({"q": 0})[:3, 3], [1, 3, 3], atol=1e-12)
            np.testing.assert_allclose(chain.forward({"q": np.pi/2})[:3, 3], [1, 2, 2], atol=1e-12)
            with self.assertRaises(ValueError):
                URDFChain(path, "tip", "base")
            with self.assertRaises(ValueError):
                chain.forward({})
            path.write_text(xml.replace('type="revolute"', 'type="floating"'))
            with self.assertRaises(ValueError):
                URDFChain(path, "base", "tip")

    def test_matches_independent_orocos_kdl(self):
        try:
            import PyKDL as kdl
        except ImportError:
            self.skipTest("Optional independent reference: python3-pykdl")
        cfg = CONFIG["robot"]["kinematics"]
        fk = RB5Kinematics(cfg)
        root = ET.parse(fk.chain.path).getroot()
        chain = kdl.Chain()
        # Independent KDL engine, model origins/axes read directly from vendor XML.
        for name in cfg["joint_names"] + ["tcp_joint"]:
            joint = next(j for j in root.findall("joint") if j.get("name") == name)
            origin = joint.find("origin")
            xyz = list(map(float, origin.get("xyz", "0 0 0").split()))
            rpy = list(map(float, origin.get("rpy", "0 0 0").split()))
            frame = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
            chain.addSegment(kdl.Segment(name + "_origin", kdl.Joint(kdl.Joint.Fixed), frame))
            if joint.get("type") != "fixed":
                axis = list(map(float, joint.find("axis").get("xyz").split()))
                chain.addSegment(kdl.Segment(name, kdl.Joint(name, kdl.Vector.Zero(),
                    kdl.Vector(*axis), kdl.Joint.RotAxis)))
        solver = kdl.ChainFkSolverPos_recursive(chain)
        rng = np.random.default_rng(850)
        for q in rng.uniform(-170, 170, (50, 6)):
            angles = kdl.JntArray(6)
            for i in range(6):
                angles[i] = np.deg2rad(q[i])
            frame = kdl.Frame()
            self.assertEqual(solver.JntToCart(angles, frame), 0)
            expected = np.eye(4)
            expected[:3, :3] = [[frame.M[i, j] for j in range(3)] for i in range(3)]
            expected[:3, 3] = [frame.p[i] for i in range(3)]
            np.testing.assert_allclose(fk.forward_degrees(q), expected, atol=1e-12)

    def test_robot_reader_uses_encoder_angles_not_tcp_reference(self):
        reader = RobotReader.__new__(RobotReader)
        reader.fk = RB5Kinematics(CONFIG["robot"]["kinematics"])
        reader.timeout = 1
        reader.last_controller_time = None
        state = SimpleNamespace(time=1., robot_state=1, real_vs_simulation_mode=0,
                                tcp_pos=np.array([10000, 10000, 10000, 0, 0, 0]),
                                jnt_ang=np.zeros(6))
        reader.channel = SimpleNamespace(request_data=lambda timeout: SimpleNamespace(sdata=state))
        sample = reader.read()
        np.testing.assert_allclose(sample["T"][:3, 3], [0, -.2074, 1.0969])
        self.assertEqual(sample["pose_source"], "urdf_fk")
        self.assertEqual(sample["tcp_mm_deg"][0], 10000)

    def test_unconfigured_layout_cannot_start_calibration(self):
        with self.assertRaisesRegex(ValueError, "FK alone cannot"):
            require_layout({**CONFIG, "mount": "unconfigured"})
        for mount in ("eye_to_hand", "eye_in_hand"):
            require_layout({**CONFIG, "mount": mount})


if __name__ == "__main__":
    unittest.main()
