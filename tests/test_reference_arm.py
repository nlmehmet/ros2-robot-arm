"""Offline and optional xacro/ROS-host checks for the reference robot model."""
import math
from pathlib import Path
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET

MODEL = Path(__file__).resolve().parents[1] / "src/robot_arm_description/urdf/reference_arm.urdf.xacro"
X = "{http://www.ros.org/wiki/xacro}"


class ReferenceArmTests(unittest.TestCase):
    def test_six_joint_chain_and_positive_limits(self):
        root = ET.parse(MODEL).getroot()
        self.assertEqual(root.get("name"), "reference_arm")
        self.assertIsNotNone(root.find(X + 'macro[@name="cyl_link"]'))
        links = root.findall(X + "cyl_link")
        self.assertEqual([l.get("name") for l in links],
                         ["base_link"] + [f"link_{n}" for n in range(1, 7)])
        joints = root.findall("joint")
        self.assertEqual([j.get("name") for j in joints],
                         [f"joint_{n}" for n in range(1, 7)])
        axes = ("0 0 1", "0 1 0", "0 1 0", "1 0 0", "0 1 0", "0 0 1")
        for i, joint in enumerate(joints):
            with self.subTest(joint=joint.get("name")):
                self.assertEqual(joint.get("type"), "revolute")
                self.assertEqual(joint.find("parent").get("link"),
                                 "base_link" if i == 0 else f"link_{i}")
                self.assertEqual(joint.find("child").get("link"), f"link_{i+1}")
                self.assertEqual(joint.find("axis").get("xyz"), axes[i])
                limit = joint.find("limit")
                self.assertLess(float(limit.get("lower")), float(limit.get("upper")))
                self.assertGreater(float(limit.get("effort")), 0)
                self.assertGreater(float(limit.get("velocity")), 0)

    @unittest.skipUnless(shutil.which("xacro"), "xacro unavailable; check on the Ubuntu ROS VM")
    def test_expansion_and_physical_properties(self):
        process = subprocess.run(["xacro", str(MODEL)], capture_output=True,
                                 check=True, text=True, timeout=20)
        root = ET.fromstring(process.stdout)
        links = root.findall("link")
        joints = root.findall("joint")
        self.assertEqual(len(links), 7)
        self.assertEqual(len(joints), 6)
        names = {link.get("name") for link in links}
        self.assertEqual({joint.find("child").get("link") for joint in joints}, names - {"base_link"})
        for link in links:
            with self.subTest(link=link.get("name")):
                self.assertIsNotNone(link.find("visual/geometry/cylinder"))
                self.assertIsNotNone(link.find("collision/geometry/cylinder"))
                self.assertGreater(float(link.find("inertial/mass").get("value")), 0)
                inertia = link.find("inertial/inertia")
                values = [float(inertia.get(k)) for k in ("ixx", "iyy", "izz")]
                self.assertTrue(all(math.isfinite(v) and v > 0 for v in values))


if __name__ == "__main__":
    unittest.main()
