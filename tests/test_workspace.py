"""Offline package contract checks; run from any working directory."""
import ast
import configparser
import os
from pathlib import Path
import re
import runpy
import sys
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

try:
    from catkin_pkg.package import parse_package
except ImportError:
    parse_package = None

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    'robot_arm_description': 'ament_cmake',
    'robot_arm_bringup': 'ament_cmake',
    'robot_arm_control': 'ament_cmake',
    'robot_arm_teleop': 'ament_python',
    'robot_arm_interfaces': 'ament_cmake',
}


class WorkspaceContracts(unittest.TestCase):
    def setUp(self):
        self.manifests = {
            p.parent.name: ET.parse(p).getroot()
            for p in (ROOT / 'src').glob('*/package.xml')
        }

    def test_all_expected_packages_exist_and_are_unique(self):
        self.assertEqual(set(self.manifests), set(EXPECTED))
        names = [m.findtext('name') for m in self.manifests.values()]
        self.assertEqual(len(names), len(set(names)))
        for directory, manifest in self.manifests.items():
            self.assertEqual(manifest.findtext('name'), directory)

    def test_manifests_have_valid_metadata_and_build_types(self):
        for name, manifest in self.manifests.items():
            with self.subTest(package=name):
                self.assertEqual(manifest.attrib['format'], '3')
                self.assertRegex(manifest.findtext('version', ''), r'^\d+\.\d+\.\d+$')
                self.assertTrue(manifest.findtext('description', '').strip())
                self.assertTrue(manifest.findtext('license', '').strip())
                self.assertIn('@', manifest.find('maintainer').attrib['email'])
                build = manifest.findtext('export/build_type')
                self.assertEqual(build, EXPECTED[name])
                self.assertIn(build, [d.text for d in manifest.findall('buildtool_depend')])

    @unittest.skipIf(parse_package is None, 'catkin_pkg unavailable; run on Ubuntu with ROS development tools')
    def test_manifests_with_ros_package_parser(self):
        for name in EXPECTED:
            with self.subTest(package=name):
                package = parse_package(str(ROOT / 'src' / name / 'package.xml'))
                package.validate()

    def test_dependency_declarations_have_valid_names_and_no_duplicates(self):
        for name, manifest in self.manifests.items():
            seen = set()
            for element in manifest:
                if not element.tag.endswith('depend'):
                    continue
                with self.subTest(package=name, dependency=element.text):
                    # Dependencies may be rosdep keys such as python3-setuptools,
                    # whose syntax is broader than a ROS package's own name.
                    self.assertRegex(element.text or '', r'^[a-z][a-z0-9_-]*$')
                    declaration = (element.tag, element.text)
                    self.assertNotIn(declaration, seen)
                    seen.add(declaration)
        teleop_runtime = {e.text for e in self.manifests['robot_arm_teleop'].findall('exec_depend')}
        self.assertTrue({'python3-setuptools', 'rclpy', 'sensor_msgs', 'trajectory_msgs'} <= teleop_runtime)

    def test_internal_dependencies_resolve_and_are_acyclic(self):
        graph = {}
        for name, manifest in self.manifests.items():
            deps = [e.text for e in manifest if e.tag.endswith('depend')]
            internal = {d for d in deps if d.startswith('robot_arm_')}
            self.assertTrue(internal <= set(EXPECTED), f'{name}: {internal}')
            self.assertNotIn(name, deps)
            graph[name] = internal
        def visit(name, path):
            self.assertNotIn(name, path, f'cyclic dependencies: {path + [name]}')
            for dependency in graph[name]:
                visit(dependency, path + [name])
        for name in graph:
            visit(name, [])
        self.assertEqual(graph['robot_arm_bringup'], {'robot_arm_description', 'robot_arm_control'})
        self.assertFalse(graph['robot_arm_description'])

    def test_cmake_packages_register_and_install_existing_asset_directories(self):
        for name, build in EXPECTED.items():
            if build != 'ament_cmake':
                continue
            base = ROOT / 'src' / name
            text = (base / 'CMakeLists.txt').read_text()
            with self.subTest(package=name):
                self.assertRegex(text, rf'project\({name}\)')
                self.assertIn('find_package(ament_cmake REQUIRED)', text)
                self.assertIn('ament_package()', text)
                install = re.search(r'install\(DIRECTORY\s+(.*?)\s+DESTINATION share/\$\{PROJECT_NAME\}\s*\)', text, re.S)
                self.assertIsNotNone(install)
                for directory in install.group(1).split():
                    self.assertTrue((base / directory).is_dir(), directory)
                expected_assets = {
                    'robot_arm_description': {'urdf', 'meshes', 'rviz'},
                    'robot_arm_bringup': {'launch', 'config'},
                    'robot_arm_control': {'config'},
                    'robot_arm_interfaces': {'msg', 'srv', 'action'},
                }
                self.assertEqual(set(install.group(1).split()), expected_assets[name])

    def test_python_setup_registers_resources_and_existing_files(self):
        name = 'robot_arm_teleop'
        base = ROOT / 'src' / name
        captured = {}
        fake = types.ModuleType('setuptools')
        fake.setup = lambda **kwargs: captured.update(kwargs)
        fake.find_packages = lambda **kwargs: [name]
        previous = Path.cwd()
        try:
            os.chdir(base)
            with patch.dict(sys.modules, {'setuptools': fake}):
                runpy.run_path(str(base / 'setup.py'), run_name='__main__')
        finally:
            os.chdir(previous)
        self.assertEqual(captured['name'], name)
        self.assertEqual(captured['version'], self.manifests[name].findtext('version'))
        self.assertEqual(captured['license'], self.manifests[name].findtext('license'))
        self.assertEqual(captured['maintainer_email'], self.manifests[name].find('maintainer').attrib['email'])
        self.assertIn('setuptools', captured['install_requires'])
        self.assertEqual(captured['entry_points']['console_scripts'], [])
        installed = dict(captured['data_files'])
        self.assertEqual(installed['share/ament_index/resource_index/packages'], ['resource/' + name])
        self.assertIn('package.xml', installed['share/' + name])
        for _, files in captured['data_files']:
            for file in files:
                self.assertTrue((base / file).is_file(), file)
        self.assertIn('script_dir=$base/lib/' + name, (base / 'setup.cfg').read_text())
        self.assertIn('install_scripts=$base/lib/' + name, (base / 'setup.cfg').read_text())
        self.assertTrue((base / name / '__init__.py').is_file())
        config = configparser.ConfigParser(interpolation=None)
        config.read(base / 'setup.cfg')
        self.assertEqual(config['develop']['script_dir'], '$base/lib/' + name)
        self.assertEqual(config['install']['install_scripts'], '$base/lib/' + name)

    def test_control_requires_portable_cpp17(self):
        cmake = (ROOT / 'src/robot_arm_control/CMakeLists.txt').read_text()
        self.assertIn('set(CMAKE_CXX_STANDARD 17)', cmake)
        self.assertIn('set(CMAKE_CXX_STANDARD_REQUIRED ON)', cmake)
        self.assertIn('set(CMAKE_CXX_EXTENSIONS OFF)', cmake)

    def test_python_sources_parse(self):
        files = list((ROOT / 'src').rglob('*.py'))
        self.assertTrue(files)
        for file in files:
            with self.subTest(file=str(file.relative_to(ROOT))):
                ast.parse(file.read_text(), filename=str(file))

    def test_interface_package_declares_rosidl_contract(self):
        manifest = self.manifests['robot_arm_interfaces']
        self.assertIn('rosidl_default_generators', [e.text for e in manifest.findall('buildtool_depend')])
        self.assertIn('rosidl_default_runtime', [e.text for e in manifest.findall('exec_depend')])
        self.assertEqual(manifest.findtext('member_of_group'), 'rosidl_interface_packages')
        for directory in ('msg', 'srv', 'action'):
            self.assertTrue((ROOT / 'src/robot_arm_interfaces' / directory).is_dir())

    def test_documentation_links_and_workspace_ignore_rules(self):
        for relative in ('README.md', 'docs/architecture.md', 'docs/development_plan.md', 'docs/ubuntu_setup.md', 'AGENTS.md'):
            file = ROOT / relative
            self.assertTrue(file.is_file())
            for link in re.findall(r'\]\(([^)]+)\)', file.read_text()):
                if '://' not in link and not link.startswith('#'):
                    self.assertTrue((file.parent / link.split('#')[0]).exists(), link)
        self.assertIn('```mermaid', (ROOT / 'docs/architecture.md').read_text())
        rules = (ROOT / '.gitignore').read_text().splitlines()
        for directory in ('/build/', '/install/', '/log/'):
            self.assertIn(directory, rules)

    def test_documentation_targets_humble_and_fortress(self):
        for relative in ('AGENTS.md', 'README.md', 'docs/architecture.md',
                         'docs/development_plan.md', 'docs/ubuntu_setup.md'):
            text = (ROOT / relative).read_text()
            with self.subTest(file=relative):
                self.assertIn('Humble', text)
                self.assertIn('Fortress', text)
                self.assertNotIn('Gazebo Classic 11', text)
                self.assertNotIn('gazebo_ros2_control', text)
                self.assertNotIn('ros-humble-gazebo-', text)
        self.assertIn('Apache License', (ROOT / 'LICENSE').read_text())


if __name__ == '__main__':
    unittest.main()
