"""
URDF Loader with Parameterization

Implements URDF loading functionality with parameterization to support sim-to-real transfer.
"""

import os
import xml.etree.ElementTree as ET
from typing import Dict, Any, Optional, List
from dataclasses import dataclass


@dataclass
class JointLimits:
    """Joint limits for a robot joint."""
    lower: float
    upper: float
    velocity: float
    effort: float


@dataclass
class RobotJoint:
    """Represents a robot joint with its properties."""
    name: str
    joint_type: str  # 'revolute', 'prismatic', 'continuous', 'fixed', etc.
    limits: Optional[JointLimits]
    parent_link: str
    child_link: str
    origin_xyz: List[float]  # [x, y, z]
    origin_rpy: List[float]  # [roll, pitch, yaw]


@dataclass
class RobotLink:
    """Represents a robot link with its properties."""
    name: str
    mass: float
    inertia: List[float]  # [ixx, ixy, ixz, iyy, iyz, izz]
    visual_mesh: Optional[str]
    collision_mesh: Optional[str]


class URDFLoader:
    """
    URDF loader with parameterization capabilities for sim-to-real transfer.
    """

    def __init__(self):
        self.joints: Dict[str, RobotJoint] = {}
        self.links: Dict[str, RobotLink] = {}
        self.robot_name: str = ""
        self.parameters: Dict[str, Any] = {}

    def load_urdf_from_file(self, urdf_path: str, parameters: Optional[Dict[str, Any]] = None) -> bool:
        """
        Load a URDF file with optional parameterization.

        Args:
            urdf_path: Path to the URDF file
            parameters: Optional parameters to customize the robot model

        Returns:
            True if loading was successful, False otherwise
        """
        if not os.path.exists(urdf_path):
            print(f"URDF file not found: {urdf_path}")
            return False

        try:
            with open(urdf_path, 'r') as file:
                urdf_content = file.read()

            # Apply parameterization if provided
            if parameters:
                urdf_content = self._apply_parameters(urdf_content, parameters)

            return self._parse_urdf_content(urdf_content)
        except Exception as e:
            print(f"Error loading URDF file {urdf_path}: {e}")
            return False

    def load_urdf_from_string(self, urdf_content: str, parameters: Optional[Dict[str, Any]] = None) -> bool:
        """
        Load a URDF from a string with optional parameterization.

        Args:
            urdf_content: URDF content as a string
            parameters: Optional parameters to customize the robot model

        Returns:
            True if loading was successful, False otherwise
        """
        try:
            # Apply parameterization if provided
            if parameters:
                urdf_content = self._apply_parameters(urdf_content, parameters)

            return self._parse_urdf_content(urdf_content)
        except Exception as e:
            print(f"Error parsing URDF content: {e}")
            return False

    def _apply_parameters(self, urdf_content: str, parameters: Dict[str, Any]) -> str:
        """
        Apply parameterization to the URDF content.

        Args:
            urdf_content: Original URDF content
            parameters: Parameters to apply

        Returns:
            Parameterized URDF content
        """
        # Store parameters for later use
        self.parameters.update(parameters)

        # Replace parameter placeholders in the URDF content
        # Parameters are expected to be in the format $(param_name) or ${param_name}
        import re

        def replace_param(match):
            param_name = match.group(1)
            if param_name in parameters:
                return str(parameters[param_name])
            else:
                # If parameter not found, keep the original placeholder
                return match.group(0)

        # Match $(param_name) or ${param_name} patterns
        pattern = r'\$\((\w+)\)|\$\{(\w+)\}'
        urdf_content = re.sub(pattern, lambda m: str(parameters.get(m.group(1) or m.group(2), m.group(0))), urdf_content)

        return urdf_content

    def _parse_urdf_content(self, urdf_content: str) -> bool:
        """
        Parse the URDF content and extract robot information.

        Args:
            urdf_content: URDF content as a string

        Returns:
            True if parsing was successful, False otherwise
        """
        try:
            root = ET.fromstring(urdf_content)

            # Get robot name
            self.robot_name = root.get('name', 'unnamed_robot')

            # Parse links
            self.links = {}
            for link_elem in root.findall('link'):
                link_name = link_elem.get('name')
                if link_name:
                    link = self._parse_link(link_elem)
                    self.links[link_name] = link

            # Parse joints
            self.joints = {}
            for joint_elem in root.findall('joint'):
                joint_name = joint_elem.get('name')
                if joint_name:
                    joint = self._parse_joint(joint_elem)
                    self.joints[joint_name] = joint

            return True
        except ET.ParseError as e:
            print(f"Error parsing URDF XML: {e}")
            return False
        except Exception as e:
            print(f"Error parsing URDF content: {e}")
            return False

    def _parse_link(self, link_elem: ET.Element) -> RobotLink:
        """
        Parse a link element from the URDF.

        Args:
            link_elem: XML element representing a link

        Returns:
            RobotLink object
        """
        link_name = link_elem.get('name')

        # Parse mass
        mass_elem = link_elem.find('inertial/mass')
        mass = float(mass_elem.get('value')) if mass_elem is not None else 0.0

        # Parse inertia
        inertia_elem = link_elem.find('inertial/inertia')
        if inertia_elem is not None:
            inertia = [
                float(inertia_elem.get('ixx', 0)),
                float(inertia_elem.get('ixy', 0)),
                float(inertia_elem.get('ixz', 0)),
                float(inertia_elem.get('iyy', 0)),
                float(inertia_elem.get('iyz', 0)),
                float(inertia_elem.get('izz', 0))
            ]
        else:
            inertia = [0, 0, 0, 0, 0, 0]

        # Parse visual mesh
        visual_elem = link_elem.find('visual/geometry/mesh')
        visual_mesh = visual_elem.get('filename') if visual_elem is not None else None

        # Parse collision mesh
        collision_elem = link_elem.find('collision/geometry/mesh')
        collision_mesh = collision_elem.get('filename') if collision_elem is not None else None

        return RobotLink(
            name=link_name,
            mass=mass,
            inertia=inertia,
            visual_mesh=visual_mesh,
            collision_mesh=collision_mesh
        )

    def _parse_joint(self, joint_elem: ET.Element) -> RobotJoint:
        """
        Parse a joint element from the URDF.

        Args:
            joint_elem: XML element representing a joint

        Returns:
            RobotJoint object
        """
        joint_name = joint_elem.get('name')
        joint_type = joint_elem.get('type', 'fixed')

        # Parse parent and child links
        parent_elem = joint_elem.find('parent')
        child_elem = joint_elem.find('child')
        parent_link = parent_elem.get('link') if parent_elem is not None else ""
        child_link = child_elem.get('link') if child_elem is not None else ""

        # Parse origin
        origin_elem = joint_elem.find('origin')
        if origin_elem is not None:
            xyz_str = origin_elem.get('xyz', '0 0 0')
            rpy_str = origin_elem.get('rpy', '0 0 0')
            origin_xyz = [float(x) for x in xyz_str.split()]
            origin_rpy = [float(x) for x in rpy_str.split()]
        else:
            origin_xyz = [0.0, 0.0, 0.0]
            origin_rpy = [0.0, 0.0, 0.0]

        # Parse limits
        limit_elem = joint_elem.find('limit')
        if limit_elem is not None and joint_type in ['revolute', 'prismatic']:
            limits = JointLimits(
                lower=float(limit_elem.get('lower', 0)),
                upper=float(limit_elem.get('upper', 0)),
                velocity=float(limit_elem.get('velocity', 0)),
                effort=float(limit_elem.get('effort', 0))
            )
        else:
            limits = None

        return RobotJoint(
            name=joint_name,
            joint_type=joint_type,
            limits=limits,
            parent_link=parent_link,
            child_link=child_link,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy
        )

    def get_joint_names(self) -> List[str]:
        """Get the names of all joints in the robot."""
        return list(self.joints.keys())

    def get_link_names(self) -> List[str]:
        """Get the names of all links in the robot."""
        return list(self.links.keys())

    def get_joint(self, joint_name: str) -> Optional[RobotJoint]:
        """Get a specific joint by name."""
        return self.joints.get(joint_name)

    def get_link(self, link_name: str) -> Optional[RobotLink]:
        """Get a specific link by name."""
        return self.links.get(link_name)

    def get_robot_parameters(self) -> Dict[str, Any]:
        """Get the parameters applied to the robot model."""
        return self.parameters.copy()

    def generate_sim_to_real_config(self) -> Dict[str, Any]:
        """
        Generate configuration for sim-to-real transfer based on the URDF.

        Returns:
            Configuration dictionary with sim-to-real parameters
        """
        config = {
            'robot_name': self.robot_name,
            'joints': {},
            'links': {},
            'parameters': self.parameters
        }

        # Add joint-specific configurations
        for joint_name, joint in self.joints.items():
            config['joints'][joint_name] = {
                'type': joint.joint_type,
                'has_limits': joint.limits is not None,
                'parent_link': joint.parent_link,
                'child_link': joint.child_link
            }
            if joint.limits:
                config['joints'][joint_name]['limits'] = {
                    'lower': joint.limits.lower,
                    'upper': joint.limits.upper,
                    'velocity': joint.limits.velocity,
                    'effort': joint.limits.effort
                }

        # Add link-specific configurations
        for link_name, link in self.links.items():
            config['links'][link_name] = {
                'mass': link.mass,
                'has_visual': link.visual_mesh is not None,
                'has_collision': link.collision_mesh is not None
            }

        return config


def load_humanoid_urdf(simulation: bool = True, config_params: Optional[Dict[str, Any]] = None) -> URDFLoader:
    """
    Load a default humanoid URDF with appropriate parameters for simulation or real hardware.

    Args:
        simulation: If True, loads simulation-optimized URDF; otherwise real hardware URDF
        config_params: Additional configuration parameters

    Returns:
        URDFLoader instance with the loaded humanoid model
    """
    loader = URDFLoader()

    # In a real implementation, we would have actual URDF files
    # For now, we'll create a basic URDF string as an example
    if simulation:
        # Example URDF for simulation (simplified)
        urdf_content = """
        <?xml version="1.0"?>
        <robot name="humanoid_robot">
            <link name="base_link">
                <inertial>
                    <mass value="10.0"/>
                    <origin xyz="0 0 0"/>
                    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
                </inertial>
                <visual>
                    <origin xyz="0 0 0"/>
                    <geometry>
                        <box size="0.5 0.5 0.5"/>
                    </geometry>
                </visual>
                <collision>
                    <origin xyz="0 0 0"/>
                    <geometry>
                        <box size="0.5 0.5 0.5"/>
                    </geometry>
                </collision>
            </link>

            <joint name="torso_joint" type="fixed">
                <parent link="base_link"/>
                <child link="torso"/>
                <origin xyz="0 0 0.5"/>
            </joint>

            <link name="torso">
                <inertial>
                    <mass value="5.0"/>
                    <origin xyz="0 0 0"/>
                    <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
                </inertial>
            </link>
        </robot>
        """
    else:
        # Example URDF for real hardware (would have real dimensions and masses)
        urdf_content = """
        <?xml version="1.0"?>
        <robot name="humanoid_robot_real">
            <link name="base_link">
                <inertial>
                    <mass value="15.0"/>
                    <origin xyz="0 0 0"/>
                    <inertia ixx="1.5" ixy="0.0" ixz="0.0" iyy="1.5" iyz="0.0" izz="1.5"/>
                </inertial>
                <visual>
                    <origin xyz="0 0 0"/>
                    <geometry>
                        <box size="0.6 0.6 0.6"/>
                    </geometry>
                </visual>
                <collision>
                    <origin xyz="0 0 0"/>
                    <geometry>
                        <box size="0.6 0.6 0.6"/>
                    </geometry>
                </collision>
            </link>

            <joint name="torso_joint" type="fixed">
                <parent link="base_link"/>
                <child link="torso"/>
                <origin xyz="0 0 0.6"/>
            </joint>

            <link name="torso">
                <inertial>
                    <mass value="7.0"/>
                    <origin xyz="0 0 0"/>
                    <inertia ixx="0.7" ixy="0.0" ixz="0.0" iyy="0.7" iyz="0.0" izz="0.7"/>
                </inertial>
            </link>
        </robot>
        """

    # Apply any additional configuration parameters
    params = config_params or {}
    if simulation:
        params.setdefault('simulation', True)
    else:
        params.setdefault('simulation', False)

    loader.load_urdf_from_string(urdf_content, params)
    return loader