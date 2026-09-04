"""Test the public launch interfaces of ground_vehicle_kinematics."""

import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_launch_module(filename: str) -> ModuleType:
    path = PACKAGE_ROOT / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename.replace('.', '_'), path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    'launch_file', ['four_swerve_kinematics.launch.py', 'three_swerve_kinematics.launch.py']
)
def test_launch_exposes_the_current_node_arguments_contract(launch_file: str) -> None:
    """Require node_args and an explicit parameter-substitution switch."""
    module = _load_launch_module(launch_file)
    declarations = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert 'node_args' in declarations
    assert 'params_file_allow_substs' in declarations

    context = LaunchContext()
    declarations['node_args'].visit(context)
    assert context.launch_configurations['node_args'] == (
        '{"output":"both","ros_arguments":["--log-level","info"]}'
    )
