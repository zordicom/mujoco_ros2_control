#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Generic URDF to MuJoCo XML converter with multi-mode actuator support.

This script converts any URDF to MuJoCo XML and automatically adds three
actuators per joint (position, velocity, motor) following the naming
conventions required by mujoco_ros2_control's actuator-centric architecture.

Usage:
    # Basic conversion
    python3 urdf_to_mjcf.py input.urdf output.xml

    # With custom gains
    python3 urdf_to_mjcf.py input.urdf output.xml --kp-pos 200 --kv-vel 20

    # Disable collisions (recommended for gravity compensation testing)
    python3 urdf_to_mjcf.py input.urdf output.xml --disable-collisions
"""

import argparse
import math
import sys
from pathlib import Path

import mujoco


def add_actuators_to_mjcf(
    mjcf_path: Path,
    kp_pos: float = 100.0,
    kv_pos: float = 0.0,
    kv_vel: float = 10.0,
) -> None:
    """Add three actuators per joint to MuJoCo XML.

    Automatically detects joints from the MuJoCo model and adds:
    - Position actuator: act_pos_{joint_name}
    - Velocity actuator: act_vel_{joint_name}
    - Motor actuator: act_tau_{joint_name}

    Args:
        mjcf_path: Path to MuJoCo XML file
        kp_pos: Position actuator stiffness (default: 100.0, validated)
        kv_pos: Position actuator damping (default: 0.0, for MIT mode compatibility)
        kv_vel: Velocity actuator gain (default: 10.0, validated)
    """
    import xml.etree.ElementTree as ET

    print(f"Adding actuators to {mjcf_path.name}...")

    # Load MuJoCo model to get joint information
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))

    # Parse XML
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    # Remove any existing actuator elements
    for actuator_elem in root.findall(".//actuator"):
        root.remove(actuator_elem)

    # Create new actuator element
    actuator_elem = ET.SubElement(root, "actuator")

    num_joints = 0
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]

        # Skip fixed joints
        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            continue

        # Get joint limits
        jnt_id = i
        qpos_adr = model.jnt_qposadr[jnt_id]
        dof_adr = model.jnt_dofadr[jnt_id]

        # Determine effort limit (default 40 Nm)
        effort_limit = 40.0
        if hasattr(model, "actuator_forcerange"):
            # Check if there are any existing actuators to infer from
            for act_id in range(model.nu):
                if model.actuator_trnid[act_id, 0] == jnt_id:
                    effort_limit = abs(model.actuator_forcerange[act_id, 1])
                    break

        # Position limits
        if model.jnt_limited[jnt_id]:
            pos_min = model.jnt_range[jnt_id, 0]
            pos_max = model.jnt_range[jnt_id, 1]
        else:
            pos_min = -math.pi  # Default for unlimited joints
            pos_max = math.pi

        # Velocity limit (default 3.0 rad/s)
        vel_limit = 3.0

        # 1. Position actuator
        ET.SubElement(
            actuator_elem,
            "position",
            name=f"act_pos_{joint_name}",
            joint=joint_name,
            kp=str(kp_pos),
            kv=str(kv_pos),
            ctrlrange=f"{pos_min} {pos_max}",
            forcerange=f"-{effort_limit} {effort_limit}",
        )

        # 2. Velocity actuator
        ET.SubElement(
            actuator_elem,
            "velocity",
            name=f"act_vel_{joint_name}",
            joint=joint_name,
            kv=str(kv_vel),
            ctrlrange=f"-{vel_limit} {vel_limit}",
            forcerange=f"-{effort_limit} {effort_limit}",
        )

        # 3. Motor actuator (torque)
        ET.SubElement(
            actuator_elem,
            "motor",
            name=f"act_tau_{joint_name}",
            joint=joint_name,
            ctrlrange=f"-{effort_limit} {effort_limit}",
            forcerange=f"-{effort_limit} {effort_limit}",
        )

        num_joints += 1

    # Write back
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)

    print(f"✓ Added {num_joints * 3} actuators ({num_joints} joints × 3 types)")
    print(f"  Position: kp={kp_pos}, kv={kv_pos}")
    print(f"  Velocity: kv={kv_vel}")
    print("  Motor: direct torque")


def disable_collisions(mjcf_path: Path) -> None:
    """Disable all collisions in the MuJoCo XML.

    Sets contype=0 and conaffinity=0 for all geoms to prevent self-collision
    issues that can interfere with gravity compensation.
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    # Set in defaults
    for default in root.findall(".//default"):
        for geom in default.findall("./geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")

    # Set for individual geoms
    for geom in root.findall(".//geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")

    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)
    print("✓ Disabled all collisions (contype=0, conaffinity=0)")


def fix_compiler_settings(mjcf_path: Path) -> None:
    """Add recommended compiler settings to MuJoCo XML."""
    import xml.etree.ElementTree as ET

    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    # Add or update compiler settings
    compiler = root.find(".//compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")

    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")
    compiler.set("balanceinertia", "true")  # Fix inertia matrix violations

    # Remove meshdir to avoid path issues
    if "meshdir" in compiler.attrib:
        del compiler.attrib["meshdir"]

    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)
    print("✓ Updated compiler settings")


def validate_mjcf(mjcf_path: Path) -> None:
    """Validate that the MuJoCo XML can be loaded.

    Args:
        mjcf_path: Path to MuJoCo XML file

    Raises:
        RuntimeError: If validation fails
    """
    print("Validating MuJoCo XML...")

    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        data = mujoco.MjData(model)

        print("✓ MuJoCo XML is valid")
        print(f"  - DOF: {model.nv}")
        print(f"  - Bodies: {model.nbody}")
        print(f"  - Joints: {model.njnt}")
        print(f"  - Actuators: {model.nu}")

        # Print joint names for reference
        if model.njnt > 0:
            print("\n  Joint names:")
            for i in range(model.njnt):
                joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                print(f"    - {joint_name}")

        # Print actuator names
        if model.nu > 0:
            print("\n  Actuator names:")
            for i in range(min(model.nu, 20)):  # Limit to 20 for readability
                act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                print(f"    - {act_name}")
            if model.nu > 20:
                print(f"    ... and {model.nu - 20} more")

    except Exception as e:
        print(f"✗ MuJoCo XML validation failed: {e}", file=sys.stderr)
        raise


def convert_urdf_to_mjcf(urdf_path: Path, output_path: Path) -> None:
    """Convert URDF to MuJoCo XML using urdf2mjcf.

    Args:
        urdf_path: Path to input URDF file
        output_path: Path to output MuJoCo XML file
    """
    print(f"Converting {urdf_path.name} to MuJoCo XML...")

    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        from urdf2mjcf.convert import convert_urdf_to_mjcf as urdf2mjcf_convert

        urdf2mjcf_convert(str(urdf_path), str(output_path), copy_meshes=True)
        print(f"✓ Converted to MuJoCo XML: {output_path}")
    except ImportError:
        print("Error: urdf2mjcf not installed. Install with: pip install urdf2mjcf")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Conversion failed: {e}", file=sys.stderr)
        raise


def main():
    """Main conversion pipeline."""
    parser = argparse.ArgumentParser(
        description="Convert URDF to MuJoCo XML with multi-mode actuators",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic conversion
  python3 urdf_to_mjcf.py robot.urdf robot.xml

  # With custom gains
  python3 urdf_to_mjcf.py robot.urdf robot.xml --kp-pos 200 --kv-vel 20

  # Disable collisions (recommended for gravity compensation)
  python3 urdf_to_mjcf.py robot.urdf robot.xml --disable-collisions

  # Validate existing XML
  python3 urdf_to_mjcf.py --validate-only robot.xml
        """,
    )

    parser.add_argument("input", type=Path, help="Input URDF file")
    parser.add_argument(
        "output",
        type=Path,
        nargs="?",
        help="Output MuJoCo XML file (required unless --validate-only)",
    )

    parser.add_argument(
        "--kp-pos",
        type=float,
        default=100.0,
        help="Position actuator stiffness gain (default: 100.0, validated)",
    )
    parser.add_argument(
        "--kv-pos",
        type=float,
        default=0.0,
        help="Position actuator damping gain (default: 0.0, for MIT mode)",
    )
    parser.add_argument(
        "--kv-vel",
        type=float,
        default=10.0,
        help="Velocity actuator gain (default: 10.0, validated)",
    )
    parser.add_argument(
        "--disable-collisions",
        action="store_true",
        help="Disable all collisions (recommended for gravity compensation testing)",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Only validate existing MuJoCo XML file (input should be .xml)",
    )

    args = parser.parse_args()

    try:
        # Validate-only mode
        if args.validate_only:
            if not args.input.exists():
                print(f"Error: File not found: {args.input}", file=sys.stderr)
                sys.exit(1)
            validate_mjcf(args.input)
            sys.exit(0)

        # Normal conversion mode
        if not args.output:
            print(
                "Error: output file is required unless --validate-only is used",
                file=sys.stderr,
            )
            parser.print_help()
            sys.exit(1)

        if not args.input.exists():
            print(f"Error: Input file not found: {args.input}", file=sys.stderr)
            sys.exit(1)

        print(f"\n{'=' * 80}")
        print("URDF to MuJoCo Conversion")
        print(f"{'=' * 80}\n")

        # Step 1: Convert URDF to MuJoCo XML
        convert_urdf_to_mjcf(args.input, args.output)

        # Step 2: Fix compiler settings
        fix_compiler_settings(args.output)

        # Step 3: Disable collisions if requested
        if args.disable_collisions:
            disable_collisions(args.output)

        # Step 4: Add multi-mode actuators
        add_actuators_to_mjcf(
            args.output,
            kp_pos=args.kp_pos,
            kv_pos=args.kv_pos,
            kv_vel=args.kv_vel,
        )

        # Step 5: Validate the result
        validate_mjcf(args.output)

        print(f"\n{'=' * 80}")
        print("✓ Conversion complete!")
        print(f"{'=' * 80}")
        print(f"\nOutput: {args.output}")
        print("\nNext steps:")
        print("  1. Add keyframes to the XML for initial poses (optional)")
        print("  2. Update your URDF <ros2_control> section to match joint names")
        print(
            "  3. Test with: ros2 launch mujoco_ros2_control_demos <your_launch_file>"
        )
        print()

    except Exception as e:
        print(f"\n✗ Conversion failed: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
