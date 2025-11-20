#!/usr/bin/env python3
"""
Script to extract joints and links from a URDF file into separate xacro files.

Usage:
    python3 extract_urdf_components.py <input_urdf> <output_joints_xacro> <output_links_xacro>

Example:
    python3 extract_urdf_components.py \\
        src/robot_description/arm_description/urdf/humanoid_arm_5dof.urdf \\
        src/robot_description/arm_description/urdf/joints/arm_joints.xacro \\
        src/robot_description/arm_description/urdf/links/arm_links.xacro
"""

import xml.etree.ElementTree as ET
import argparse
import sys
from pathlib import Path


def indent_xml(elem, level=0, indent_size=4):
    """Add proper indentation to XML elements."""
    indent_str = "\n" + " " * (indent_size * level)

    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent_str + " " * indent_size
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent_str
        for child in elem:
            indent_xml(child, level + 1, indent_size)
        if not child.tail or not child.tail.strip():
            child.tail = indent_str
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = indent_str


def create_xacro_header():
    """Create the XML declaration and robot opening tag for xacro files."""
    return '<?xml version="1.0"?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'


def create_xacro_footer():
    """Create the closing robot tag."""
    return '</robot>\n'


def extract_elements(urdf_file, element_type):
    """Extract all elements of a specific type (link or joint) from URDF."""
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    elements = root.findall(element_type)
    return elements


def format_element(element, indent_level=1):
    """Format an XML element with proper indentation."""
    # Create a copy to avoid modifying the original
    elem_copy = ET.fromstring(ET.tostring(element))

    # Apply indentation
    indent_xml(elem_copy, level=indent_level, indent_size=4)

    # Convert to string
    elem_str = ET.tostring(elem_copy, encoding='unicode')

    # Add proper indentation for the first line
    indent = "    " * indent_level
    elem_str = indent + elem_str.lstrip()

    return elem_str


def fix_mesh_paths(element):
    """Fix mesh paths from package://humanoid_arm_5dof/meshes/ to package://arm_description/meshes/visual/"""
    # Find all mesh elements
    for mesh in element.iter('mesh'):
        filename = mesh.get('filename', '')
        if 'package://humanoid_arm_5dof/meshes/' in filename:
            # Extract just the mesh filename
            mesh_name = filename.split('/')[-1]
            # Update to new path
            new_path = f'package://arm_description/meshes/visual/{mesh_name}'
            mesh.set('filename', new_path)


def write_xacro_file(output_file, elements, element_type):
    """Write elements to a xacro file with proper formatting."""
    with open(output_file, 'w') as f:
        # Write header
        f.write(create_xacro_header())

        # Write each element
        for elem in elements:
            # Fix mesh paths before formatting
            fix_mesh_paths(elem)

            elem_str = format_element(elem, indent_level=1)
            f.write(elem_str)
            if not elem_str.endswith('\n'):
                f.write('\n')

        # Write footer
        f.write(create_xacro_footer())


def main():
    parser = argparse.ArgumentParser(
        description='Extract joints and links from URDF file into separate xacro files.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    python3 extract_urdf_components.py \\
        src/robot_description/arm_description/urdf/humanoid_arm_5dof.urdf \\
        src/robot_description/arm_description/urdf/joints/arm_joints.xacro \\
        src/robot_description/arm_description/urdf/links/arm_links.xacro
        """
    )

    parser.add_argument('input_urdf', type=str, help='Path to input URDF file')
    parser.add_argument('output_joints', type=str, help='Path to output joints xacro file')
    parser.add_argument('output_links', type=str, help='Path to output links xacro file')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')

    args = parser.parse_args()

    # Validate input file exists
    input_path = Path(args.input_urdf)
    if not input_path.exists():
        print(f"Error: Input file '{args.input_urdf}' not found!", file=sys.stderr)
        sys.exit(1)

    # Create output directories if they don't exist
    Path(args.output_joints).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output_links).parent.mkdir(parents=True, exist_ok=True)

    try:
        # Extract links
        if args.verbose:
            print(f"Extracting links from {args.input_urdf}...")
        links = extract_elements(args.input_urdf, 'link')
        if args.verbose:
            print(f"Found {len(links)} links")

        # Extract joints
        if args.verbose:
            print(f"Extracting joints from {args.input_urdf}...")
        joints = extract_elements(args.input_urdf, 'joint')
        if args.verbose:
            print(f"Found {len(joints)} joints")

        # Write output files
        if args.verbose:
            print(f"Writing links to {args.output_links}...")
        write_xacro_file(args.output_links, links, 'link')

        if args.verbose:
            print(f"Writing joints to {args.output_joints}...")
        write_xacro_file(args.output_joints, joints, 'joint')

        print(f"✓ Successfully extracted {len(links)} links and {len(joints)} joints")
        print(f"  Links:  {args.output_links}")
        print(f"  Joints: {args.output_joints}")

    except ET.ParseError as e:
        print(f"Error parsing URDF file: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
