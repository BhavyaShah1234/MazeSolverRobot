#!/usr/bin/env python3
# Copyright (c) 2026 Bhavya Shah
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Converts Intel's official RealSense D435 mesh into a binary STL.

Build-time helper (run by CMakeLists.txt, not installed): converts Intel's
official RealSense D435 mesh from realsense2_description into a binary STL.

The triangles are copied through unchanged -- only facet normals are added.
That's needed because d435.dae carries POSITION data only, no normals, and
Gazebo can't light a mesh without them: it renders as a flat white
silhouette no matter what material is set. STL carries a normal per facet,
so the same geometry renders shaded. Generating this at build time from the
installed official package (rather than committing a derived ~12 MB binary)
keeps the repo small and the mesh in sync with Intel's.

Only needs numpy (no pycollada): the DAE is plain XML, and this file's
structure is simple -- one <triangles> block per geometry, position-only
indices, no node transforms -- which is asserted below rather than assumed.

Usage: convert_realsense_mesh.py <d435.dae> <out.stl>
"""

# struct: packs the binary STL header/triangle-count/facet data.
import struct
# sys: reads argv and exits with an error message.
import sys
# ElementTree: parses the DAE (which is plain XML).
import xml.etree.ElementTree as ET

# NumPy: vector math for normals and bulk array reshaping.
import numpy as np

# The COLLADA XML namespace every tag in a .dae file lives under.
NS = '{http://www.collada.org/2005/11/COLLADASchema}'
# The set of node-transform tags this converter explicitly does not support.
TRANSFORM_TAGS = {NS + t for t in ('matrix', 'translate', 'rotate', 'scale')}


# Parse a DAE file into a single (N, 3, 3) array of triangle vertex positions.
def load_triangles(dae_path: str) -> np.ndarray:
    """Parses a DAE file into a single array of triangle vertex positions.

    Args:
        dae_path: Path to the source COLLADA (.dae) mesh file.

    Returns:
        An (N, 3, 3) array of N triangles' 3 vertex positions each.
    """
    # Parse the DAE XML and get its root element.
    root = ET.parse(dae_path).getroot()

    # Refuse to proceed if any <node> applies a transform this converter doesn't handle.
    if any(child.tag in TRANSFORM_TAGS for node in root.iter(NS + 'node') for child in node):
        # Bail out loudly rather than silently producing wrong geometry.
        sys.exit('convert_realsense_mesh: DAE has node transforms, which this converter does not apply')

    # Accumulates one (N, 3, 3) triangle array per <geometry> block.
    triangles = []
    # Walk every <geometry> element in the document.
    for geometry in root.iter(NS + 'geometry'):
        # This geometry's <mesh> element.
        mesh = geometry.find(NS + 'mesh')
        # Map each <source> id to its raw float array, reshaped into (N, 3) vectors.
        sources = {
            s.get('id'): np.array(s.find(NS + 'float_array').text.split(), dtype=np.float32).reshape(-1, 3)
            for s in mesh.findall(NS + 'source')
        }
        # Map each <vertices> id to the source id it actually reads positions from.
        vertices = {v.get('id'): v.find(NS + 'input').get('source')[1:] for v in mesh.findall(NS + 'vertices')}
        # Walk every <triangles> block in this mesh.
        for block in mesh.findall(NS + 'triangles'):
            # This block's <input> elements (one per vertex attribute stream).
            inputs = block.findall(NS + 'input')
            # This converter only supports position-only triangle data.
            if len(inputs) != 1:
                # Bail out loudly rather than silently dropping extra attributes.
                sys.exit('convert_realsense_mesh: expected position-only triangles, found extra vertex inputs')
            # The actual (N, 3) position array this triangle block indexes into.
            positions = sources[vertices[inputs[0].get('source')[1:]]]
            # The flat index list, reshaped into one row of 3 indices per triangle.
            indices = np.array(block.find(NS + 'p').text.split(), dtype=np.int64).reshape(-1, 3)
            # Gather this block's actual triangle vertex positions and store them.
            triangles.append(positions[indices])
    # Concatenate every geometry's triangles into one array.
    return np.concatenate(triangles)


# Write an (N, 3, 3) triangle array out as a binary STL file, computing facet normals.
def write_binary_stl(path: str, triangles: np.ndarray) -> None:
    """Writes a triangle array out as a binary STL file with facet normals.

    Args:
        path: Destination path for the binary STL file.
        triangles: An (N, 3, 3) array of N triangles' 3 vertex positions
            each, as returned by :func:`load_triangles`.
    """
    # Facet normal = cross product of two edge vectors of each triangle.
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    # Length of each normal vector, kept 2D for broadcasting in the divide below.
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    # Normalize each normal to unit length, leaving degenerate (zero-length) ones as zero.
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)

    # Binary STL's per-facet record layout: normal, 3 vertices, and a padding "attribute" field.
    facets = np.zeros(len(triangles), dtype=[('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attributes', '<u2')])
    # Fill in each facet's normal.
    facets['normal'] = normals
    # Fill in each facet's three vertex positions.
    facets['vertices'] = triangles
    # Write the binary STL file.
    with open(path, 'wb') as f:
        # STL's 80-byte header comment, padded to exactly 80 bytes.
        f.write(b'RealSense D435 (realsense2_description d435.dae) with facet normals added'.ljust(80, b' '))
        # 4-byte little-endian triangle count.
        f.write(struct.pack('<I', len(triangles)))
        # The packed facet records themselves.
        f.write(facets.tobytes())


# Only run when invoked directly (this is a build-time script, not an importable module).
if __name__ == '__main__':
    # Require exactly the input and output file paths as arguments.
    if len(sys.argv) != 3:
        # Print usage and exit if called incorrectly.
        sys.exit(__doc__ or 'usage: convert_realsense_mesh.py <d435.dae> <out.stl>')
    # Load the triangles from the input DAE.
    tris = load_triangles(sys.argv[1])
    # Write them out as a binary STL with computed normals.
    write_binary_stl(sys.argv[2], tris)
    # Report how many triangles were converted.
    print(f'convert_realsense_mesh: wrote {len(tris)} triangles to {sys.argv[2]}')
