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
#
# Build-time helper (run by CMakeLists.txt, not installed): converts Intel's
# official RealSense D435 mesh from realsense2_description into a binary STL.
#
# The triangles are copied through unchanged -- only facet normals are added.
# That's needed because d435.dae carries POSITION data only, no normals, and
# Gazebo can't light a mesh without them: it renders as a flat white
# silhouette no matter what material is set. STL carries a normal per facet,
# so the same geometry renders shaded. Generating this at build time from the
# installed official package (rather than committing a derived ~12 MB binary)
# keeps the repo small and the mesh in sync with Intel's.
#
# Only needs numpy (no pycollada): the DAE is plain XML, and this file's
# structure is simple -- one <triangles> block per geometry, position-only
# indices, no node transforms -- which is asserted below rather than assumed.
#
# Usage: convert_realsense_mesh.py <d435.dae> <out.stl>

import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

NS = '{http://www.collada.org/2005/11/COLLADASchema}'
TRANSFORM_TAGS = {NS + t for t in ('matrix', 'translate', 'rotate', 'scale')}


def load_triangles(dae_path):
    root = ET.parse(dae_path).getroot()

    if any(child.tag in TRANSFORM_TAGS for node in root.iter(NS + 'node') for child in node):
        sys.exit('convert_realsense_mesh: DAE has node transforms, which this converter does not apply')

    triangles = []
    for geometry in root.iter(NS + 'geometry'):
        mesh = geometry.find(NS + 'mesh')
        sources = {
            s.get('id'): np.array(s.find(NS + 'float_array').text.split(), dtype=np.float32).reshape(-1, 3)
            for s in mesh.findall(NS + 'source')
        }
        vertices = {v.get('id'): v.find(NS + 'input').get('source')[1:] for v in mesh.findall(NS + 'vertices')}
        for block in mesh.findall(NS + 'triangles'):
            inputs = block.findall(NS + 'input')
            if len(inputs) != 1:
                sys.exit('convert_realsense_mesh: expected position-only triangles, found extra vertex inputs')
            positions = sources[vertices[inputs[0].get('source')[1:]]]
            indices = np.array(block.find(NS + 'p').text.split(), dtype=np.int64).reshape(-1, 3)
            triangles.append(positions[indices])
    return np.concatenate(triangles)


def write_binary_stl(path, triangles):
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)

    facets = np.zeros(len(triangles), dtype=[('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attributes', '<u2')])
    facets['normal'] = normals
    facets['vertices'] = triangles

    with open(path, 'wb') as f:
        f.write(b'RealSense D435 (realsense2_description d435.dae) with facet normals added'.ljust(80, b' '))
        f.write(struct.pack('<I', len(triangles)))
        f.write(facets.tobytes())


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__ or 'usage: convert_realsense_mesh.py <d435.dae> <out.stl>')
    tris = load_triangles(sys.argv[1])
    write_binary_stl(sys.argv[2], tris)
    print(f'convert_realsense_mesh: wrote {len(tris)} triangles to {sys.argv[2]}')
