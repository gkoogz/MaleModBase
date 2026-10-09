"""Author a chosen ray count from an adapter's measured collision inspection.

Private character exports stay outside Git. Shared adaptive guide placement,
lofting and whole-triangle clearance remain in Base.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import struct
import subprocess
import numpy as np
from scipy.spatial import ConvexHull
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from malemod_base.taut_guides import adaptive_meridian_paths, meridian_taut_paths
from malemod_base.meridian_surface import cloth_surface
from benchmark_meridian_lods import read_obj


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--inspection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rays', type=int, default=32)
    parser.add_argument('--rows', type=int, default=24)
    parser.add_argument('--seam-fitter', type=Path)
    args = parser.parse_args()
    if not 8 <= args.rays <= 256 or not 4 <= args.rows <= 128:
        parser.error('Use rays 8..256 and rows 4..128')
    if args.output.exists():
        raise SystemExit('Choose a fresh output directory')
    source = args.inspection/'collision-model.json'
    data = json.loads(source.read_text())
    guide = data['tautGuides']
    points, groups = read_obj(args.inspection/'collision-model.obj')
    meshes = [(points[np.unique(f)], np.searchsorted(np.unique(f), f)) for f in groups]
    pole, axis = data['dome']['apex'], guide['meridians']['polarAxis']
    paths, receipts, origins, order, density = adaptive_meridian_paths(
        guide['orangeOutline'], pole, axis, meshes, count=args.rays)
    fitted = None
    args.output.mkdir(parents=True)
    if args.seam_fitter:
        seam_input=args.output/'seam-input.bin';seam_output=args.output/'seam-output.bin'
        with seam_input.open('wb') as out:
            out.write(struct.pack('<II',args.rays,len(meshes)))
            out.write(np.asarray([pole,axis],dtype='<f4').tobytes())
            out.write(np.asarray(origins,dtype='<f4').tobytes())
            for vertices,_ in meshes:
                planes=np.unique(np.round(ConvexHull(vertices).equations,9),axis=0)
                planes[:,3]*=-1
                out.write(struct.pack('<I',len(planes)));out.write(np.asarray(planes,dtype='<f4').tobytes())
        subprocess.run([str(args.seam_fitter.resolve()),str(seam_input),str(seam_output)],check=True)
        fitted=np.fromfile(seam_output,dtype='<f4').reshape(-1,3)
        paths,receipts,order=meridian_taut_paths(fitted,pole,axis,meshes)
    points, faces, proof = cloth_surface(paths, guide['orangeOutline'],
        density['originFractions'], pole, axis, data['normals'][-1], meshes,
        rows=args.rows, subdivisions=1, end_density=True,boundary_points=fitted)
    np.savez(args.output/'cloth.npz', points=points, faces=faces,
             columns=args.rays, rows=args.rows)
    proof.update(adaptiveDensity=density, meridians=order,
                 seamFitMaximum=float(np.linalg.norm(fitted-origins,axis=1).max()) if fitted is not None else 0.,
                 inspectionSHA256=hashlib.sha256(source.read_bytes()).hexdigest(),
                 collisionSHA256=hashlib.sha256((args.inspection/'collision-model.obj').read_bytes()).hexdigest(),
                 assetSHA256=hashlib.sha256((args.output/'cloth.npz').read_bytes()).hexdigest(),
                 installed=False, nativeVerified=False)
    (args.output/'proof.json').write_text(json.dumps(proof, indent=2)+'\n')
    print(json.dumps({k: v for k, v in proof.items() if k not in ('adaptiveDensity', 'meridians')}))


if __name__ == '__main__':
    main()
