"""Export the evaluated Wolverine reset state and its mechanical guide together.

Offline 32-bit strict source session only; never modifies a game or adapter.
"""
import argparse,hashlib,json,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def export(executable,output,steps=120):
    output=output.resolve()
    if not output.is_relative_to(ROOT/'build') or output.exists():
        raise ValueError('Choose a new output directory inside Base build/')
    output.mkdir(parents=True)
    prefs=output/'controls.txt';prefs.write_text('2 '+' '.join(['50']*17)+'\n')
    xyz=output/'surface.xyz'
    subprocess.run([str(executable.resolve()),str(prefs),str(xyz),str(steps)],check=True)
    positions=np.fromfile(xyz,dtype='<f4').reshape(-1,3)
    indices=np.fromfile(str(xyz)+'.indices',dtype='<u2').reshape(-1,3)
    uv=np.fromfile(str(xyz)+'.uv',dtype='<f4').reshape(-1,2)
    with np.load(ROOT/'assets/wolverine-reference/geometry.npz') as bank:
        np.testing.assert_array_equal(indices,bank['neck_render_data__nrIndices'].reshape(-1,3))
        posed=bank['derived__final_reference_positions']
    mechanical=json.loads(Path(str(xyz)+'.mechanics.json').read_text())
    guide=np.asarray(mechanical['shaftGuide'])
    if guide.shape!=(12,3) or np.min(np.linalg.norm(np.diff(guide,axis=0),axis=1))<.01:
        raise ValueError('Uninitialized source guide')
    if not all(np.isfinite(x).all() for x in [positions,uv,guide]):raise ValueError('Invalid evaluated default')
    np.savez_compressed(output/'baseline.npz',positions=positions,indices=indices,uv=uv,sourceVertexIDs=np.arange(len(positions),dtype=np.uint32))
    (output/'mechanics.json').write_text(json.dumps(mechanical,indent=2)+'\n')
    record=dict(contractVersion=1,state=2,controls='all seventeen controls at UI 50; ResetStudyControls defaults',steps=steps,
        sourceCommit=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
        executableSHA256=sha(executable),surfaceImplementationHashes={p:sha(ROOT/p) for p in ['include/malemod/surface/runtime.hpp','src/surface/runtime.cpp','tools/export_default_baseline.py']},
        sourceBankSHA256=sha(ROOT/'assets/wolverine-reference/geometry.npz'),exactTopology=True,
        sourceBounds=[positions.min(0).tolist(),positions.max(0).tolist()],
        posedReferenceBounds=[posed.min(0).tolist(),posed.max(0).tolist()],
        differenceFromPosedReferenceRMS=float(np.sqrt(np.mean(np.sum((positions-posed)**2,axis=1)))),
        files={p.name:sha(p) for p in [output/'baseline.npz',output/'mechanics.json',prefs]},
        observedGameplay=False,physicsFrame='source reference thigh capsules; zero gait/side input')
    (output/'manifest.json').write_text(json.dumps(record,indent=2)+'\n');return record
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);p.add_argument('--executable',type=Path,default=ROOT/'build/surface-cmake/Release/surface_runtime_cli.exe');p.add_argument('--steps',type=int,default=120)
    a=p.parse_args();print(json.dumps(export(a.executable,a.output,a.steps),indent=2))
