"""Evaluate only Overall against the verified Win32 source session, offline."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def export(output, values, executable, steps=120):
    output=output.resolve(); executable=executable.resolve()
    if not output.is_relative_to(ROOT/'build') or output.exists():
        raise ValueError('Choose a new Base build directory')
    output.mkdir(parents=True)
    with np.load(ROOT/'assets/wolverine-reference/geometry.npz') as bank:
        faces=bank['neck_render_data__nrIndices'].reshape(-1,3).copy()
    def one(ui):
        if not 1<=ui<=100:raise ValueError('Overall UI range is 1..100')
        folder=output/('ui-'+str(ui));folder.mkdir()
        controls=folder/'controls.txt'
        controls.write_text('2 '+str(ui)+' '+' '.join(['50']*16)+'\n')
        raw=folder/'surface.xyz'
        process=subprocess.run([str(executable),str(controls),str(raw),str(steps)],capture_output=True,text=True,check=True)
        positions=np.fromfile(raw,dtype='<f4').reshape(-1,3)
        actual_faces=np.fromfile(str(raw)+'.indices',dtype='<u2').reshape(-1,3)
        np.testing.assert_array_equal(actual_faces,faces)
        arrays=dict(positions=positions,indices=actual_faces,uv=np.fromfile(str(raw)+'.uv',dtype='<f4').reshape(-1,2))
        for side in range(2):arrays['body'+str(side)]=np.fromfile(str(raw)+'.body'+str(side),dtype='<f4').reshape(-1,3)
        if not all(np.isfinite(v).all() for v in arrays.values()):raise ValueError('Nonfinite source output')
        target=folder/'surface.npz';np.savez_compressed(target,**arrays)
        mechanics=folder/'mechanics.json';mechanics.write_bytes(Path(str(raw)+'.mechanics.json').read_bytes())
        row=dict(ui=ui,surface=target.relative_to(output).as_posix(),surfaceSHA256=sha(target),mechanics=mechanics.relative_to(output).as_posix(),mechanicsSHA256=sha(mechanics))
        print('Evaluated Overall',ui,process.stdout.strip(),flush=True)
        return row
    with ThreadPoolExecutor(max_workers=2) as pool:
        rows=list(pool.map(one,values))
    record=dict(contractVersion=1,sourceCommit=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
                executableSHA256=sha(executable),exporterSHA256=sha(Path(__file__)),steps=steps,state=2,
                otherControls='all sixteen at UI 50',evaluatedPipeline='complete source including final UnifiedCollar and both body sections',
                rows=rows,observedGameplay=False)
    (output/'manifest.json').write_text(json.dumps(record,indent=2)+'\n')
    return record

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path)
    p.add_argument('--values',default='1,10,20,30,40,50,60,70,80,90,100')
    p.add_argument('--executable',type=Path,default=ROOT/'build/surface-cmake/Release/surface_runtime_cli.exe')
    a=p.parse_args();export(a.output,[int(x) for x in a.values.split(',')],a.executable)
