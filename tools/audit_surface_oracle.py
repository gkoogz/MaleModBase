"""Inspect actual final Wolverine vertices against the static export table."""
import json,hashlib
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
def read_packed(path):
    raw=np.fromfile(path,dtype=np.uint8).reshape(-1,32)
    return raw[:,:12].copy().view('<f4').reshape(-1,3),raw

def audit(prefix, verbose=True):
    prefix=Path(prefix)
    actual,packed=read_packed(str(prefix)+'.final')
    with np.load(ROOT/'assets/wolverine-reference/geometry.npz') as bank:
        reference=bank['derived__final_reference_positions']
        indices=bank['neck_render_data__nrIndices'].reshape(-1,3)
    native_indices=np.fromfile(str(prefix)+'.final-indices',dtype='<u2').reshape(-1,3)
    if not np.array_equal(native_indices,indices):raise ValueError('Source topology changed')
    delta=np.linalg.norm(actual-reference,axis=1)
    result=dict(finalVertices=len(actual),triangles=len(indices),
        exactSourceIndexOrder=True,finite=bool(np.isfinite(actual).all()),
        staticReferenceDifference=dict(rms=float(np.sqrt(np.mean(delta**2))),
            maximum=float(delta.max()),p99=float(np.quantile(delta,.99))),
        actualBounds=[actual.min(0).tolist(),actual.max(0).tolist()],
        referenceBounds=[reference.min(0).tolist(),reference.max(0).tolist()],
        finalVertexSHA256=hashlib.sha256(packed.tobytes()).hexdigest(),
        observedGameplay=False)
    prefix.with_suffix('.audit.json').write_text(json.dumps(result,indent=2)+'\n')
    if verbose:print(json.dumps(result,indent=2))
    return actual,reference,indices

if __name__=='__main__':
    import sys
    audit(sys.argv[1])
