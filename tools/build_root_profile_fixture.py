"""Run original source regularization on the same caller inputs as Python."""
import argparse,subprocess,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tests'))
from test_root_profile import fixture_inputs


def main(executable):
    inputs=list(fixture_inputs());lines=[]
    for points,frame,growth in inputs:
        lines.append(' '.join(format(float(x),'.9g') for x in [growth,frame.body_radius]))
        lines.append(' '.join(format(float(x),'.9g') for x in frame.centers.ravel()))
        rows=np.column_stack([points,frame.flex,frame.root_follow])
        lines.append(' '.join(format(float(x),'.9g') for x in rows.ravel()))
    result=subprocess.run([str(Path(executable).resolve())],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    rows=np.loadtxt(result.stdout.splitlines(),delimiter=',')
    if rows.shape!=(len(inputs)*2388,5) or not np.isfinite(rows).all():raise ValueError('Unexpected original oracle output')
    if not np.array_equal(rows[:,:2],np.array([(i,j) for i in range(len(inputs)) for j in range(2388)])):
        raise ValueError('Oracle output ordering changed')
    output=np.asarray(rows[:,2:],dtype=np.float32).reshape(len(inputs),2388,3)
    # Plain NPY is deterministic; compressed ZIP metadata would change hashes.
    np.save(ROOT/'tests/data/wolverine-root-profile.npy',output,allow_pickle=False)
    print('Original root-profile vertices:',len(rows))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('executable',type=Path)
    main(parser.parse_args().executable)
