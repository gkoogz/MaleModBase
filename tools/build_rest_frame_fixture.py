"""Regenerate rest measurements from the compiled original source oracle."""
import argparse,subprocess,sys
import numpy as np
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tests'))
from malemod_base.authored_shape import AuthoredShape,shape_values
from test_rest_frame import cases

def main(executable):
    lines=[]
    with np.load(ROOT/'assets/wolverine-reference/geometry.npz',allow_pickle=False) as bank:shape=AuthoredShape(bank)
    for preferences,mode,prior in cases():
        _,values=shape_values(preferences)
        lines.append(' '.join(str(x) for x in [*values,preferences['hang'],mode,prior]))
        points=np.asarray(shape.evaluate(preferences).coarse,dtype=np.float32)
        lines.append(' '.join(format(float(x),'.9g') for x in points.ravel()))
    result=subprocess.run([str(Path(executable).resolve())],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    rows=result.stdout.splitlines()
    if len(rows)!=len(lines)//2*18 or any(len(row.split(','))!=11 for row in rows):raise ValueError('Unexpected oracle output')
    (ROOT/'tests/data/wolverine-rest-frame.csv').write_bytes(('case,index,x,y,z,radius,length,prior_used,vertex,flex,root_follow\n'+'\n'.join(rows)+'\n').encode())
    print('Original rest-frame rows:',len(rows))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('executable',type=Path)
    main(parser.parse_args().executable)
