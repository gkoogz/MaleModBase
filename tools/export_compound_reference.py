"""Export measured compound-graft inputs for the SDK-free native evaluator."""
import argparse,struct
from pathlib import Path
import numpy as np

def export(support,module,candidate,output):
    target=np.load(support/'body-union.npz');source=np.load(module/'scrotum-detached.npz')
    neck=np.load(candidate/'neck.npz');skin=np.load(candidate/'scrotum-attached.npz');contact=np.load(candidate/'contact.npz')
    order=np.argsort(neck['source_curve_parameter']);rings=neck['rings'][:,order]
    parameters=.5*(1-np.cos(np.pi*np.arange(len(rings))/(len(rings)-1)))
    with output.open('wb') as f:
        f.write(b'MMCCNT02'+struct.pack('<7I',len(target['points']),len(target['faces']),len(source['points']),len(source['faces']),len(contact['footprint']),len(order),len(rings)))
        for values,dtype in [(target['points'],'<f8'),(target['faces'],'<u4'),(contact['chart'],'<f8'),(contact['eligible'],'u1'),(~target['generated'],'u1'),
            (source['points']+neck['source_translation'],'<f8'),(source['faces'],'<u4'),(contact['source_loop'],'<u4'),(contact['footprint'],'<f8'),
            (neck['source_curve_parameter'][order],'<f8'),(parameters,'<f8'),(neck['start_derivative'][order],'<f8'),(neck['end_derivative'][order],'<f8'),(skin['points'][rings],'<f8')]:
            f.write(values.astype(dtype).tobytes())

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['support','module','candidate','output']:p.add_argument('--'+name,type=Path,required=True)
    export(**vars(p.parse_args()))
