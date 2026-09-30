"""Regenerate the early-shape CSV using the compiled original-source oracle."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tests'))
from malemod_base.authored_shape import shape_values
from test_authored_shape import fixture_preferences


def main(executable):
    lines=[]
    for p in fixture_preferences():
        _, values=shape_values(p)
        lines.append(' '.join(str(x) for x in [*values,p['hang']]))
    result=subprocess.run([str(Path(executable).resolve())],input='\n'.join(lines)+'\n',
                          text=True,capture_output=True,check=True)
    rows=result.stdout.splitlines()
    if len(rows)!=len(lines)*65 or any(len(row.split(','))!=7 for row in rows):
        raise ValueError('Unexpected original-source fixture output')
    (ROOT/'tests/data/wolverine-authored-shape.csv').write_text(
        'case,index,x,y,z,growth,hang_z\n'+'\n'.join(rows)+'\n',encoding='utf-8',newline='\n')
    print('Original-source fixture rows:',len(rows))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('executable',type=Path)
    main(parser.parse_args().executable)
