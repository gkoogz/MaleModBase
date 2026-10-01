"""Capture float32 reference scalar laws for every physics slider and mode."""
import argparse
import csv
import io
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from malemod_base.controls import defaults, mapped, PHYSICS


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('oracle', type=Path)
    args=parser.parse_args()
    cases=[]
    keys=[p[0] for p in PHYSICS]
    for key in keys:
        for value in [1,25,50,75,100]:
            for mode in [0,.5,1,1.5,2]:
                for length in [12,24,48]:
                    ui=defaults();ui[key]=value
                    raw=[ui[k] for k in keys];values=[mapped(k,ui[k]) for k in keys]
                    cases.append([mode,length,*raw,*values])
    text='\n'.join(' '.join(str(v) for v in row) for row in cases)+'\n'
    result=subprocess.run([str(args.oracle.resolve())],input=text,text=True,capture_output=True,check=True)
    rows=[[float(v) for v in line.split()] for line in result.stdout.splitlines()]
    if len(rows)!=len(cases) or any(len(row)!=22 for row in rows):
        raise ValueError('Incomplete native oracle output')
    output=io.StringIO(newline='')
    writer=csv.writer(output,lineterminator='\n')
    writer.writerow(['mode','rest_length',*keys,*['mapped_'+k for k in keys],*['source_'+str(i) for i in range(22)]])
    writer.writerows([*case,*row] for case,row in zip(cases,rows))
    path=ROOT/'tests/data/physics-controls.csv'
    path.write_text(output.getvalue(),encoding='utf-8')
    print('Captured',len(rows),'original-code material profiles')


if __name__=='__main__':main()
