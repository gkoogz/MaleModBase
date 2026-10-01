"""Capture complete original-source control surfaces in isolated processes."""
import argparse,json,subprocess,hashlib
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from malemod_base.controls import defaults,SHAPE,PHYSICS,limits
from audit_surface_oracle import audit

def capture(executable,output,key,value,seed='-'):
    values=defaults()
    if key!='default':values[key]=value
    label=key if key=='default' else key+'-'+str(value)
    prefix=output/label;prefs=output/(label+'.txt')
    order=['state']+[x[0] for x in SHAPE]+['glans','hang']+[x[0] for x in PHYSICS]
    prefs.write_text(' '.join(str(values[k]) for k in order))
    result=subprocess.run([str(executable),str(seed),
        str(prefs),str(prefix)],check=True,capture_output=True,text=True)
    actual,_,indices=audit(prefix,verbose=False)
    record=dict(label=label,preferences=values,stdout=result.stdout.strip(),
        finalSHA256=hashlib.sha256(prefix.with_suffix('.final').read_bytes()).hexdigest(),
        topologySHA256=hashlib.sha256(indices.tobytes()).hexdigest())
    print('Captured '+label,flush=True)
    return record

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--executable',type=Path,default=ROOT/'build/wolverine_surface_oracle.exe')
    parser.add_argument('--output',type=Path,default=ROOT/'build/surface-oracle/control-sweep')
    parser.add_argument('--controls',nargs='+',default=['overall','length','width','glans','scrotum','hang','angle','forward','vertical'])
    parser.add_argument('--seed',default='-',help='Source-table seed by default; optional local captured donor')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cases=[('default',50)]+[(k,v) for k in args.controls for v in sorted(set([limits(k)[0],25,75,limits(k)[1]]))]
    # Two independent reference processes; each owns its complete solver state.
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures=[pool.submit(capture,args.executable,args.output,k,v,args.seed) for k,v in cases]
        records=[f.result() for f in futures]
    (args.output/'manifest.json').write_text(json.dumps(dict(referenceOnly=True,sourceCommit=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
        oracleSHA256=hashlib.sha256(args.executable.read_bytes()).hexdigest(),seed=args.seed,controls=records),indent=2)+'\n')
