"""Compare the complete SDK-free session outputs against the original oracle."""
import argparse,hashlib,json,subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
from audit_surface_oracle import read_packed


def compare(executable, directory, outdir, record, tolerance):
    label=record['label'];reference=directory/label;output=outdir/(label+'.xyz')
    subprocess.run([str(executable),str(reference)+'.txt',str(output)],check=True,capture_output=True,text=True)
    truth,packed=read_packed(str(reference)+'.final')
    actual=np.fromfile(output,dtype='<f4').reshape(-1,3)
    if actual.shape!=truth.shape or not np.isfinite(actual).all():raise ValueError('Invalid surface: '+label)
    error=np.linalg.norm(actual-truth,axis=1)
    if error.max()>tolerance:raise ValueError('Surface differs: '+label+' '+str(error.max()))
    indices=np.fromfile(str(output)+'.indices',dtype='<u2')
    if not np.array_equal(indices,np.fromfile(str(reference)+'.final-indices',dtype='<u2')):raise ValueError('Topology differs: '+label)
    attributes={}
    for suffix,offset in [('normals',16),('tangents',12)]:
        expected=packed[:,offset:offset+3].astype(np.float32)/np.float32(255)*np.float32(2)-np.float32(1)
        emitted=np.fromfile(str(output)+'.'+suffix,dtype='<f4').reshape(-1,3)
        deviation=float(np.max(np.abs(emitted-expected)))
        # One packed quantization step is a separately recorded lighting gate.
        if deviation>2/255+1e-7:raise ValueError('Lighting differs: '+label+' '+suffix)
        attributes[suffix]=deviation
    uv=packed[:,28:32].copy().view('<f2').reshape(-1,2).astype('<f4')
    if not np.array_equal(uv,np.fromfile(str(output)+'.uv',dtype='<f4').reshape(-1,2)):raise ValueError('UV aliases differ: '+label)
    body_errors=[]
    for i in range(2):
        expected,_=read_packed(str(reference)+'.body'+str(i))
        emitted=np.fromfile(str(output)+'.body'+str(i),dtype='<f4').reshape(-1,3)
        d=float(np.linalg.norm(expected-emitted,axis=1).max())
        if d>tolerance:raise ValueError('Coupled body differs: '+label+' '+str(i)+' '+str(d))
        body_errors.append(d)
    result=dict(label=label,maxPositionError=float(error.max()),rmsPositionError=float(np.sqrt(np.mean(error**2))),
        bodyMaxPositionError=body_errors,lightingMaxComponentError=attributes,exactTopology=True,exactUV=True)
    print('PASS '+label+' max='+str(result['maxPositionError']),flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable',type=Path,required=True)
    parser.add_argument('--reference',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--tolerance',type=float,default=1e-4)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((args.reference/'manifest.json').read_text())
    with ThreadPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(lambda r:compare(args.executable,args.reference,args.output,r,args.tolerance),manifest['controls']))
    report=dict(referenceOnly=True,observedGameplay=False,toleranceSourceUnits=args.tolerance,
        executableSHA256=hashlib.sha256(args.executable.read_bytes()).hexdigest(),
        referenceManifestSHA256=hashlib.sha256((args.reference/'manifest.json').read_bytes()).hexdigest(),cases=results)
    (args.output/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: '+str(len(results))+' complete surfaces, both body sections, topology, UV and lighting gates')
