"""Compare the default-off root candidate with its exact pre-root source library."""
import argparse
import hashlib
import json
import struct
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def verify(args):
    out=args.output.resolve()
    if not out.is_relative_to((ROOT/'build').resolve()) or out.exists():
        raise ValueError('Use a fresh owned proof directory')
    expected=json.loads((ROOT/'provenance/source-surface.json').read_text())['currentProcessRuntime']
    if digest(args.reference_library)!=expected['librarySHA256']:
        raise ValueError('Reference library differs from the preserved pre-root receipt')
    out.mkdir(parents=True)
    cases=[]
    for manifest in args.manifests:
        for record in json.loads(manifest.read_text())['controls']:
            cases.append((manifest.parent/record['label'],manifest.parent.name+'-'+record['label']))
    def compare(case):
        source,label=case;folder=out/label;folder.mkdir()
        for exe,name in ((args.reference,'reference'),(args.executable,'candidate')):
            subprocess.run([str(exe.resolve()),str(source)+'.txt',str(folder/name)],check=True,capture_output=True)
        artifacts=[]
        for suffix in ('','.normals','.tangents','.uv','.indices','.body0','.body1','.mechanics.json'):
            p=folder/('reference'+suffix);q=folder/('candidate'+suffix)
            if not p.stat().st_size or p.read_bytes()!=q.read_bytes():
                raise ValueError('Disabled source output differs: '+label+suffix)
            artifacts.append(dict(suffix=suffix,bytes=q.stat().st_size,sha256=digest(q)))
        print('PASS '+label,flush=True)
        return dict(label=label,artifacts=artifacts)
    with ThreadPoolExecutor(max_workers=2) as pool:
        outputs=list(pool.map(compare,cases))
    reactions=[]
    for state in range(3):
        rows={}
        for mode in (0,2,3,4,5,6):
            p=out/f'reaction-s{state}-m{mode}-reference.bin';q=out/f'reaction-s{state}-m{mode}-candidate.bin'
            for exe,path in ((args.reference_reactions,p),(args.reactions,q)):
                subprocess.run([str(exe.resolve()),str(mode),str(state),str(path)],check=True,capture_output=True)
            a,b=p.read_bytes(),q.read_bytes()
            if struct.unpack_from('<I',a)[0]!=6 or struct.unpack_from('<I',b)[0]!=7 or a[4:]!=b[4:]:
                raise ValueError('Reaction numerical payload differs from actual prior wire6 source')
            rows[mode]=dict(referenceSHA256=digest(p),candidateSHA256=digest(q),payloadSHA256=hashlib.sha256(b[4:]).hexdigest())
        if rows[0]['payloadSHA256']!=rows[2]['payloadSHA256'] or rows[3]['payloadSHA256']==rows[0]['payloadSHA256'] or any(rows[m]['payloadSHA256']!=rows[3]['payloadSHA256'] for m in (4,5,6)):
            raise ValueError('Source impulse consumption/retry/coalescing differs')
        reactions.append(dict(state=state,rows=rows,disabledExact=True,impulseChangesSource=True,retryExact=True,queuedExact=True,coalescedExact=True))
    paths=['CMakeLists.txt','include/malemod/surface/root_contact.hpp','include/malemod/surface/runtime.hpp','include/malemod/surface/wire.hpp','src/surface/runtime.cpp','tools/extract_surface_runtime.py','tools/verify_root_source.py','tests/surface_runtime_cli.cpp','tests/garment_support_runtime_cli.cpp','tests/surface_root_clinical_test.cpp']
    proof=dict(schema='malemod.root-joint-source-consumption/1',wireVersion=7,rootContactsEnabled=False,
        sourceCommit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        sourceHashes={p:digest(ROOT/p) for p in paths},
        binaries={name:dict(path=str(path.resolve()),sha256=digest(path)) for name,path in
                  (('referenceLibrary',args.reference_library),('candidateLibrary',args.library),('reference',args.reference),('candidate',args.executable),('referenceReactions',args.reference_reactions),('candidateReactions',args.reactions))},
        manifests={str(p):digest(p) for p in args.manifests},cases=outputs,reactions=reactions,
        byteIdentical=True,nativeGameplay=False,visualAcceptance=False,fullAttachmentGate=False,installed=False)
    (out/'proof.json').write_text(json.dumps(proof,indent=2)+'\n')
    print('PASS',len(outputs),'complete source cases and all reaction consumption modes')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('reference','executable','reference-library','library','reference-reactions','reactions','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--manifests',nargs='+',type=Path,required=True)
    verify(parser.parse_args())
