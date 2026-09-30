"""Build the metric reference body, semantic rig and GLB interchange asset."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.generic import build
from malemod_base.glb import write_glb
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--preferences',type=Path,default=ROOT/'profiles/default-preferences.json');parser.add_argument('--output',type=Path,default=ROOT/'assets/generic-male');args=parser.parse_args()
    asset=build(json.loads(args.preferences.read_text()));args.output.mkdir(parents=True,exist_ok=True)
    path=args.output/'reference.json';path.write_text(json.dumps(asset,separators=(',',':'))+'\n',newline='\n')
    write_glb(args.output/'reference.glb',asset['mesh'],asset['skeleton'])
    manifest={'contractVersion':1,'generator':'tools/build_generic.py','units':'meters','vertexCount':len(asset['mesh']['positions']),
              'triangleCount':len(asset['mesh']['indices']),'jointCount':len(asset['skeleton']),'socketCount':len(asset['sockets']),
              'files':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [path,args.output/'reference.glb']},
              'limitations':['reference proxy made of overlapping surfaces','body control changes require target re-fitting','not a game skeleton','facial controls reserved']}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',newline='\n')
    print(f'Built {manifest["vertexCount"]} vertices, {manifest["triangleCount"]} triangles, {manifest["jointCount"]} joints and {manifest["socketCount"]} sockets.')
if __name__=='__main__':main()
