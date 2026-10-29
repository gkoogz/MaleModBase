"""Validate, scaffold and prepare approved voice delivery without synthesis."""
import argparse,csv,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from malemod_base.voice_pipeline import COLUMNS,catalog,next_id,scaffold,read_script,validate

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catalog',type=Path,default=ROOT/'profiles/voice-lines.json')
    sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('check-catalog');sub.add_parser('next-id')
    s=sub.add_parser('scaffold');s.add_argument('--character',required=True);s.add_argument('--output',type=Path,required=True)
    s=sub.add_parser('check');s.add_argument('--script',type=Path,required=True);s.add_argument('--audio-root',type=Path)
    s.add_argument('--require-approved',action='store_true');s.add_argument('--output',type=Path)
    a=p.parse_args();data=catalog(a.catalog)
    if a.command=='check-catalog':print('PASS voice catalog:',len(data['lines']),'permanent IDs');return
    if a.command=='next-id':print(next_id(data));return
    if a.command=='scaffold':
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open('x',encoding='utf-8-sig',newline='')as f:
            w=csv.DictWriter(f,fieldnames=COLUMNS);w.writeheader();w.writerows(scaffold(data,a.character))
        print(a.output);return
    result=validate(data,read_script(a.script),a.audio_root,a.require_approved)
    result.update(catalog=str(a.catalog.resolve()),script=str(a.script.resolve()),
        catalogSHA256=hashlib.sha256(a.catalog.read_bytes()).hexdigest(),scriptSHA256=hashlib.sha256(a.script.read_bytes()).hexdigest())
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open('x',encoding='utf-8',newline='\n')as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(dict(character=result['character_id'],lines=result['lineCount'],statuses=result['statuses'],ready=result['ready'])))
if __name__=='__main__':main()
