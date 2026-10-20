"""Verify the shared pouch implementation and its independent evidence scope."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def main():
    proof=json.loads((ROOT/'provenance/pouch-cage.json').read_text())
    if proof['schema']!='malemod.pouch-cage/1' or proof['geometryBindingRevision']!=6 or proof['hashNormalization']!='LF':
        raise ValueError('Unsupported pouch cage provenance contract')
    for name,digest in proof['files'].items():
        actual=hashlib.sha256((ROOT/name).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
        if actual!=digest:raise ValueError('Pouch cage provenance differs: '+name)
    if proof['continuousContactCertified'] or proof['fullAttachmentGatePassed']:
        raise ValueError('Current sampled evidence does not establish complete contact or attachment acceptance')
    print('PASS pouch cage source hashes and sampled-evidence scope')

if __name__=='__main__':main()
