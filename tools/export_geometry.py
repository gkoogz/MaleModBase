"""Extract source arrays and reference surfaces without a game or graphics SDK."""
import ast
import hashlib
import json
from pathlib import Path
import re
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'legacy/wolverine/src/runtime'
OUT = ROOT / 'assets/wolverine-reference'
ARRAY = re.compile(r'static\s+(?:const|constexpr)\s+([\w ]+?)\s+(\w+)\[([^\]]+)\]\s*=\s*\{([^{}]*?)\};',re.S)
DTYPES = {'float':'<f4','double':'<f8','unsigned char':'u1','unsigned short':'<u2',
          'unsigned':'<u4','unsigned int':'<u4','UINT':'<u4','int':'<i4','short':'<i2'}

def dimension(expression, constants):
    node = ast.parse(expression,mode='eval').body
    def value(n):
        if isinstance(n,ast.Constant) and type(n.value) is int: return n.value
        if isinstance(n,ast.Name): return constants[n.id]
        if isinstance(n,ast.BinOp) and isinstance(n.op,ast.Mult): return value(n.left)*value(n.right)
        if isinstance(n,ast.BinOp) and isinstance(n.op,ast.Add): return value(n.left)+value(n.right)
        raise ValueError(f'Unsupported array dimension: {expression}')
    return value(node)

def arrays(path):
    text = path.read_text()
    constants = {name:int(number) for name,number in re.findall(r'\b(\w+)\s*=\s*(\d+)\s*[,;]',text)}
    for match in ARRAY.finditer(text):
        typ,name,expression,body = match.groups()
        typ = typ.strip()
        if typ not in DTYPES: continue
        try:
            count = dimension(expression,constants)
        except (KeyError, ValueError):
            continue  # Runtime-dependent declarations remain in the exact snapshot.
        body = re.sub(r'//[^\n]*|/\*.*?\*/','',body,flags=re.S)
        body = re.sub(r'(?<=[\d.])[fFuUlL]+\b','',body)
        tokens = [t.strip() for t in body.split(',') if t.strip()]
        literal = r'[+-]?(?:0x[0-9a-fA-F]+|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)'
        if any(re.fullmatch(literal,t) is None for t in tokens):
            continue  # Symbolic initializers cannot be exported as literal arrays.
        # Parse hexadecimal byte tables as well as decimal float declarations.
        if typ in {'float','double'}:
            data = np.array([float(t) for t in tokens],dtype=DTYPES[typ])
        else:
            data = np.array([int(t,16) if '0x' in t.lower() else int(t) for t in tokens],dtype=DTYPES[typ])
        if len(data) != count: raise ValueError(f'{path.name}:{name}: {len(data)} != {count}')
        if not np.isfinite(data).all(): raise ValueError(f'Nonfinite array: {name}')
        yield name, data, expression

def write_obj(path, points, indices, uv=None):
    points = np.asarray(points,dtype=np.float32).reshape(-1,3)
    faces = np.asarray(indices).reshape(-1,3)[:,[0,2,1]]
    if faces.max() >= len(points): raise ValueError('Face outside vertex range')
    with path.open('w',newline='\n') as file:
        file.write('# Source reference geometry; uncalibrated model units; reversed source winding.\n')
        for p in points: file.write('v '+' '.join(format(float(x),'.9g') for x in p)+'\n')
        if uv is not None:
            if len(uv) != len(points): raise ValueError('UV count mismatch')
            for t in uv: file.write('vt '+' '.join(format(float(x),'.9g') for x in t)+'\n')
        for tri in faces+1:
            file.write('f '+' '.join(f'{i}/{i}' if uv is not None else str(i) for i in tri)+'\n')

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    bank, inventory, skipped = {}, [], []
    # Every available one-dimensional numerical table, including authored morphs.
    for path in sorted(RUNTIME.glob('*.h')):
        found = list(arrays(path))
        for name,data,expression in found:
            key = path.stem+'__'+name
            bank[key] = data
            inventory.append({'key':key,'source':path.name,'symbol':name,'dtype':data.dtype.str,
                              'count':len(data),'declarationDimension':expression,
                              'sha256':hashlib.sha256(data.tobytes()).hexdigest()})
        if re.search(r'\w+\s*\[[^\]]+\]\s*\[',path.read_text()):
            skipped.append(path.name)
    get = lambda header,name: bank[header+'__'+name]
    base = get('r14_asset','r14Base').reshape(-1,3)
    support = get('pouch_surface_data','cpReference').reshape(-1,3)
    direct = get('neck_render_data','nrDirect')
    rows = get('neck_render_data','nrRows')
    sources = get('neck_render_data','nrSources')
    weights = get('neck_render_data','nrWeights')
    if len(rows) != len(direct)+1 or rows[-1] != len(sources) or len(weights) != len(sources):
        raise ValueError('Invalid sparse binding dimensions')
    if sources.max() >= len(support): raise ValueError('Binding source out of bounds')
    if np.any(np.diff(rows.astype(np.int64)) < 0): raise ValueError('Nonmonotonic binding rows')
    points = np.empty((len(direct),3),dtype='<f4')
    for i,source in enumerate(direct):
        if source != 65535:
            points[i] = support[source]
        else:
            start,end = int(rows[i]),int(rows[i+1])
            if start == end: raise ValueError('Unbound final vertex')
            # Match source float32 accumulation order.
            points[i] = 0
            for j in range(start,end): points[i] += support[sources[j]]*weights[j]
    # Packed UV is the final four bytes of each 20-byte attribute record.
    attributes = get('neck_render_data','nrAttributes').reshape(-1,20).copy()
    support_uv = np.empty((len(support),2),dtype='<f4')
    support_uv[:len(base)] = get('r14_asset','r14UV').reshape(-1,2).astype('<f2').astype('<f4')
    fine_source = get('rounded_render_data','rsFineSource').reshape(-1,3)
    fine_bary = get('rounded_render_data','rsFineBary').reshape(-1,3)
    if len(base)+len(fine_source) != len(support): raise ValueError('Support UV bindings mismatch')
    source_uv = get('r14_asset','r14UV').reshape(-1,2)
    for k,(tri,w) in enumerate(zip(fine_source,fine_bary)):
        support_uv[len(base)+k] = (source_uv[tri[0]]*w[0]+source_uv[tri[1]]*w[1]+source_uv[tri[2]]*w[2]).astype('<f2').astype('<f4')
    retained = direct != 65535
    uv = attributes[:,16:20].copy().view('<f2').astype('<f4')
    uv[retained] = support_uv[direct[retained]]
    indices = get('neck_render_data','nrIndices')
    if not np.isfinite(points).all() or not np.isfinite(uv).all(): raise ValueError('Invalid final geometry')
    write_obj(OUT/'coarse-sculpt.obj',base,get('r14_asset','r14Indices'),get('r14_asset','r14UV').reshape(-1,2))
    write_obj(OUT/'support-reference.obj',support,get('rounded_render_data','rsIndices'))
    write_obj(OUT/'final-reference.obj',points,indices,uv)
    bank['derived__final_reference_positions'] = points
    bank['derived__final_reference_uv'] = uv
    np.savez_compressed(OUT/'geometry.npz',**bank)
    meshes = [{'file':name,'vertices':vertices,'triangles':triangles,'sha256':hashlib.sha256((OUT/name).read_bytes()).hexdigest()}
              for name,vertices,triangles in [('coarse-sculpt.obj',len(base),len(get('r14_asset','r14Indices'))//3),
              ('support-reference.obj',len(support),len(get('rounded_render_data','rsIndices'))//3),
              ('final-reference.obj',len(points),len(indices)//3)]]
    report = {'contractVersion':1,'sourceCommit':json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
              'units':'uncalibrated-source-model-units','referenceOnly':True,'meshes':meshes,'arrays':inventory,
              'multidimensionalTablesRetainedInSource':skipped,
              'unresolved':['native skeleton and palette','material mapping','live solver evaluation']}
    (OUT/'manifest.json').write_text(json.dumps(report,indent=2)+'\n',newline='\n')
    print(f'Exported {len(bank)} arrays and three reference meshes; final topology {len(points)} vertices / {len(indices)//3} triangles.')

if __name__ == '__main__': main()
