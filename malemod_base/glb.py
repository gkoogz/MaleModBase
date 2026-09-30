"""Small glTF 2.0 binary writer for authored meshes and a semantic bind skeleton."""
import json
from pathlib import Path
import struct
import numpy as np

def write_glb(path,mesh,skeleton):
    document={'asset':{'version':'2.0','generator':'MaleModBase generic proxy'},'scene':0,'scenes':[{'nodes':[0,len(skeleton)+1]}],
              'nodes':[],'meshes':[],'skins':[],'accessors':[],'bufferViews':[],
              'materials':[{'name':'Reference ghost','doubleSided':True,'alphaMode':'BLEND',
                'pbrMetallicRoughness':{'baseColorFactor':[.32,.66,.75,.38],'metallicFactor':0,'roughnessFactor':.85}}]}
    blob=bytearray()
    def accessor(data,kind,component,target=None,bounds=False):
        data=np.ascontiguousarray(data)
        while len(blob)%4:blob.append(0)
        start=len(blob);blob.extend(data.tobytes())
        view={'buffer':0,'byteOffset':start,'byteLength':data.nbytes}
        if target:view['target']=target
        document['bufferViews'].append(view)
        item={'bufferView':len(document['bufferViews'])-1,'componentType':component,'count':len(data),'type':kind}
        if bounds:item.update(min=data.min(axis=0).tolist(),max=data.max(axis=0).tolist())
        document['accessors'].append(item);return len(document['accessors'])-1
    # Authoring is right-handed +Z up / -Y forward. glTF is +Y up: rotate X -90.
    basis=np.array([[1,0,0],[0,0,1],[0,-1,0]],dtype='<f4')
    positions=np.asarray(mesh['positions'],dtype='<f4')@basis.T
    normals=np.asarray(mesh['normals'],dtype='<f4')@basis.T
    attributes={'POSITION':accessor(positions,'VEC3',5126,34962,True),
                'NORMAL':accessor(normals,'VEC3',5126,34962),
                'TEXCOORD_0':accessor(np.asarray(mesh['uv'],dtype='<f4'),'VEC2',5126,34962),
                'JOINTS_0':accessor(np.asarray(mesh['joints'],dtype='<u2'),'VEC4',5123,34962),
                'WEIGHTS_0':accessor(np.asarray(mesh['weights'],dtype='<f4'),'VEC4',5126,34962)}
    index=accessor(np.asarray(mesh['indices'],dtype='<u4').ravel(),'SCALAR',5125,34963)
    ibm=[]
    for joint in skeleton:
        matrix=np.eye(4,dtype='<f4');matrix[:3,3]=-(basis@np.asarray(joint['position']))
        ibm.append(matrix.T.ravel()) # glTF matrices are column-major.
    ibm_index=accessor(np.array(ibm,dtype='<f4'),'MAT4',5126)
    document['nodes'].append({'name':'MaleModBase','children':[1]})
    for i,joint in enumerate(skeleton):
        parent=joint['parent'];local=np.array(joint['position'])-(np.array(skeleton[parent]['position']) if parent>=0 else 0)
        node={'name':joint['id'],'translation':(basis@local).tolist()}
        children=[k+1 for k,j in enumerate(skeleton) if j['parent']==i]
        if children:node['children']=children
        document['nodes'].append(node)
    document['nodes'].append({'name':'Generic male reference','mesh':0,'skin':0})
    document['meshes'].append({'name':'Generic reference body','primitives':[{'attributes':attributes,'indices':index,'material':0}]})
    document['skins'].append({'name':'Semantic reference rig','joints':list(range(1,len(skeleton)+1)),'skeleton':1,'inverseBindMatrices':ibm_index})
    document['buffers']=[{'byteLength':len(blob)}]
    encoded=json.dumps(document,separators=(',',':')).encode();encoded+=b' '*((-len(encoded))%4);blob+=b'\0'*((-len(blob))%4)
    result=struct.pack('<III',0x46546c67,2,12+8+len(encoded)+8+len(blob))+struct.pack('<II',len(encoded),0x4e4f534a)+encoded+struct.pack('<II',len(blob),0x004e4942)+blob
    Path(path).write_bytes(result)
