"""Extract the current canonical numerical declaration closure without a fork.

Uses libclang only during generation. Source expressions and declaration order
are retained. Engine observation, logging and graphics publication are adapter
boundaries; every substitution is listed in the generated manifest.
"""
import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path


def extract(args):
    args.wolverine=args.wolverine.resolve();args.base=args.base.resolve()
    sys.path.insert(0,str(args.clang))
    from clang import cindex as C
    runtime=args.wolverine.resolve()/'src/runtime'
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    envscript=out/'compiler-env.cmd'
    envscript.write_text('@echo off\ncall '+str(args.vcvars)+' >nul\nset\n')
    env=dict(s.split('=',1) for s in subprocess.check_output(['cmd.exe','/d','/c',str(envscript)],text=True).splitlines() if '=' in s and not s.startswith('='))
    flags=['-x','c++','-std=c++17','-fms-extensions','-fms-compatibility',
        '-fms-compatibility-version=19.40','-target','i686-pc-windows-msvc',
        '-Wno-invalid-token-paste','-ferror-limit=0','-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH',
        '-D_WIN32','-D_M_IX86=600','-D_MT','-D_CPPUNWIND','-D_CRT_SECURE_NO_WARNINGS']
    flags+=['-I'+p for p in env['INCLUDE'].split(';') if p]
    flags+=['-I'+str(args.wolverine/'third-party'),'-I'+str(args.base/'include'),'-I'+str(args.directx/'Include')]
    index=C.Index.create();cache=out/'canonical.ast'
    identity=json.dumps(dict(flags=flags,sources=[(str(p),p.stat().st_mtime_ns,p.stat().st_size)for p in sorted(runtime.iterdir())if p.is_file()]),sort_keys=True)
    identity_file=out/'parse-identity.json'
    if cache.is_file() and identity_file.is_file() and identity_file.read_text()==identity:
        tu=index.read(str(cache))
    else:
        tu=index.parse(str(runtime/'d3d9_proxy.cpp'),args=flags)
        tu.save(str(cache));identity_file.write_text(identity)
    errors=[str(d)for d in tu.diagnostics if d.severity>=3]
    if errors:raise RuntimeError('\n'.join(errors[:12]))
    roots={'EvaluateAnatomy','UpdateConstraintSolver','InitializeConstraintSolver','ResetCompliantDynamics',
           'ApplyControlMapping','UpdateThrob','ResetThrobClock','PrepareMenuRetargetBodyBasis','ResetPelvicAttachmentBody',
           'collisionCapsuleOverride','overrideLeftA','overrideLeftB','overrideRightA','overrideRightB'}
    skips={'teachingFluid','teachingTimeline','Fluid'}
    substitutions={
      'Log':'static void Log(const char*,...){}',
      'PerfClock':'static double PerfClock(){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();}',
      'PerfScope':'struct PerfScope {explicit PerfScope(int){}};',
      'PerfSegment':'static void PerfSegment(int,double& start){start=PerfClock();}',
      'UpdateJockstrapSource':'static void UpdateJockstrapSource(const unsigned char*){}',
      'RootJointRequested':'static bool RootJointRequested(){return sourceRootJointRequested;}',
      'CollisionCapsules':'''static bool CollisionCapsules(V3& leftA,V3& leftB,V3& rightA,V3& rightB){
       if(collisionCapsuleOverride){leftA=overrideLeftA;leftB=overrideLeftB;rightA=overrideRightA;rightB=overrideRightB;return false;}
       leftA={2.f,-7.8f,79.f};leftB={1.f,-8.2f,43.f};rightA={2.f,7.8f,79.f};rightB={1.f,8.2f,43.f};return false;
      }'''}
    kinds={C.CursorKind.FUNCTION_DECL,C.CursorKind.FUNCTION_TEMPLATE,C.CursorKind.VAR_DECL,
           C.CursorKind.STRUCT_DECL,C.CursorKind.CLASS_DECL,C.CursorKind.CLASS_TEMPLATE,
           C.CursorKind.TYPE_ALIAS_DECL,C.CursorKind.TYPEDEF_DECL,C.CursorKind.ENUM_DECL}
    selected={};pending=[];source={};rank={};counter=0;original_forwards=[];templates={}
    def eligible(c):
        if not c.location.file:return False
        p=Path(c.location.file.name).resolve()
        return p.is_relative_to(runtime)
    def top(c):
        if not eligible(c):return None
        if Path(c.location.file.name).name=='teaching_sequence.h':return None
        d=c.get_definition()
        if d and eligible(d):c=d
        template=templates.get((str(Path(c.location.file.name).resolve()),c.spelling))
        if template is not None:c=template
        while c.semantic_parent and c.semantic_parent.kind in {C.CursorKind.STRUCT_DECL,C.CursorKind.CLASS_DECL,C.CursorKind.CLASS_TEMPLATE}:
            c=c.semantic_parent
        if not c.semantic_parent or c.semantic_parent.kind not in {C.CursorKind.TRANSLATION_UNIT,C.CursorKind.NAMESPACE}:return None
        return c if c.kind in kinds and c.spelling not in skips else None
    def add(c):
        c=top(c)
        if c is None:return
        key=(str(Path(c.location.file.name).resolve()),c.extent.start.offset)
        old=selected.get(key)
        if old is None or old.extent.end.offset<c.extent.end.offset:
            selected[key]=c;pending.append(c)
    def index(c):
        nonlocal counter
        if c.kind==C.CursorKind.NAMESPACE:
            for x in c.get_children():index(x)
        elif eligible(c):
            if c.kind in {C.CursorKind.FUNCTION_TEMPLATE,C.CursorKind.CLASS_TEMPLATE}:
                templates[(str(Path(c.location.file.name).resolve()),c.spelling)]=c
            rank[(str(Path(c.location.file.name).resolve()),c.extent.start.offset)]=counter;counter+=1
            if c.kind==C.CursorKind.FUNCTION_DECL and not c.is_definition():original_forwards.append(c)
            if c.spelling in roots:add(c)
    for c in tu.cursor.get_children():index(c)
    # Template references may have been admitted before their canonical
    # declaration was encountered during the first source-order inventory.
    for c in list(selected.values()):add(c)
    found={name for name in roots if any(re.search(r'\b'+re.escape(name)+r'\b',Path(c.location.file.name).read_bytes()[c.extent.start.offset:c.extent.end.offset].decode())for c in selected.values())}
    if roots-found:raise RuntimeError('Missing numerical roots: '+str(roots-found))
    def references(c):
        ref=c.referenced
        if ref:add(ref)
        for x in c.get_children():references(x)
    seen=set()
    while pending:
        c=pending.pop();key=c.hash
        if key in seen:continue
        seen.add(key)
        if c.spelling not in substitutions:references(c)
    usrs={c.get_usr()for c in selected.values()if c.kind==C.CursorKind.FUNCTION_DECL}
    for c in original_forwards:
        if c.get_usr()in usrs:selected[(str(Path(c.location.file.name).resolve()),c.extent.start.offset)]=c
    parts=[];rows=[]
    for key,c in sorted(selected.items(),key=lambda item:(rank.get(item[0],1000000),item[0])):
        path=Path(key[0]);raw=source.setdefault(path,path.read_bytes())
        text=raw[c.extent.start.offset:c.extent.end.offset].decode('utf-8')
        if c.kind not in {C.CursorKind.FUNCTION_DECL,C.CursorKind.FUNCTION_TEMPLATE} or not c.is_definition():text+=';'
        if c.spelling in substitutions and c.is_definition():text=substitutions[c.spelling]
        namespaces=[];parent=c.semantic_parent
        while parent.kind==C.CursorKind.NAMESPACE:
            namespaces.insert(0,parent.spelling);parent=parent.semantic_parent
        wrap=lambda s:'\n'.join('namespace '+n+' {'for n in namespaces)+'\n'+s+'\n'+'}'*len(namespaces)
        # Source functions can call forward declarations that their definition
        # closure does not otherwise retain. Keep their exact original signature.
        parts.append(wrap(text))
        rows.append(dict(file=path.relative_to(args.wolverine).as_posix(),name=c.spelling,
            start=c.extent.start.offset,end=c.extent.end.offset,kind=str(c.kind),namespaces=namespaces))
    prelude=(Path(__file__).with_name('canonical_surface_prelude.inc')).read_text()
    includes=[]
    for p in sorted(runtime.iterdir()):
        if p.is_file() and p.suffix in {'.h','.cpp','.inl'}:
            includes+=re.findall(r'^\s*#include\s+<(malemod/[^>]+)>',p.read_text(errors='replace'),re.M)
    portable='\n'.join('#include <'+name+'>'for name in sorted(set(includes)))
    prelude=portable+'\n'+prelude
    teaching=(runtime/'teaching_sequence.h').read_text().replace('#pragma once','')
    fluid=(runtime/'teaching_volume.h').read_text()
    def member(signature):
        begin=fluid.index(signature);end=fluid.index('{',begin)+1;depth=1
        while depth:
            depth+=(fluid[end]=='{')-(fluid[end]=='}');end+=1
        return fluid[begin:end]
    prelude=prelude.replace('@CANONICAL_TIMELINE@',teaching).replace('@CANONICAL_EVENT_TIME@',member('static float EventTime(')).replace('@CANONICAL_LATERAL_YAW@',member('float MainLateralYaw('))
    content=prelude+'\n'+'\n'.join(parts)+'\n}\n'
    (out/'canonical-kernel.inc').write_text(content,encoding='utf-8',newline='\n')
    record=dict(schema='malemod.canonical-surface-closure/1',sourceCommit=subprocess.check_output(['git','-C',str(args.wolverine),'rev-parse','HEAD'],text=True).strip(),
        sourceFiles=[dict(path=p.relative_to(args.wolverine).as_posix(),sha256=hashlib.sha256(data).hexdigest())for p,data in sorted(source.items())],
        declarations=rows,adapterBoundaries=sorted(skips|substitutions.keys()),kernelSHA256=hashlib.sha256(content.encode()).hexdigest(),
        verifiedAgainstOracle=False)
    (out/'closure.json').write_text(json.dumps(record,indent=2)+'\n')
    print('Extracted '+str(len(rows))+' declarations from canonical source',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['wolverine','base','clang','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--vcvars',type=Path,default=Path('C:/BuildTools/VC/Auxiliary/Build/vcvars32.bat'))
    p.add_argument('--directx',type=Path,default=Path('C:/Program Files (x86)/Microsoft DirectX SDK (June 2010)'))
    extract(p.parse_args())
