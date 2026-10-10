"""Paired x86 rebuild replay from an explicit private adapter/recipe and raw poses.

No game launch or installation. Private inputs and emitted vertex streams stay
in the selected output directory. Compare the same topology, poses and compiler.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(__doc__)
    ap.add_argument('--build', type=Path, required=True)
    ap.add_argument('--candidate-base', type=Path, required=True)
    ap.add_argument('--poses', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--vcvars', type=Path, default=Path('C:/BuildTools/VC/Auxiliary/Build/vcvars32.bat'))
    args = ap.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    native = args.build / 'source/src/runtime'
    recipe = native / 'meridian_recipe.h'
    adapter = native / 'meridian_adapter.h'
    text = adapter.read_text()
    start = '  M::CircularSection rings[7];'
    end = '  const auto raw=points;'
    if text.count(start) != 1 or text.count(end) != 1:
        raise SystemExit('Adapter preparation extraction contract changed')
    preparation = start + text.split(start)[1].split(end)[0]
    source = r'''
#include <malemod/garments/meridian_rig.hpp>
#include <malemod/garments/taut_contact.hpp>
#include "meridian_recipe.h"
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
namespace M=malemod::garments::meridian;
int main(int argc,char**argv){
 if(argc!=4)return 2;std::ifstream list(argv[1]);std::ofstream output(argv[2],std::ios::binary);std::string path;
 unsigned repeats=unsigned(std::stoul(argv[3]));
 while(std::getline(list,path)){
  std::vector<M::Vec> input(MeridianRecipe::sampleCount);std::ifstream file(path,std::ios::binary|std::ios::ate);
  if(!file||file.tellg()!=std::streamoff(input.size()*sizeof(M::Vec)))return 3;
  file.seekg(0);file.read((char*)input.data(),input.size()*sizeof(M::Vec));
  for(unsigned run=0;run<=repeats;run++){
   auto points=input;M::WrapReceipt receipt{};unsigned accepted=0;std::string reason;
   auto began=std::chrono::steady_clock::now();
   try{
@PREPARATION@
    std::vector<M::Hull> chartHulls;for(const auto& h:hulls)chartHulls.push_back(chart.Transform(h));
    for(unsigned i=0;i<MeridianRecipe::clothCount;i++)points[i]=chart.Forward(points[i]);
    M::FitSeam(points,MeridianRecipe::columns,points[MeridianRecipe::clothCount-1],liveAxis,chartHulls,.12f,6.f);
    std::vector<M::Vec> seed;
    receipt=M::WalkCertifiedTautEnvelope(points,MeridianRecipe::columns,MeridianRecipe::rows,MeridianRecipe::rowHeights,MeridianRecipe::clothFaces,MeridianRecipe::clothFaceCount,chartHulls,liveAxis,.04f,nullptr,&seed);
    for(unsigned i=0;i<MeridianRecipe::clothCount;i++){points[i]=chart.Inverse(points[i]);seed[i]=chart.Inverse(seed[i]);}
    if(!M::WithinMeridianSampling(points,seed,MeridianRecipe::columns,MeridianRecipe::rows))throw std::runtime_error("Physical sampling budget");
    accepted=1;
   }catch(const std::exception&e){reason=e.what();}
   double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
   if(!run)continue;
   output.write((char*)&accepted,sizeof(accepted));output.write((char*)&receipt,sizeof(receipt));
   unsigned length=unsigned(reason.size());output.write((char*)&length,sizeof(length));output.write(reason.data(),length);
   output.write((char*)points.data(),points.size()*sizeof(M::Vec));
   std::cout<<accepted<<"\t"<<std::setprecision(9)<<ms<<"\t"<<path<<"\t"<<reason<<"\n";
  }
 }
}
'''.replace('@PREPARATION@', preparation)
    cpp = out / 'replay.cpp'
    cpp.write_text(source)
    poses = sorted(args.poses.glob('MeridianRaw-*.bin'))
    if not poses:
        raise SystemExit('No raw poses')
    (out/'poses.txt').write_text(''.join(str(p.resolve())+'\n' for p in poses))
    # Snapshot only the changed module over the exact baseline include tree.
    shutil.copytree(args.build/'base/include', out/'candidate/include')
    changed = 'malemod/garments/taut_contact.hpp'
    shutil.copyfile(args.candidate_base/'include'/changed, out/'candidate/include'/changed)
    for label, include in [('baseline', args.build/'base/include'), ('candidate', out/'candidate/include')]:
        command = out/(label+'.cmd')
        command.write_text(f'@echo off\ncall "{args.vcvars}" >nul\ncl /nologo /O2 /MT /EHsc /std:c++17 /I"{include.resolve()}" /I"{native.resolve()}" "{cpp}" /Fo"{out/label}.obj" /Fe"{out/label}.exe"\n')
        result = subprocess.run(['cmd','/d','/c',str(command)], capture_output=True, text=True)
        (out/(label+'-compile.log')).write_text(result.stdout+result.stderr)
        result.check_returncode()
    results = []
    # Alternate execution order to expose simple thermal/background-work bias.
    for trial in range(3):
        pair = {}
        for label in (['baseline','candidate'] if trial % 2 == 0 else ['candidate','baseline']):
            binary = out/f'{label}-{trial}.bin'
            run = subprocess.run([str(out/(label+'.exe')), str(out/'poses.txt'), str(binary), str(args.repeats)], capture_output=True, text=True, check=True)
            (out/f'{label}-{trial}.tsv').write_text(run.stdout)
            rows = [line.split('\t') for line in run.stdout.splitlines()]
            times = sorted(float(r[1]) for r in rows)
            pair[label] = dict(meanMs=statistics.mean(times),p95Ms=times[min(len(times)-1,int(.95*len(times)))],maxMs=max(times),accepted=sum(int(r[0]) for r in rows),calls=len(rows),outputSHA256=sha(binary))
        pair['exactOutputEqual'] = pair['baseline']['outputSHA256'] == pair['candidate']['outputSHA256']
        results.append(pair)
    proof = dict(adapterSHA256=sha(adapter),recipeSHA256=sha(recipe),baselineHeaderSHA256=sha(args.build/'base/include'/changed),candidateHeaderSHA256=sha(out/'candidate/include'/changed),poses={str(p):sha(p) for p in poses},trials=results,nativeFPSMeasured=False)
    proof['meanReductionPercent']=100*(1-statistics.mean(p['candidate']['meanMs'] for p in results)/statistics.mean(p['baseline']['meanMs'] for p in results))
    (out/'comparison.json').write_text(json.dumps(proof,indent=2))
    print(json.dumps({k:v for k,v in proof.items() if k not in ('poses',)},indent=2))
    if not all(p['exactOutputEqual'] for p in results):
        raise SystemExit('Output or rejection parity failed')


if __name__ == '__main__':
    main()
