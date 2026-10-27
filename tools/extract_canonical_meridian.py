"""Extract the canonical sewn-pouch build without a garment algorithm fork.

Generated geometry stays SDK-free. Adapters supply measured posed samples and
the locked authored recipe. Only timing/log statements are omitted.
"""
import argparse,hashlib,json
from pathlib import Path


def extract(source,output):
    source=Path(source);output=Path(output);output.mkdir(exist_ok=True,parents=True)
    s=(source/'meridian_adapter.h').read_text()
    a=s.index('  M::CircularSection rings[7];');b=s.index(' }catch(const std::exception& e)',a)
    geometry=s[a:b]
    removed=[line for line in geometry.splitlines()if 'preparationTimer.stage='in line or 'Log('in line]
    geometry='\n'.join(line for line in geometry.splitlines()if line not in removed)
    a=s.index(' for(unsigned i=0;i<MeridianRecipe::count;i++){\n  auto& v=vertices[i];v.color[0]=0;');b=s.index(' preparationTimer.stage="upload";',a)
    material=s[a:b]
    header='''#pragma once
#include <malemod/garments/meridian_clearance.hpp>
#include <malemod/garments/meridian_rig.hpp>
#include <malemod/garments/pouch_cage.hpp>
#include <malemod/garments/meridian_material.hpp>
namespace malemod::garments::reference {
struct Vertex {float p[3],n[3],uv[2],color[4];};
template<class Recipe> void Build(std::vector<meridian::Vec> points,std::vector<Vertex>& vertices){
 namespace M=meridian;vertices.resize(Recipe::sampleCount);
'''+(geometry+'\n'+material).replace('MeridianRecipe::','Recipe::')+'\n}\n}\n'
    (output/'canonical-meridian.inc').write_text(header)
    recipe=(source/'meridian_recipe.h').read_text().replace('namespace MeridianRecipe {','struct CanonicalMeridianRecipe {').replace('static const ','inline static const ')
    if not recipe.rstrip().endswith('}'):raise ValueError('Recipe namespace changed')
    recipe=recipe.rstrip()[:-1]+'};\n';(output/'canonical-meridian-recipe.inc').write_text(recipe)
    record=dict(schema='malemod.canonical-meridian-extraction/1',inputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in [source/'meridian_adapter.h',source/'meridian_recipe.h']},outputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in [output/'canonical-meridian.inc',output/'canonical-meridian-recipe.inc']},omittedDiagnostics=removed,numericalStatementsChanged=False,garmentDynamics=False,sourceRecipeRevision=6,nativeVerified=False)
    (output/'meridian-extraction.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();extract(a.source,a.output)
