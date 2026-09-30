"""Extract the active Wolverine collar metric row, preserving numeric expressions."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'legacy/wolverine/src/runtime/unified_collar_solver.h'


def outputs():
    source = SOURCE.read_text()
    start = source.index(' float growth=Smoother01((radius-2.9f)/4.72f);')
    end = source.index('  inverse.emplace_back', start)
    raw = source[start:end]
    raw = raw.replace(' for(unsigned i=0;i<ucCount;i++){\n', '')
    raw = raw.replace('V3 p=Point(before,i),q=p-root;', 'V3 q=p-root;')
    raw = raw.replace('mask[i]=w;area[i]=(std::max)(area[i],.005);screen[i]=area[i]*(2.5+2*pow(1-w,4));',
                      'area=(std::max)(area,.005);double screen=area*(2.5+2*pow(1-w,4));')
    signature = 'inline MetricRow EvaluateMetricRow(V3 p,V3 root,V3 axis,V3 up,float radius,float length,double area){\n'
    reference = ('struct MetricRow {double mask,screen,area;};\n' + signature + raw +
                 ' return {w,screen,area};\n}\n')
    portable = ('#pragma once\n// Generated active-source metric; see provenance/collar.json.\n'
                '#include <malemod/math.hpp>\n#include <algorithm>\n#include <cmath>\n'
                'namespace malemod::collar {\nusing std::max;using std::min;\n'
                'inline float Smoother01(float value){value=max(0.f,min(1.f,value));return value*value*value*(value*(value*6.f-15.f)+10.f);}\n' +
                reference + '}\n')
    return {'include/malemod/surface/collar_field.hpp': portable,
            'tests/data/wolverine-collar-metric.inc': reference}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    files = []
    for name, value in outputs().items():
        path = ROOT / name
        if args.check:
            if path.read_bytes() != value.encode():
                raise SystemExit('Collar extraction differs: ' + name)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(value, newline='\n')
        files.append({'path': name, 'sha256': hashlib.sha256(value.encode()).hexdigest()})
    other = ['legacy/wolverine/src/runtime/unified_collar_data.h',
             'legacy/wolverine/src/runtime/pelvic_attachment.h',
             'legacy/wolverine/src/runtime/anatomy_surface.h',
             'malemod_base/collar.py', 'malemod_base/graft_collar.py',
             'malemod_base/surface_limit.py', 'legacy/wolverine/src/runtime/surface_limit.h',
             'tests/test_graft_collar.py', 'tests/data/wolverine-collar-metric.csv',
             'tests/collar_test.cpp', 'modules/pelvic-collar.json']
    report = {'schemaVersion': 1, 'sourceCommit': json.loads((ROOT / 'provenance/wolverine.json').read_text())['commit'],
              'source': SOURCE.relative_to(ROOT).as_posix(), 'sourceSHA256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              'files': files,
              'relatedFiles': [{'path': name, 'sha256': hashlib.sha256((ROOT / name).read_bytes()).hexdigest()} for name in other],
              'scope': 'Exact scalar metric extraction and source-derived offline coupled collar evaluator; not installed runtime parity',
              'omissions': ['packed game buffers and palette mappings', 'Raphe guide authoring/runtime',
                            'live pose/contact integration', 'source custom SIMD LDLT path',
                            '8-frame sequence phase correction', 'packed normal/tangent uploads'],
              'units': 'Uncalibrated source model units. A target must supply measured length scale and authoring frame.'}
    text = json.dumps(report, indent=2) + '\n'
    destination = ROOT / 'provenance/collar.json'
    if args.check:
        if destination.read_bytes() != text.encode():
            raise SystemExit('Collar provenance differs')
    else:
        destination.write_text(text, newline='\n')
    print('PASS: collar extraction and provenance.' if args.check else 'Extracted active collar metric.')


if __name__ == '__main__':
    main()
