"""Alternate optimized 2ea2b7b baseline and compact runtime CPU fixtures."""
import json,re,subprocess,statistics
from compact_runtime import HERE,ROOT

def main():
    wd=ROOT/'tools/neck';runs=[]
    for size in ['typical','large']:
        for pair in range(3):
            for version in (['baseline','compact'] if pair%2==0 else ['compact','baseline']):
                exe=wd/('performance-compact-baseline-fast.exe' if version=='baseline' else 'performance-fast.exe')
                result=subprocess.run([str(exe)]+(['NUL','100'] if size=='large' else []),cwd=wd,capture_output=True,text=True,check=True)
                m=re.search(r'physics=([\d.]+) surface=([\d.]+)',result.stdout);assert m
                row={'size':size,'pair':pair,'version':version,'physicsMS':float(m[1]),'surfaceMS':float(m[2]),'output':result.stdout};runs.append(row)
                print(size,pair,version,row['surfaceMS'],flush=True)
    comparisons={}
    for size in ['typical','large']:
        med={v:statistics.median(r['surfaceMS'] for r in runs if r['size']==size and r['version']==v) for v in ['baseline','compact']}
        comparisons[size]={**med,'surfaceReductionPercent':100*(1-med['compact']/med['baseline'])}
    report={'baselineCommit':'2ea2b7b','method':'Three alternating pairs, 90 moving frames per process; CPU surface stage only, not in-game FPS','comparisons':comparisons,'runs':runs}
    (HERE/'performance-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(comparisons,indent=2))

if __name__=='__main__':main()
