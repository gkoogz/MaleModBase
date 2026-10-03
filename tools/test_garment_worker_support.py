"""Production worker garment-force proof, each scenario in a fresh process."""
import argparse,hashlib,json,subprocess
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--runtime',type=Path,required=True);p.add_argument('--surface-runtime',type=Path,required=True);p.add_argument('--collar-fixtures',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    report={'observedGameplay':False,'wireVersion':5,'accelerationConvention':'calibrated source-local length units/time squared','supportFractionCap':.15,'runtimeSHA256':hashlib.sha256(a.runtime.read_bytes()).hexdigest(),'disabledCases':[],'supportCases':[]}
    for state in range(3):
        packets=[];mechanics=[]
        for mode in range(3):
            dest=a.output/f'state{state}-mode{mode}.wire'
            result=subprocess.run([str(a.runtime.resolve()),str(mode),str(state),str(dest.resolve())],check=True,capture_output=True,text=True)
            packets.append(dest.read_bytes());mechanics.append([float(x) for x in result.stdout.split()])
        assert packets[0]==packets[2],(state,'disabled nonzero support changed source replay')
        assert packets[0]!=packets[1],(state,'enabled support did not reach production integration')
        report['supportCases'].append({'state':state,'disabledTipAndLobeHeights':mechanics[0],'enabledTipAndLobeHeights':mechanics[1],'disabledByteIdenticalWithNonzeroIgnoredForces':True,'enabledOutputChanged':True})
    # Existing revision-2 numerical fixtures prove the new disabled integration
    # branch preserves the established geometry and mechanics byte-for-byte.
    for label in ['default','requested-s0-a10','requested-s2-a64','combined-s1-a100']:
        dest=a.output/(label+'.disabled');prefs=a.collar_fixtures/(label+'.txt')
        subprocess.run([str(a.surface_runtime.resolve()),str(prefs.resolve()),str(dest.resolve()),'120'],check=True,stdout=subprocess.DEVNULL)
        for suffix in ['', '.body0','.body1','.normals','.tangents','.uv','.indices','.mechanics.json']:
            assert Path(str(dest)+suffix).read_bytes()==Path(str(a.collar_fixtures/(label+'.current'))+suffix).read_bytes(),(label,suffix,'disabled baseline changed')
        report['disabledCases'].append({'label':label,'steps':120,'allGeometryLightingTopologyUVMechanicsByteIdentical':True})
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
