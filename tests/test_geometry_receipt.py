"""Final delivery must remeasure actual geometry, not trust copied PASS flags."""
import copy,itertools,json,math,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base import garment_geometry_audit as g

class GeometryReceiptTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name).resolve();self.build=self.root/'build';self.build.mkdir()
        self.pin='a'*40
        for relative in ['include/malemod/garments/header.hpp','tools/audit_garment_matrix.py','tools/audit_surface_intersections.cpp','malemod_base/garment_geometry_audit.py','tests/test_garment_geometry_audit.py']:
            p=self.root/relative;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(relative)
        self.exe=self.build/'audit.exe';self.exe.write_bytes(b'independently tested binary')
        self.header=self.write('headers.json',dict(sourceRoot=str(self.root),commit=self.pin,cleanPin=True,headers=[self.ref(self.root/'include/malemod/garments/header.hpp')]))
        self.fixture=self.write('fixtures.json',dict(crossingAndClosureFixturesPassed=True,crossingExitCode=0,closureExitCode=0,artifacts={
            'intersectionExecutable':self.ref(self.exe),'intersectionSource':self.ref(self.root/'tools/audit_surface_intersections.cpp'),
            'closureAuditSource':self.ref(self.root/'malemod_base/garment_geometry_audit.py'),'closureTestSource':self.ref(self.root/'tests/test_garment_geometry_audit.py')}))
        vertices=[[float(c),float(r),0.]for r in range(3)for c in range(3)]
        faces=[]
        for r in range(2):
            for c in range(2):
                a=r*3+c;faces.extend([[a,a+1,a+3],[a+1,a+4,a+3]])
        ribbons=[]
        for side in range(4):
            start=len(vertices)
            for r in range(3):
                x=float(side*3);y=float(r)
                vertices.extend([[x-.1,y,.1],[x+.1,y,.1],[x+.1,y,-.1],[x-.1,y,-.1]])
            ribbons.append(dict(start=start,sections=3,corners=4))
        self.obj=self.build/'actual.obj';self.obj.write_text('\n'.join('v '+' '.join(map(str,p))for p in vertices)+'\n'+'\n'.join('f '+' '.join(str(v+1)for v in f)for f in faces)+'\n')
        names=[f'hem-{side}-{end}'for side in (0,1)for end in ('top','bottom')]+[f'strap-{side}-bottom'for side in (0,1)]
        joints=[dict(name=n,a=dict(vertices=[0],weights=[1.]),b=dict(vertices=[0],weights=[1.]),restOffset=[0.,0.,0.],offsetProvenance='authored-thickness-contract')for n in names]
        self.layout=self.write('layout.json',dict(sheet=dict(start=0,rows=2,columns=2,faceStart=0,faceCount=8),straps=ribbons[:2],sideHems=ribbons[2:],sideBoundary=[[0,3,6],[2,5,8]],measuredCircumference=1.,bandThicknessNormalized=.003,authoredJoints=joints))
        self.geometry_path=self.build/'geometry.json';self.geometry=g.audit(self.obj,self.layout,self.geometry_path)
        self.crossing=dict(sheetCrossings=0,sheetCoplanarOverlaps=0,degenerateFaceCount=0,intersectionTolerance=1e-9,twiceAreaDegeneracyThreshold=1e-12,coplanarOverlapAreaThreshold=1e-12)
        self.crossing_path=self.write('crossing.json',self.crossing)
        self.receipt=dict(schema=2,expectedCases=72,matrix=dict(families=list(g.FAMILIES),overall=list(g.SIZES),angles=list(g.ANGLES),states=list(g.STATES)),baseCommit=self.pin,allCasesPassed=True,cleanPinVerified=True,headerManifest=self.ref(self.header),independentFixtureProof=self.ref(self.fixture),auditTool=self.ref(self.root/'tools/audit_garment_matrix.py'),intersectionAuditor=self.ref(self.exe),cases=[])
        for i,key in enumerate(itertools.product(g.FAMILIES,g.SIZES,g.ANGLES,g.STATES)):
            state=dict(zip(('family','overall','angle','state'),key));sample=self.write(f'sampler{i}.json',state)
            inp=self.build/f'input{i}';inp.write_bytes(str(key).encode())
            values=dict(input=self.ref(inp),meshOBJ=self.ref(self.obj),layout=self.ref(self.layout),baseHeaderManifest=self.ref(self.header),auditTool=self.receipt['auditTool'],samplerBinding=self.ref(sample))
            export=self.write(f'export{i}.json',dict(state,artifacts={n:values[n]for n in ('input','meshOBJ','layout','baseHeaderManifest')}));values['sourceExportReceipt']=self.ref(export)
            self.receipt['cases'].append(dict(state,status='PASS',failures=[],pending=[],artifacts=values,detailArtifacts=dict(geometry=self.ref(self.geometry_path),intersections=self.ref(self.crossing_path)),sheetCrossings=0,sheetCoplanarOverlaps=0,degenerateFaces=0,maxStrapTurnRadians=0.,maxHemTurnRadians=0.,hemJunctionGapNormalized=0.,authoredJointAudit=self.geometry['authoredJointAudit'],measuredTolerances=dict(maxStrapTurnRadians=g.MAX_TURN,maxHemTurnRadians=g.MAX_TURN,maxAuthoredSeamResidualNormalized=.003/4,**{n:self.crossing[n]for n in ('intersectionTolerance','twiceAreaDegeneracyThreshold','coplanarOverlapAreaThreshold')})))
        self.path=self.build/'proof.json'
    def ref(self,p):return dict(path=str(p.resolve()),sha256=g.digest(p))
    def write(self,n,v):
        p=self.build/n;p.write_text(json.dumps(v));return p
    def run_gate(self,dirty=False):
        self.path.write_text(json.dumps(self.receipt))
        def git(args,**kwargs):return ((' M include/header.hpp'if dirty else '') if 'status'in args else self.pin)+'\n'
        def intersection(args,**kwargs):Path(args[-1]).write_text(json.dumps(self.crossing))
        with patch.object(g.subprocess,'check_output',side_effect=git),patch.object(g.subprocess,'run',side_effect=intersection):
            return g.validate_geometry_receipt(self.path,self.root,self.pin)
    def test_complete_actual_geometry_passes_and_returns_all_bound_files(self):
        paths=self.run_gate();self.assertIn(self.obj,paths);self.assertIn(self.path,paths);self.assertEqual(len(self.receipt['cases']),72)
    def test_missing_case_duplicate_or_wrong_ui_rejected(self):
        for mutate in (lambda r:r['cases'].pop(),lambda r:r['cases'].__setitem__(1,copy.deepcopy(r['cases'][0])),lambda r:r['cases'][0].__setitem__('angle',0)):
            old=copy.deepcopy(self.receipt);mutate(self.receipt)
            with self.assertRaises(ValueError):self.run_gate()
            self.receipt=old
    def test_stale_headers_mesh_or_fixture_rejected(self):
        for p in (self.obj,self.root/'include/malemod/garments/header.hpp',self.root/'tests/test_garment_geometry_audit.py'):
            old=p.read_bytes();p.write_bytes(old+b'changed')
            with self.assertRaises(ValueError):self.run_gate()
            p.write_bytes(old)
    def test_dirty_or_stale_pin_rejected(self):
        with self.assertRaises(ValueError):self.run_gate(dirty=True)
        self.receipt['baseCommit']='b'*40
        with self.assertRaises(ValueError):self.run_gate()
    def test_false_pass_row_rejected(self):
        self.receipt['cases'][0]['maxHemTurnRadians']=.01
        with self.assertRaisesRegex(ValueError,'falsely reports'):self.run_gate()
    def test_crossing_overlap_and_degenerate_cannot_be_hidden_by_pass(self):
        for name in ('sheetCrossings','sheetCoplanarOverlaps','degenerateFaceCount'):
            self.crossing[name]=1;self.crossing_path.write_text(json.dumps(self.crossing))
            for row in self.receipt['cases']:row['detailArtifacts']['intersections']=self.ref(self.crossing_path)
            with self.assertRaisesRegex(ValueError,'crossings'):self.run_gate()
            self.crossing[name]=0
    def test_uncertified_seams_not_promoted_by_receipt(self):
        layout=json.loads(self.layout.read_text());layout.pop('authoredJoints');self.layout.write_text(json.dumps(layout))
        self.geometry=g.audit(self.obj,self.layout,self.geometry_path)
        for row in self.receipt['cases']:
            row['artifacts']['layout']=self.ref(self.layout)
            exported=Path(row['artifacts']['sourceExportReceipt']['path']);v=json.loads(exported.read_text());v['artifacts']['layout']=self.ref(self.layout);exported.write_text(json.dumps(v));row['artifacts']['sourceExportReceipt']=self.ref(exported)
            row['detailArtifacts']['geometry']=self.ref(self.geometry_path)
        with self.assertRaisesRegex(ValueError,'uncertified'):self.run_gate()
    def test_sampler_actual_ui_mismatch_rejected(self):
        row=self.receipt['cases'][0];p=Path(row['artifacts']['samplerBinding']['path']);v=json.loads(p.read_text());v['state']=2;p.write_text(json.dumps(v));row['artifacts']['samplerBinding']=self.ref(p)
        with self.assertRaisesRegex(ValueError,'UI state differs'):self.run_gate()
    def test_reauditor_detects_forged_raw_geometry_report(self):
        v=json.loads(self.geometry_path.read_text());v['routes'][0]['maxCenterlineTurnDegrees']=0.001;self.geometry_path.write_text(json.dumps(v))
        for row in self.receipt['cases']:row['detailArtifacts']['geometry']=self.ref(self.geometry_path)
        with self.assertRaisesRegex(ValueError,'measurements do not match'):self.run_gate()
    def test_frozen_hem_threshold_cannot_be_relaxed(self):
        self.receipt['cases'][0]['measuredTolerances']['maxHemTurnRadians']=math.pi
        with self.assertRaisesRegex(ValueError,'relaxed'):self.run_gate()
    def refresh_geometry_exports(self):
        self.geometry=g.audit(self.obj,self.layout,self.geometry_path)
        for row in self.receipt['cases']:
            row['artifacts']['meshOBJ']=self.ref(self.obj)
            exported=Path(row['artifacts']['sourceExportReceipt']['path']);v=json.loads(exported.read_text());v['artifacts']['meshOBJ']=self.ref(self.obj);exported.write_text(json.dumps(v));row['artifacts']['sourceExportReceipt']=self.ref(exported)
            row['detailArtifacts']['geometry']=self.ref(self.geometry_path)
    def test_actual_winding_remeasured_not_trusted(self):
        lines=self.obj.read_text().splitlines();idx=next(i for i,l in enumerate(lines)if l=='f 2 5 4');lines[idx]='f 2 4 5';self.obj.write_text('\n'.join(lines)+'\n');self.refresh_geometry_exports()
        with self.assertRaisesRegex(ValueError,'winding'):self.run_gate()
    def test_actual_hem_bend_remeasured_not_trusted(self):
        lines=self.obj.read_text().splitlines();layout=json.loads(self.layout.read_text());start=layout['sideHems'][0]['start']+8
        for i in range(start,start+4):
            xyz=list(map(float,lines[i].split()[1:]));xyz[0]+=3;lines[i]='v '+' '.join(map(str,xyz))
        self.obj.write_text('\n'.join(lines)+'\n');self.refresh_geometry_exports()
        with self.assertRaisesRegex(ValueError,'smoothness'):self.run_gate()

if __name__=='__main__':unittest.main()
