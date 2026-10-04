"""Measure actual exported material-grid cells; no visual or dynamic approval."""
import argparse,json,hashlib
import itertools,math,subprocess,tempfile
from pathlib import Path
import numpy as np

FAMILIES = ("geralt", "wolverine")
SIZES = (25, 50, 75, 100)
ANGLES = (1, 50, 100)
STATES = (0, 1, 2)
MAX_TURN = math.pi / 6
REQUIRED_ARTIFACTS = ("input", "sourceExportReceipt", "samplerBinding", "meshOBJ", "layout", "baseHeaderManifest", "auditTool")

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def artifact(value):
    path = Path(value["path"])
    if not path.is_absolute() or not path.is_file():
        raise ValueError("Artifact is not an existing absolute file: " + str(path))
    actual = digest(path)
    if actual != value["sha256"]:
        raise ValueError("Artifact changed: " + str(path))
    return {"path": str(path.resolve()), "sha256": actual}

def validate_geometry_receipt(receipt_path, source_root, expected_pin):
    """Recheck the complete actual matrix, not its asserted PASS flags.

    Reuses the same geometry auditor and independently tested intersection
    executable as the matrix exporter. All raw measurements are recomputed.
    Returns every bound artifact for inclusion in an adapter delivery receipt.
    This source gate establishes neither dynamics nor installed gameplay.
    """
    root = Path(source_root).resolve()
    path = Path(receipt_path).resolve()
    if not path.is_relative_to(root / "build") or not path.is_file():
        raise ValueError("Geometry receipt must exist under the pinned Base build directory")
    receipt = json.loads(path.read_text())
    def require(condition, message):
        if not condition:
            raise ValueError(message)
    matrix = dict(families=list(FAMILIES), overall=list(SIZES), angles=list(ANGLES), states=list(STATES))
    require(receipt.get("schema") == 2 and receipt.get("expectedCases") == 72 and receipt.get("matrix") == matrix,
            "Geometry receipt does not declare the frozen 72-case actual UI matrix")
    require(receipt.get("allCasesPassed") is True and receipt.get("cleanPinVerified") is True,
            "Geometry receipt is incomplete or not clean-pinned")
    require(receipt.get("baseCommit") == expected_pin, "Geometry receipt names a stale Base pin")
    actual_pin = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    dirty = subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"], text=True).strip()
    require(actual_pin == expected_pin and not dirty, "Geometry Base checkout is not the exact clean current pin")
    paths = [path]
    def bound(value):
        verified = artifact(value)
        paths.append(Path(verified["path"]))
        return verified
    header = bound(receipt["headerManifest"])
    metadata = json.loads(Path(header["path"]).read_text())
    require(Path(metadata["sourceRoot"]).resolve() == root and metadata.get("cleanPin") is True and metadata.get("commit") == expected_pin,
            "Geometry header manifest is not the clean pinned Base source")
    records = metadata["headers"]
    recorded = {Path(v["path"]).resolve(): bound(v)["sha256"] for v in records}
    current = {p.resolve(): digest(p) for p in (root / "include/malemod/garments").rglob("*") if p.is_file()}
    require(bool(current) and len(recorded) == len(records) and recorded == current,
            "Geometry header manifest is incomplete or stale")
    tool = bound(receipt["auditTool"])
    require(tool == dict(path=str(root / "tools/audit_garment_matrix.py"), sha256=digest(root / "tools/audit_garment_matrix.py")),
            "Geometry matrix exporter differs from current pinned source")
    fixture_record = bound(receipt["independentFixtureProof"])
    fixture = json.loads(Path(fixture_record["path"]).read_text())
    require(fixture.get("crossingAndClosureFixturesPassed") is True and fixture.get("crossingExitCode") == 0 and fixture.get("closureExitCode") == 0,
            "Independent crossing and closure fixture proof did not pass")
    fixture_artifacts = {name: bound(v) for name, v in fixture["artifacts"].items()}
    executable = bound(receipt["intersectionAuditor"])
    require(fixture_artifacts["intersectionExecutable"] == executable, "Geometry auditor binary is not independently tested")
    for name, relative in (("intersectionSource", "tools/audit_surface_intersections.cpp"),
                           ("closureAuditSource", "malemod_base/garment_geometry_audit.py"),
                           ("closureTestSource", "tests/test_garment_geometry_audit.py")):
        require(fixture_artifacts[name] == dict(path=str(root / relative), sha256=digest(root / relative)),
                "Independent geometry fixture source is stale: " + name)
    keys = set(itertools.product(FAMILIES, SIZES, ANGLES, STATES))
    cases = receipt["cases"]
    seen = set()
    require(len(cases) == len(keys), "Geometry receipt lacks 72 actual cases")
    with tempfile.TemporaryDirectory(prefix="malemod-geometry-verify-") as temporary:
        for index, row in enumerate(cases):
            key = tuple(row.get(n) for n in ("family", "overall", "angle", "state"))
            require(all(type(row.get(n)) is int for n in ("overall", "angle", "state")) and key in keys and key not in seen,
                    "Duplicate, relabeled or invalid actual geometry UI case")
            seen.add(key)
            require(row.get("status") == "PASS" and row.get("failures") == [] and row.get("pending") == [],
                    "Actual geometry case is not accepted: " + str(key))
            values = row["artifacts"]
            require(all(name in values for name in REQUIRED_ARTIFACTS), "Missing actual geometry source artifacts")
            verified = {name: bound(value) for name, value in values.items()}
            require(verified["baseHeaderManifest"] == header and verified["auditTool"] == tool,
                    "Actual geometry case used stale headers or auditor")
            for name in ("samplerBinding", "sourceExportReceipt"):
                document = json.loads(Path(verified[name]["path"]).read_text())
                require(all(document.get(n) == row[n] and type(document.get(n)) is type(row[n]) for n in ("family", "overall", "angle", "state")),
                        "Actual source sampler/export receipt UI state differs")
                if name == "sourceExportReceipt":
                    require(document.get("exportExitCode", 0) == 0, "Actual geometry export failed")
                    for kind in ("input", "meshOBJ", "layout", "baseHeaderManifest"):
                        require(document.get("artifacts", {}).get(kind) == verified[kind], "Actual source export artifact differs: " + kind)
            details = {name: bound(value) for name, value in row["detailArtifacts"].items()}
            original_geometry = json.loads(Path(details["geometry"]["path"]).read_text())
            original_crossing = json.loads(Path(details["intersections"]["path"]).read_text())
            geometry = audit(verified["meshOBJ"]["path"], verified["layout"]["path"], Path(temporary) / f"{index}-geometry.json")
            require(geometry == original_geometry, "Geometry measurements do not match actual exported mesh/layout")
            topology = json.loads(Path(verified["layout"]["path"]).read_text())
            sheet = topology["sheet"]
            crossing_path = Path(temporary) / f"{index}-crossing.json"
            subprocess.run([executable["path"], verified["meshOBJ"]["path"], str(sheet["faceStart"]), str(sheet["faceCount"]), str(crossing_path)],
                           check=True, capture_output=True, text=True)
            crossing = json.loads(crossing_path.read_text())
            require(crossing == original_crossing, "Intersection measurements do not match actual exported mesh")
            require(all(type(crossing.get(n)) is int and crossing[n] == 0 for n in ("sheetCrossings", "sheetCoplanarOverlaps", "degenerateFaceCount")) and geometry["windingConflictCount"] == 0,
                    "Actual garment has crossings, overlap, degeneracy or inconsistent winding")
            turns = {kind: [math.radians(r["maxCenterlineTurnDegrees"]) for r in geometry["routes"] if r["kind"] == kind] for kind in ("strap", "hem")}
            require(all(len(v) == 2 and all(math.isfinite(t) and 0 <= t < MAX_TURN for t in v) for v in turns.values()),
                    "Actual straps or hems exceed frozen smoothness limit")
            closure = geometry["authoredJointAudit"]
            thickness = topology["bandThicknessNormalized"]
            tolerance = thickness / 4
            require(math.isfinite(tolerance) and tolerance > 0 and closure.get("certified") is True and closure.get("passed") is True and
                    closure.get("toleranceNormalized") == tolerance and math.isfinite(closure["maximumResidualNormalized"]) and 0 <= closure["maximumResidualNormalized"] <= tolerance,
                    "Actual authored finite seams are uncertified or exceed bandThickness/4")
            measured = row["measuredTolerances"]
            require(measured.get("maxStrapTurnRadians") == MAX_TURN and measured.get("maxHemTurnRadians") == MAX_TURN and measured.get("maxAuthoredSeamResidualNormalized") == tolerance,
                    "Geometry receipt relaxed its frozen strap/hem/seam tolerances")
            expected = dict(sheetCrossings=0, sheetCoplanarOverlaps=0, degenerateFaces=0,
                            maxStrapTurnRadians=max(turns["strap"]), maxHemTurnRadians=max(turns["hem"]),
                            hemJunctionGapNormalized=closure["maximumResidualNormalized"], authoredJointAudit=closure)
            require(all(row.get(n) == v for n, v in expected.items()), "Geometry receipt falsely reports actual measurements")
            for n in ("intersectionTolerance", "twiceAreaDegeneracyThreshold", "coplanarOverlapAreaThreshold"):
                require(measured.get(n) == crossing[n], "Geometry receipt changed actual intersection tolerances")
    require(seen == keys, "Geometry receipt lacks exact 72 actual UI states")
    return list(dict.fromkeys(paths))

def measure_authored_joints(vertices, declarations, circumference, band_thickness):
    """Measure current sewn points against externally authored finite offsets.

    Rest offsets must come from the thickness/seam contract, never be inferred
    from this fitted output. Missing declarations remain uncertified.
    """
    if not declarations:
        return dict(certified=False, reason="No authored finite-offset joint declarations", maximumResidualNormalized=None, toleranceNormalized=band_thickness / 4, joints=[])
    if not np.isfinite(circumference) or circumference <= 0 or not np.isfinite(band_thickness) or band_thickness <= 0:
        raise ValueError("Invalid measured seam scale")
    vertices = np.asarray(vertices, dtype=float)
    def sample(binding):
        raw_ids = np.asarray(binding["vertices"])
        if raw_ids.ndim != 1 or not np.issubdtype(raw_ids.dtype, np.integer):
            raise ValueError("Declared seam vertices must be actual integer indices")
        ids = raw_ids.astype(int)
        weights = np.asarray(binding["weights"], dtype=float)
        if weights.ndim != 1 or ids.size != weights.size or not 1 <= ids.size <= 4 or (ids < 0).any() or (ids >= len(vertices)).any() or not np.isfinite(weights).all() or (weights < 0).any() or abs(weights.sum() - 1) > 1e-9:
            raise ValueError("Invalid declared seam attachment")
        return (vertices[ids] * weights[:, None]).sum(axis=0)
    measured = []
    for joint in declarations:
        if joint.get("offsetProvenance") != "authored-thickness-contract":
            raise ValueError("Captured fitted gaps cannot authorize sewn rest offsets")
        offset = np.asarray(joint["restOffset"], dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("Invalid declared rest seam offset")
        current = sample(joint["a"]) - sample(joint["b"])
        measured.append(dict(name=joint["name"], rawGapNormalized=float(np.linalg.norm(current) / circumference), authorizedOffsetNormalized=(offset / circumference).tolist(), residualNormalized=float(np.linalg.norm(current - offset) / circumference)))
    names = {j["name"] for j in measured}
    if len(names) != len(measured):
        raise ValueError("Duplicate authored seam correspondence")
    required = {f"hem-{side}-{end}" for side in (0, 1) for end in ("top", "bottom")} | {f"strap-{side}-bottom" for side in (0, 1)}
    missing = sorted(required - names)
    maximum = max(j["residualNormalized"] for j in measured)
    return dict(certified=not missing, reason="Missing authored joint correspondences: " + ",".join(missing) if missing else "All authored seam correspondences measured", maximumResidualNormalized=maximum, toleranceNormalized=band_thickness / 4, passed=not missing and maximum <= band_thickness / 4, joints=measured)


def audit(obj,layout,report):
    obj,layout,report=Path(obj),Path(layout),Path(report);vertices=[];faces=[]
    for line in obj.read_text().splitlines():
        fields=line.split()
        if fields and fields[0]=='v':vertices.append([float(x)for x in fields[1:4]])
        elif fields and fields[0]=='f':faces.append([int(x.split('/')[0])-1 for x in fields[1:4]])
    v,f=np.asarray(vertices),np.asarray(faces);typed=json.loads(layout.read_text());g=typed['sheet'];start,rows,cols=g['start'],g['rows'],g['columns'];grid=v[start:start+(rows+1)*(cols+1)].reshape(rows+1,cols+1,3)
    def point(i):return dict(vertex=int(i),row=int((i-start)//(cols+1)),column=int((i-start)%(cols+1)),position=v[i].tolist())
    def edges(axis):
        lengths=np.linalg.norm(np.diff(grid,axis=axis),axis=2);ids=np.argsort(lengths,axis=None)[:20];samples=[]
        for at in ids:
            r,c=np.unravel_index(at,lengths.shape);a=start+r*(cols+1)+c;b=a+(cols+1 if axis==0 else 1);samples.append(dict(length=float(lengths[r,c]),a=point(a),b=point(b)))
        return dict(minimum=float(lengths.min()),maximum=float(lengths.max()),percentiles=np.percentile(lengths,[0,1,5,50,95,99,100]).tolist(),shortest=samples)
    sheet=f[g['faceStart']:g['faceStart']+g['faceCount']];t=v[sheet];normal=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);area2=np.linalg.norm(normal,axis=1);unit=normal/np.maximum(area2[:,None],1e-30);lengths=np.linalg.norm(t[:,[1,2,0]]-t,axis=2);aspect=lengths.max(1)**2/np.maximum(area2,1e-30);adjacency={};folds=[];winding=[]
    for i,tri in enumerate(sheet):
        for a,b in zip(tri,np.roll(tri,-1)):
            key=tuple(sorted((int(a),int(b))))
            if key in adjacency:
                j,direction=adjacency[key];turn=float(np.rad2deg(np.arccos(np.clip(unit[i]@unit[j],-1,1))))
                if direction==(a,b):winding.append(dict(faces=[int(g['faceStart']+j),int(g['faceStart']+i)],edge=[point(a),point(b)]))
                if turn>60:folds.append(dict(turnDegrees=turn,faces=[int(g['faceStart']+j),int(g['faceStart']+i)],edge=[point(a),point(b)]))
            else:adjacency[key]=(i,(a,b))
    routes=[];route_geometry={}
    for kind,key in [('strap','straps'),('hem','sideHems')]:
        for side,route in enumerate(typed.get(key,[])):
            if route['corners']!=4:raise ValueError('Only typed four-corner ribbon sections are supported')
            q=v[route['start']:route['start']+route['sections']*4].reshape(-1,4,3);center=q.mean(1);edge=np.diff(center,axis=0);length=np.linalg.norm(edge,axis=1)
            if np.any(length<=0):raise ValueError('Degenerate material ribbon centerline')
            tangent=edge/length[:,None];turn=np.rad2deg(np.arccos(np.clip(np.sum(tangent[:-1]*tangent[1:],axis=1),-1,1)))
            width=(q[:,1]+q[:,2]-q[:,0]-q[:,3])/2;normal=(q[:,0]+q[:,1]-q[:,2]-q[:,3])/2
            wl=np.linalg.norm(width,axis=1);nl=np.linalg.norm(normal,axis=1)
            if np.any(wl<=0) or np.any(nl<=0):raise ValueError('Degenerate material ribbon cross section')
            wu=width/wl[:,None];nu=normal/nl[:,None]
            item=dict(kind=kind,side=side,sections=route['sections'],length=float(length.sum()),minimumSectionEdge=float(length.min()),maximumSectionEdge=float(length.max()),maxCenterlineTurnDegrees=float(turn.max()),worstCenterlineSection=int(np.argmax(turn))+1,minimumAdjacentWidthDot=float(np.sum(wu[:-1]*wu[1:],axis=1).min()),minimumAdjacentNormalDot=float(np.sum(nu[:-1]*nu[1:],axis=1).min()))
            worst=item['worstCenterlineSection'];item['worstCenterlinePosition']=center[worst].tolist();item['worstCenterlineNeighborhood']=[dict(section=k,position=center[k].tolist())for k in range(max(0,worst-2),min(len(center),worst+3))]
            item['startCenter']=center[0].tolist();item['endCenter']=center[-1].tolist()
            route_geometry[(kind,side)]=(center,q)
            if kind=='hem':
                boundary=typed['sideBoundary'][side]
                if len(boundary)!=len(center):raise ValueError('Hem and sheet boundary section count differs')
                item['maximumSheetEdgeCenterlineGap']=float(np.linalg.norm(center-v[boundary],axis=1).max())
            routes.append(item)
    junctions=[]
    if typed.get('bottomSeams') and typed.get('sideHems'):
        seams=[v[indices].mean(0)for indices in typed['bottomSeams']]
        for side,route in enumerate(typed.get('straps',[])):
            center,q=route_geometry[('strap',side)];attached=int(np.argmin([np.linalg.norm(center[-1]-seam)for seam in seams]));hem,hq=route_geometry[('hem',attached)]
            incoming_strap=(center[-1]-center[-2])/np.linalg.norm(center[-1]-center[-2]);incoming_hem=(hem[-1]-hem[-2])/np.linalg.norm(hem[-1]-hem[-2])
            distances=np.linalg.norm(q[-1][:,None,:]-hq[-1][None,:,:],axis=2)
            junctions.append(dict(strap=side,sheetSide=attached,strapEndpointToBottomSeamCenterGap=float(np.linalg.norm(center[-1]-seams[attached])),hemEndpointToSheetBoundaryGap=float(np.linalg.norm(hem[-1]-v[typed['sideBoundary'][attached][-1]])),hemToStrapEndpointCenterGap=float(np.linalg.norm(hem[-1]-center[-1])),minimumHemStrapTerminalCornerGap=float(distances.min()),approachingTangentAngleDegrees=float(np.rad2deg(np.arccos(np.clip(incoming_strap@incoming_hem,-1,1)))),straightContinuationTurnDegrees=float(np.rad2deg(np.arccos(np.clip(-incoming_strap@incoming_hem,-1,1)))),meaning='measured junction geometry; seam width/finite thickness and chosen sewn bend require interpretation, not an automatic pass'))
    authored=measure_authored_joints(v,typed.get('authoredJoints',[]),typed.get('measuredCircumference',1.),typed.get('bandThicknessNormalized',.003))
    result=dict(authoredJointAudit=authored,sourceOBJ=str(obj),sourceSHA256=hashlib.sha256(obj.read_bytes()).hexdigest(),layoutSHA256=hashlib.sha256(layout.read_bytes()).hexdigest(),grid=g,longitudinal=edges(0),transverse=edges(1),maximumTriangleAspect=float(aspect.max()),worstTriangles=[dict(face=int(g['faceStart']+i),aspect=float(aspect[i]),area=float(area2[i]/2),points=[point(a)for a in sheet[i]])for i in np.argsort(aspect)[-20:][::-1]],foldCountOver60=len(folds),worstFolds=sorted(folds,key=lambda x:-x['turnDegrees'])[:30],windingConflictCount=len(winding),windingConflicts=winding[:20],routes=routes,junctions=junctions,meaning='rest grid and typed ribbon measurement only; neither visual acceptance nor dynamic success')
    report.write_text(json.dumps(result,indent=2));return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('obj',type=Path);p.add_argument('layout',type=Path);p.add_argument('--report',required=True,type=Path);a=p.parse_args();r=audit(a.obj,a.layout,a.report);print(json.dumps({k:r[k]for k in ['maximumTriangleAspect','foldCountOver60','windingConflictCount']}))


if __name__ == "__main__":
    main()
