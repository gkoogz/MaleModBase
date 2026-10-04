"""Hash-bound, SDK-free garment geometry audit. Runtime success is separate.

The manifest supplies actual exported cases and their source artifacts. Missing
exports stay PENDING. No fitted output may authorize its own seam offsets.
"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from malemod_base.garment_geometry_audit import (
    audit, artifact, digest, FAMILIES, SIZES, ANGLES, STATES, MAX_TURN,
    REQUIRED_ARTIFACTS,
)


def run(manifest_path, output, intersection_exe):
    manifest = json.loads(Path(manifest_path).read_text())
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    directory = output.parent / (output.stem + "-details")
    directory.mkdir(exist_ok=True)
    header = artifact(manifest["headerManifest"])
    source_header = json.loads(Path(header["path"]).read_text())
    # A clean current Git pin and all header hashes are independently checked;
    # a boolean copied into a manifest does not prove source cleanliness.
    source_root = Path(source_header["sourceRoot"]).resolve()
    actual_pin = subprocess.check_output(["git", "-C", str(source_root), "rev-parse", "HEAD"], text=True).strip()
    dirty = subprocess.check_output(["git", "-C", str(source_root), "status", "--porcelain"], text=True).strip()
    source_clean = not dirty and source_header.get("cleanPin") is True and source_header.get("commit") == actual_pin
    recorded_headers = {Path(record["path"]).resolve(): artifact(record)["sha256"] for record in source_header["headers"]}
    current_headers = {path.resolve(): digest(path) for path in (source_root / "include/malemod/garments").rglob("*") if path.is_file()}
    source_clean = source_clean and len(recorded_headers) == len(source_header["headers"]) and recorded_headers == current_headers
    proof = artifact(manifest["independentFixtureProof"])
    rows_by_key = {}
    for row in manifest["cases"]:
        key = (row["family"], row["overall"], row["angle"], row["state"])
        if key in rows_by_key:
            raise ValueError("Duplicate actual case: " + str(key))
        if key not in set(itertools.product(FAMILIES, SIZES, ANGLES, STATES)):
            raise ValueError("Case lies outside the measured UI matrix: " + str(key))
        rows_by_key[key] = row
    fixture = json.loads(Path(proof["path"]).read_text())
    if fixture.get("crossingAndClosureFixturesPassed") is not True or fixture.get("crossingExitCode") != 0 or fixture.get("closureExitCode") != 0:
        raise ValueError("Independent crossing and finite-offset closure fixtures did not pass")
    executable = Path(intersection_exe).resolve()
    fixture_artifacts = {name: artifact(value) for name, value in fixture["artifacts"].items()}
    if fixture_artifacts["intersectionExecutable"] != dict(path=str(executable), sha256=digest(executable)):
        raise ValueError("Intersection executable differs from the independently tested binary")
    for name, path in (("intersectionSource", ROOT / "tools/audit_surface_intersections.cpp"),
                       ("closureAuditSource", ROOT / "malemod_base/garment_geometry_audit.py"),
                       ("closureTestSource", ROOT / "tests/test_garment_geometry_audit.py")):
        if fixture_artifacts[name] != dict(path=str(path), sha256=digest(path)):
            raise ValueError("Audit source differs from the independent fixture proof: " + name)
    rows = []
    for key in itertools.product(FAMILIES, SIZES, ANGLES, STATES):
        family, overall, angle, state = key
        row = dict(family=family, overall=overall, angle=angle, state=state)
        case = rows_by_key.get(key)
        if case is None:
            rows.append(dict(row, status="PENDING", reason="No actual export for this case"))
            continue
        if case.get("exportExitCode") not in (None, 0):
            rows.append(dict(row, status="FAILED", reason="Actual source export returned an error", exportExitCode=case["exportExitCode"], artifacts={k: artifact(v) for k, v in case.get("artifacts", {}).items()}))
            continue
        supplied = case.get("artifacts", {})
        missing = [name for name in REQUIRED_ARTIFACTS if name not in supplied]
        verified = {name: artifact(value) for name, value in supplied.items()}
        if "meshOBJ" not in verified or "layout" not in verified:
            rows.append(dict(row, status="PENDING", reason="Missing actual mesh/layout", missingArtifacts=missing, artifacts=verified))
            continue
        obj = Path(verified["meshOBJ"]["path"])
        layout = Path(verified["layout"]["path"])
        topology = json.loads(layout.read_text())
        stem = f"{family}-overall{overall}-angle{angle}-state{state}"
        geometry_path = directory / (stem + "-geometry.json")
        crossing_path = directory / (stem + "-crossing.json")
        geometry = audit(obj, layout, geometry_path)
        sheet = topology["sheet"]
        subprocess.run([str(executable), str(obj), str(sheet["faceStart"]), str(sheet["faceCount"]), str(crossing_path)], check=True, capture_output=True, text=True)
        crossing = json.loads(crossing_path.read_text())
        turns = [math.radians(route["maxCenterlineTurnDegrees"]) for route in geometry["routes"] if route["kind"] == "strap"]
        hem_turns = [math.radians(route["maxCenterlineTurnDegrees"]) for route in geometry["routes"] if route["kind"] == "hem"]
        if len(turns) != 2 or len(hem_turns) != 2:
            raise ValueError("Actual garment lacks two typed straps/hems")
        closure = geometry["authoredJointAudit"]
        failures = []
        if crossing["sheetCrossings"]:
            failures.append("Sheet has proper noncoplanar crossings")
        if crossing["sheetCoplanarOverlaps"]:
            failures.append("Sheet has positive-area coplanar overlaps")
        if crossing["degenerateFaceCount"]:
            failures.append("Sheet has degenerate faces")
        if max(turns) >= MAX_TURN:
            failures.append("Rear strap section turn exceeds frozen pi/6 limit")
        if max(hem_turns) >= MAX_TURN:
            failures.append("Side hem section turn exceeds frozen pi/6 limit")
        if closure.get("certified") and not closure["passed"]:
            failures.append("Authored seam-offset residual exceeds bandThickness/4")
        if geometry["windingConflictCount"]:
            failures.append("Sheet face orientation is inconsistent")
        pending = []
        if missing:
            pending.append("Missing hash-bound artifacts: " + ",".join(missing))
        if not closure.get("certified"):
            pending.append(closure["reason"])
        if not source_clean:
            pending.append("Base is not an independently verified clean current pin")
        if verified.get("baseHeaderManifest") != header:
            pending.append("Case was not exported against this exact Base header manifest")
        if "samplerBinding" in verified:
            binding = json.loads(Path(verified["samplerBinding"]["path"]).read_text())
            if any(binding.get(name) != value for name, value in row.items()):
                pending.append("Sampler binding does not declare the exact actual UI state")
        if "sourceExportReceipt" in verified:
            exported = json.loads(Path(verified["sourceExportReceipt"]["path"]).read_text())
            if any(exported.get(name) != value for name, value in row.items()):
                pending.append("Source export receipt does not declare the exact actual UI state")
            for name in ("input", "meshOBJ", "layout", "baseHeaderManifest"):
                if name in verified and exported.get("artifacts", {}).get(name) != verified[name]:
                    pending.append("Source export receipt is not bound to artifact " + name)
        row.update(status="REJECTED" if failures else "PENDING" if pending else "PASS", failures=failures, pending=pending,
                   sheetCrossings=crossing["sheetCrossings"], sheetCoplanarOverlaps=crossing["sheetCoplanarOverlaps"], sheetTouchPairs=crossing["sheetTouchPairs"],
                   degenerateFaces=crossing["degenerateFaceCount"], maxStrapTurnRadians=max(turns), maxHemTurnRadians=max(hem_turns),
                   hemJunctionGapNormalized=closure["maximumResidualNormalized"], authoredJointAudit=closure, rawJunctions=geometry["junctions"],
                   measuredTolerances=dict(maxStrapTurnRadians=MAX_TURN, maxHemTurnRadians=MAX_TURN, maxAuthoredSeamResidualNormalized=closure["toleranceNormalized"],
                                           intersectionTolerance=crossing["intersectionTolerance"], twiceAreaDegeneracyThreshold=crossing["twiceAreaDegeneracyThreshold"],
                                           coplanarOverlapAreaThreshold=crossing["coplanarOverlapAreaThreshold"]), artifacts=verified,
                   detailArtifacts=dict(geometry=dict(path=str(geometry_path), sha256=digest(geometry_path)), intersections=dict(path=str(crossing_path), sha256=digest(crossing_path))))
        rows.append(row)
    result = dict(schema=2, scope="Independent actual static garment consistency; runtime cloth, native optics and gameplay are separate",
                  expectedCases=72, matrix=dict(families=FAMILIES, overall=SIZES, angles=ANGLES, states=STATES), headerManifest=header,
                  cleanPinVerified=source_clean, baseCommit=actual_pin, independentFixtureProof=proof,
                  intersectionAuditor=dict(path=str(executable), sha256=digest(executable)),
                  auditTool=dict(path=str(Path(__file__).resolve()), sha256=digest(__file__)),
                  allCasesPassed=all(row["status"] == "PASS" for row in rows), cases=rows)
    output.write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--intersection-exe", required=True, type=Path)
    args = parser.parse_args()
    result = run(args.manifest, args.output, args.intersection_exe)
    print(json.dumps(dict(allCasesPassed=result["allCasesPassed"], statuses={name: sum(row["status"] == name for row in result["cases"]) for name in ("PASS", "REJECTED", "FAILED", "PENDING")})))
