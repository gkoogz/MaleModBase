"""Exact edge/triangle crossing checks for posed shaft and pouch-body faces."""
import json
import numpy as np
from scipy.spatial import cKDTree
from PIL import Image, ImageDraw
from compact_runtime import ROOT, HERE, ARRAY
from audit_compact import points
from render_review import camera, render, font


def crossings(a, b):
    result = np.zeros(len(a), bool)
    for source, target in [(a, b), (b, a)]:
        e = target[:, 1] - target[:, 0]
        g = target[:, 2] - target[:, 0]
        for corner in range(3):
            start = source[:, corner]
            direction = source[:, (corner + 1) % 3] - start
            h = np.cross(direction, g)
            det = np.sum(e * h, axis=1)
            valid = np.abs(det) > 1e-10
            inv = np.zeros_like(det)
            inv[valid] = 1 / det[valid]
            delta = start - target[:, 0]
            u = np.sum(delta * h, axis=1) * inv
            q = np.cross(delta, e)
            v = np.sum(direction * q, axis=1) * inv
            t = np.sum(g * q, axis=1) * inv
            result |= valid & (u > 1e-7) & (v > 1e-7) & (u + v < 1 - 1e-7) & (t > 1e-7) & (t < 1 - 1e-7)
    return result


def main():
    counts = json.loads((HERE / 'compact-report.json').read_text())
    density = np.load(ROOT.parents[1] / 'outputs/triangle-density-investigation-20260929/density-audit.npz')
    regions = density['regions'][np.load(HERE / 'candidate.npz')['nr_lineage']]
    text = (ROOT / 'src/runtime/neck_render_data.h').read_text()
    match = next(m for m in ARRAY.finditer(text) if m[3] == 'nrIndices')
    faces = np.fromstring(match[5].replace('\n', ''), sep=',', dtype=int).reshape(-1, 3)
    results = []
    poses = {}
    for version in ['before', 'after']:
        for state in range(3):
            raw = (ROOT / f'captures/hanging-{version}/hanging-{state}.bin').read_bytes()
            p = points(raw, counts['nrVertices'], counts['ucNodes'] * 28)
            poses[version, state] = p
            a = p[faces[regions == 'shaft']]
            b = p[faces[regions == 'pouch body']]
            ac, bc = a.mean(1), b.mean(1)
            ar = np.linalg.norm(a - ac[:, None], axis=2).max(1)
            br = np.linalg.norm(b - bc[:, None], axis=2).max(1)
            pairs = cKDTree(bc).query_ball_point(ac, ar + br.max())
            ai = np.repeat(np.arange(len(a)), [len(x) for x in pairs])
            bi = np.concatenate(pairs).astype(int)
            hit = crossings(a[ai], b[bi])
            results.append(dict(version=version, state=state, intersectingPairs=int(hit.sum()), intersectingShaftFaces=len(np.unique(ai[hit]))))
    assert any(r['intersectingPairs'] for r in results if r['version'] == 'before'), 'Fixture did not reproduce the clipping'
    assert not any(r['intersectingPairs'] for r in results if r['version'] == 'after'), results
    output = ROOT.parents[1] / 'outputs/hanging-contact-review-20260929'
    output.mkdir(exist_ok=True)
    (output / 'intersection-audit.json').write_text(json.dumps(dict(method='Bounding-sphere candidate search, bidirectional edge/triangle tests; excludes shared root/neck and coplanar contacts', cases=results), indent=2))
    sheet = Image.new('RGB', (1400, 1710), '#f7f9fc')
    draw = ImageDraw.Draw(sheet)
    draw.text((25, 15), 'BEFORE: undersized contact tube', fill='#23384a', font=font(24))
    draw.text((725, 15), 'AFTER: matches ventral envelope', fill='#23384a', font=font(24))
    for row, direction in enumerate([[1, 0, 0], [0, -1, 0]]):
        basis = camera(direction)
        q = np.r_[poses['before', 2], poses['after', 2]] @ basis
        limits = (q[:, :2].min(0), q[:, :2].max(0))
        for column, version in enumerate(['before', 'after']):
            sheet.paste(render(poses[version, 2], faces, basis, limits, W=700, H=800), (column * 700, 85 + row * 810))
    sheet.save(output / 'hanging-comparison.png')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
