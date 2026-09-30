"""Check compact collar winding and packed lighting against baseline poses."""
import json
from pathlib import Path
import numpy as np
from compact_runtime import ARRAY, ROOT, HERE


def array(header, name):
    text = (ROOT / 'src/runtime' / header).read_text()
    match = next(m for m in ARRAY.finditer(text) if m[3] == name)
    return np.fromstring(match[5].replace('\n', ''), sep=',', dtype=np.int64)


def main():
    counts = json.loads((HERE / 'compact-report.json').read_text())
    keep = np.load(HERE / 'remaps.npz')['nrKeep']
    indices = array('neck_render_data.h', 'nrIndices').reshape(-1, 3)
    mapping = array('unified_collar_data.h', 'ucMap')
    joined = array('unified_collar_data.h', 'ucFaces').reshape(-1, 3)
    # Joined collar normals use forward cross products; rendering uses reverse.
    assert np.array_equal(joined[-len(indices):], mapping[indices[:, [0, 2, 1]]])
    results = []
    paths = sorted((ROOT / 'captures/surface-audit-lighting-fixed').glob('*.bin'))
    assert len(paths) == 88
    for path in paths:
        raw = path.read_bytes()
        oldraw = (ROOT / 'captures/surface-audit-revised' / path.name).read_bytes()
        brokenraw = (ROOT / 'captures/surface-audit-compact' / path.name).read_bytes()
        offset = counts['ucNodes'] * 28
        a = np.frombuffer(oldraw, np.uint8, 30717 * 32, 50059 * 28).reshape(-1, 32)[keep]
        b = np.frombuffer(raw, np.uint8, counts['nrVertices'] * 32, offset).reshape(-1, 32)
        broken = np.frombuffer(brokenraw, np.uint8, counts['nrVertices'] * 32, offset).reshape(-1, 32)
        # A shading correction must preserve shape, material coordinates and motion.
        delta = b[:, :12].copy().view(np.float32) - broken[:, :12].copy().view(np.float32)
        assert np.max(np.abs(delta)) < 1e-5, path.name
        assert np.array_equal(b[:, 20:], a[:, 20:]), path.name
        assert raw[-168:] == oldraw[-168:], path.name
        n = a[:, 16:19].astype(float) / 255 * 2 - 1
        q = b[:, 16:19].astype(float) / 255 * 2 - 1
        dot = np.sum(n * q, axis=1)
        collarMask = np.frombuffer(raw, np.float32, counts['ucNodes'] * 7).reshape(-1, 7)[:, 6][mapping[:len(keep)]] > 1e-4
        positions = np.frombuffer(raw, np.float32, counts['ucNodes'] * 7).reshape(-1, 7)[:, :3].astype(float)
        triangles = positions[joined]
        faceNormals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normals = np.zeros_like(positions)
        for corner in range(3):
            np.add.at(normals, joined[:, corner], faceNormals)
        expected = normals[mapping[:len(keep)]]
        lengths = np.linalg.norm(expected, axis=1)
        # Tiny folded neighborhoods can differ from the denser baseline. Test
        # actual packed lighting against the outward normals of this geometry.
        valid = collarMask & (lengths > 1e-5)
        agreement = np.sum(expected[valid] * q[valid], axis=1) / (lengths[valid] * np.linalg.norm(q[valid], axis=1))
        assert agreement.min() > .99, (path.name, agreement.min())
        brokenN = broken[:, 16:19].astype(float) / 255 * 2 - 1
        results.append(dict(pose=path.name, oppositeNormals=int(np.sum(dot < 0)),
                            oppositeCollarNormals=int(np.sum((dot < 0) & collarMask)),
                            minimumGeometricNormalAgreement=float(agreement.min()),
                            previousOppositeNormals=int(np.sum(np.sum(n * brokenN, axis=1) < 0)),
                            minimumNormalDot=float(dot.min()), maximumPositionChange=float(np.max(np.abs(delta)))))
    report = dict(cases=len(results), windingMatchesRenderConvention=True,
                  maximumOppositeNormals=max(r['oppositeNormals'] for r in results),
                  maximumOppositeCollarNormals=max(r['oppositeCollarNormals'] for r in results),
                  maximumPositionChange=max(r['maximumPositionChange'] for r in results), poses=results)
    (HERE / 'lighting-audit.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'poses'}, indent=2))


if __name__ == '__main__':
    main()
