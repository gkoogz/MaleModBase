"""Deterministic topology-safe endpoint quadric simplification, offline only.
Adapted from this project's prior authoring study; no runtime dependency.
"""
import heapq
import numpy as np
MAX_CLUSTER_RADIUS=.70
RAPHE_T_MIN=0.
RAPHE_T_MAX=.93
def vertex_region(flex):return np.zeros(len(flex),dtype=np.int8)
def build_raphe_lock(positions,flex,shaft_mask,group):return np.zeros(len(flex),dtype=bool)
class QEMDecimator:
    """Deterministic endpoint-collapse QEM with protected seams and features."""

    def __init__(
        self,
        positions: np.ndarray,
        faces: np.ndarray,
        flex: np.ndarray,
        shaft_mask: np.ndarray,
        max_cluster_radius: float = MAX_CLUSTER_RADIUS,
    ) -> None:
        self.p = np.asarray(positions, dtype=np.float64)
        self.f = np.asarray(faces, dtype=np.int64).copy()
        self.flex = np.clip(np.asarray(flex), 0.0, 1.0)
        self.region = vertex_region(self.flex)
        self.nv = len(self.p)
        if len(self.region) != self.nv or self.f.ndim != 2 or self.f.shape[1] != 3:
            raise ValueError("Vertex attributes and triangular faces do not match")
        if self.f.min() < 0 or self.f.max() >= self.nv:
            raise ValueError("Triangle index outside vertex array")

        self.active_face = np.ones(len(self.f), dtype=bool)
        self.alive_vertex = np.ones(self.nv, dtype=bool)
        self.parent = np.arange(self.nv, dtype=np.int64)
        self.locked = np.zeros(self.nv, dtype=bool)
        self.cluster_radius = np.zeros(self.nv, dtype=np.float64)
        self.max_cluster_radius = float(max_cluster_radius)
        self.version = np.zeros(self.nv, dtype=np.int32)
        self.heap: list[tuple[float, int, int, int, int, int, int]] = []
        self.serial = 0
        self.vertex_faces = [set() for _ in range(self.nv)]
        self.neighbors = [set() for _ in range(self.nv)]
        self.edge_faces: dict[tuple[int, int], set[int]] = {}
        self.face_group = np.full(len(self.f), -1, dtype=np.int8)
        self.group_counts = {0: 0, 1: 0}
        self.active_faces = len(self.f)
        self._init_topology()
        self._init_locks(shaft_mask)
        self._init_quadrics()
        self._init_groups()

    @staticmethod
    def _edge(a: int, b: int) -> tuple[int, int]:
        return (a, b) if a < b else (b, a)

    def _init_topology(self) -> None:
        for fid, tri in enumerate(self.f):
            a, b, c = map(int, tri)
            for v in (a, b, c):
                self.vertex_faces[v].add(fid)
            for x, y in ((a, b), (b, c), (c, a)):
                edge = self._edge(x, y)
                self.edge_faces.setdefault(edge, set()).add(fid)
                self.neighbors[x].add(y)
                self.neighbors[y].add(x)

    def _init_locks(self, shaft_mask: np.ndarray) -> None:
        boundary = set()
        for (a, b), adj in self.edge_faces.items():
            if len(adj) != 2:
                boundary.add(a)
                boundary.add(b)
        if boundary:
            self.locked[list(boundary)] = True

        raphe = build_raphe_lock(self.p, self.flex, shaft_mask, self.region)
        self.locked |= raphe

        # Freeze the transition loops so simplification cannot migrate the
        # collar/rim boundary into the pelvis or the glans apex.
        for (a, b), _ in self.edge_faces.items():
            if self.region[a] != self.region[b]:
                self.locked[a] = self.locked[b] = True
        for v in range(self.nv):
            if self.region[v] < 0:
                self.locked[v] = True

        # Lock a vertex ring next to protected geometry. This retains the
        # proximal trumpet, the ventral raphe support, and the coronal fold.
        near_protected = set()
        for v in np.flatnonzero(self.locked):
            near_protected.update(self.neighbors[int(v)])
        if near_protected:
            self.locked[list(near_protected)] = True

    def _init_quadrics(self) -> None:
        self.quadric = np.zeros((self.nv, 4, 4), dtype=np.float64)
        for tri in self.f:
            a, b, c = map(int, tri)
            n = np.cross(self.p[b] - self.p[a], self.p[c] - self.p[a])
            area2 = np.linalg.norm(n)
            if area2 <= 1e-14:
                continue
            n /= area2
            plane = np.r_[n, -np.dot(n, self.p[a])]
            q = np.outer(plane, plane) * (0.5 * area2)
            self.quadric[a] += q
            self.quadric[b] += q
            self.quadric[c] += q

    def _face_group(self, tri: np.ndarray) -> int:
        labels = self.region[tri]
        if labels[0] >= 0 and labels[0] == labels[1] == labels[2]:
            return int(labels[0])
        return -1

    def _init_groups(self) -> None:
        for fid, tri in enumerate(self.f):
            group = self._face_group(tri)
            self.face_group[fid] = group
            if group >= 0:
                self.group_counts[group] += 1

    def _cost(self, u: int, v: int) -> tuple[float, int, int]:
        q = self.quadric[u] + self.quadric[v]
        hu = np.r_[self.p[u], 1.0]
        hv = np.r_[self.p[v], 1.0]
        cu = float(hu @ q @ hu)
        cv = float(hv @ q @ hv)
        if cu <= cv:
            return max(0.0, cu), u, v
        return max(0.0, cv), v, u

    def _push_edge(self, u: int, v: int, group: int) -> None:
        if not self.alive_vertex[u] or not self.alive_vertex[v] or u == v:
            return
        if self.region[u] != group or self.region[v] != group:
            return
        if self.locked[u] or self.locked[v]:
            return
        edge = self._edge(u, v)
        if edge not in self.edge_faces:
            return
        dist = float(np.linalg.norm(self.p[u] - self.p[v]))
        costs = []
        for keep, remove in ((u, v), (v, u)):
            if max(self.cluster_radius[keep], self.cluster_radius[remove] + dist) <= self.max_cluster_radius:
                q = self.quadric[u] + self.quadric[v]
                h = np.r_[self.p[keep], 1.0]
                costs.append((max(0.0, float(h @ q @ h)), keep, remove))
        if not costs:
            return
        cost, keep, remove = min(costs)
        if group == 1:
            cost *= 0.35  # Prefer spending the planned reduction in the dense apex.
        self.serial += 1
        heapq.heappush(
            self.heap,
            (cost, self.serial, u, v, int(self.version[u]), int(self.version[v]), keep),
        )

    def _start_group(self, group: int) -> None:
        self.heap.clear()
        for u, v in self.edge_faces:
            self._push_edge(u, v, group)

    def _collapse_valid(self, u: int, v: int, keep: int, group: int) -> bool:
        edge = self._edge(u, v)
        incident_edge = self.edge_faces.get(edge, set())
        if len(incident_edge) != 2:
            return False
        if self._face_group(self.f[next(iter(incident_edge))]) != group:
            return False
        if any(self._face_group(self.f[fid]) != group for fid in incident_edge):
            return False
        opposite = set()
        for fid in incident_edge:
            opposite.update(int(x) for x in self.f[fid] if x != u and x != v)
        common = (self.neighbors[u] & self.neighbors[v]) - {u, v}
        if common != opposite or len(opposite) != 2:
            return False

        remove = v if keep == u else u
        dist = float(np.linalg.norm(self.p[keep] - self.p[remove]))
        if max(self.cluster_radius[keep], self.cluster_radius[remove] + dist) > self.max_cluster_radius:
            return False
        if self.locked[remove] or self.locked[keep]:
            return False
        affected = self.vertex_faces[remove]
        for fid in affected:
            tri = self.f[fid]
            if self._face_group(tri) != group:
                return False
            if np.any(self.locked[tri]):
                return False
            if keep in tri:
                continue
            replacement = tri.copy()
            replacement[replacement == remove] = keep
            old_n = np.cross(self.p[tri[1]] - self.p[tri[0]], self.p[tri[2]] - self.p[tri[0]])
            new_n = np.cross(
                self.p[replacement[1]] - self.p[replacement[0]],
                self.p[replacement[2]] - self.p[replacement[0]],
            )
            old_len = float(np.linalg.norm(old_n))
            new_len = float(np.linalg.norm(new_n))
            if old_len <= 1e-14 or new_len <= 1e-12:
                return False
            if float(np.dot(old_n, new_n)) < 0.20 * old_len * new_len:
                return False
        return True

    def _remove_face(self, fid: int) -> None:
        tri = self.f[fid]
        group = int(self.face_group[fid])
        if group >= 0:
            self.group_counts[group] -= 1
        for v in map(int, tri):
            self.vertex_faces[v].discard(fid)
        for x, y in ((int(tri[0]), int(tri[1])), (int(tri[1]), int(tri[2])), (int(tri[2]), int(tri[0]))):
            edge = self._edge(x, y)
            faces = self.edge_faces[edge]
            faces.discard(fid)
            if not faces:
                del self.edge_faces[edge]
                self.neighbors[x].discard(y)
                self.neighbors[y].discard(x)
        self.active_face[fid] = False
        self.active_faces -= 1

    def _add_face(self, fid: int, tri: np.ndarray) -> None:
        self.f[fid] = tri
        self.active_face[fid] = True
        self.active_faces += 1
        group = self._face_group(tri)
        self.face_group[fid] = group
        if group >= 0:
            self.group_counts[group] += 1
        a, b, c = map(int, tri)
        for v in (a, b, c):
            self.vertex_faces[v].add(fid)
        for x, y in ((a, b), (b, c), (c, a)):
            edge = self._edge(x, y)
            self.edge_faces.setdefault(edge, set()).add(fid)
            self.neighbors[x].add(y)
            self.neighbors[y].add(x)

    def _do_collapse(self, u: int, v: int, keep: int) -> None:
        remove = v if keep == u else u
        delta = float(np.linalg.norm(self.p[keep] - self.p[remove]))
        incident = sorted(self.vertex_faces[remove])
        old_faces = [(fid, self.f[fid].copy()) for fid in incident]
        dirty = {keep, remove}
        for fid, tri in old_faces:
            dirty.update(map(int, tri))
        for fid, _ in old_faces:
            self._remove_face(fid)
        for fid, tri in old_faces:
            tri[tri == remove] = keep
            if len(set(map(int, tri))) < 3:
                continue
            self._add_face(fid, tri)

        self.quadric[keep] += self.quadric[remove]
        self.alive_vertex[remove] = False
        self.parent[remove] = keep
        self.cluster_radius[keep] = max(
            self.cluster_radius[keep], self.cluster_radius[remove] + delta
        )
        for x in list(dirty):
            if self.alive_vertex[x]:
                dirty.update(self.neighbors[x])
        for x in dirty:
            if self.alive_vertex[x]:
                self.version[x] += 1
        for x in dirty:
            if not self.alive_vertex[x]:
                continue
            for y in self.neighbors[x]:
                if self.region[x] == self.region[y] and self.region[x] >= 0:
                    self._push_edge(x, y, int(self.region[x]))

    def simplify_group(self, group: int, target_faces: int) -> None:
        self._start_group(group)
        while self.group_counts[group] > target_faces:
            if not self.heap:
                raise RuntimeError(
                    f"Could not reach group {group} target: "
                    f"{self.group_counts[group]} faces remain, target {target_faces}"
                )
            _, _, u, v, vu, vv, keep = heapq.heappop(self.heap)
            if not self.alive_vertex[u] or not self.alive_vertex[v]:
                continue
            if self.version[u] != vu or self.version[v] != vv:
                self._push_edge(u, v, group)
                continue
            edge = self._edge(u, v)
            if edge not in self.edge_faces:
                continue
            if not self._collapse_valid(u, v, keep, group):
                continue
            old_count = self.group_counts[group]
            self._do_collapse(u, v, keep)
            if old_count - self.group_counts[group] != 2:
                raise RuntimeError("An interior collapse did not remove exactly two triangles")

    def result(self) -> tuple[np.ndarray, np.ndarray, dict]:
        face_ids = np.flatnonzero(self.active_face)
        faces = self.f[face_ids].copy()
        displacement = self.cluster_radius[self.parent != np.arange(self.nv)]
        edges = np.sort(np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        report = {
            "faces": int(len(faces)),
            "boundary_edges": int(np.count_nonzero(counts == 1)),
            "nonmanifold_edges": int(np.count_nonzero(counts > 2)),
            "max_cluster_radius": float(self.cluster_radius.max(initial=0.0)),
            "median_cluster_radius": float(np.median(self.cluster_radius[self.cluster_radius > 0])) if np.any(self.cluster_radius > 0) else 0.0,
        }
        return faces, face_ids, report
