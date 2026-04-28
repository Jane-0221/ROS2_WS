#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parent.parent
MODELS_DIR = WORKSPACE / "src" / "medipick_simple3_description" / "models"


def parse_obj(path: Path):
    vertices = []
    texcoords = []
    faces = []
    mtllib = None
    usemtl = None

    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("mtllib "):
            mtllib = line.split(maxsplit=1)[1]
            continue
        if line.startswith("usemtl "):
            usemtl = line.split(maxsplit=1)[1]
            continue
        if line.startswith("v "):
            _, x, y, z = line.split()
            vertices.append((float(x), float(y), float(z)))
            continue
        if line.startswith("vt "):
            parts = line.split()
            texcoords.append((float(parts[1]), float(parts[2])))
            continue
        if line.startswith("f "):
            entries = []
            for token in line.split()[1:]:
                pieces = token.split("/")
                v_idx = int(pieces[0]) - 1
                vt_idx = int(pieces[1]) - 1 if len(pieces) > 1 and pieces[1] else None
                entries.append((v_idx, vt_idx))
            faces.append(entries)

    return mtllib, usemtl, vertices, texcoords, faces


def subtract(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(v):
    length = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if length == 0.0:
        return (0.0, 0.0, 1.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def triangulate_face(face):
    if len(face) == 3:
        return [face]
    if len(face) == 4:
        return [[face[0], face[1], face[2]], [face[0], face[2], face[3]]]
    triangles = []
    for i in range(1, len(face) - 1):
        triangles.append([face[0], face[i], face[i + 1]])
    return triangles


def convert_obj_to_triangulated_with_normals(path: Path):
    mtllib, usemtl, vertices, texcoords, faces = parse_obj(path)
    if not vertices or not texcoords or not faces:
        raise RuntimeError(f"Unexpected OBJ structure in {path}")

    out_lines = []
    if mtllib:
        out_lines.append(f"mtllib {mtllib}")
    if usemtl:
        out_lines.append(f"usemtl {usemtl}")
    out_lines.append("o Box")

    out_vertices = []
    out_texcoords = []
    out_normals = []
    out_faces = []

    for face in faces:
        base_pos = vertices[face[0][0]]
        edge_a = subtract(vertices[face[1][0]], base_pos)
        edge_b = subtract(vertices[face[2][0]], base_pos)
        normal = normalize(cross(edge_a, edge_b))
        triangles = triangulate_face(face)
        for tri in triangles:
            face_indices = []
            for v_idx, vt_idx in tri:
                out_vertices.append(vertices[v_idx])
                out_texcoords.append(texcoords[vt_idx] if vt_idx is not None else (0.0, 0.0))
                out_normals.append(normal)
                current_index = len(out_vertices)
                face_indices.append((current_index, current_index, current_index))
            out_faces.append(face_indices)

    for x, y, z in out_vertices:
        out_lines.append(f"v {x:.6f} {y:.6f} {z:.6f}")
    for u, v in out_texcoords:
        out_lines.append(f"vt {u:.6f} {v:.6f}")
    for nx, ny, nz in out_normals:
        out_lines.append(f"vn {nx:.6f} {ny:.6f} {nz:.6f}")
    for face in out_faces:
        out_lines.append(
            "f " + " ".join(f"{v_idx}/{vt_idx}/{vn_idx}" for v_idx, vt_idx, vn_idx in face)
        )

    path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")


def main():
    model_paths = sorted(MODELS_DIR.glob("textured_drug_*/meshes/model.obj"))
    if not model_paths:
        raise SystemExit("No textured drug models found.")

    for model_path in model_paths:
        convert_obj_to_triangulated_with_normals(model_path)
        print(f"Updated {model_path.relative_to(WORKSPACE)}")


if __name__ == "__main__":
    main()
