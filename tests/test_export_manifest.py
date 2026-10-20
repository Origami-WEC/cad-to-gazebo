"""Unit tests for cad.export_manifest -- pure-Python manifest helpers.

These tests never touch FreeCAD: they cover the SHA-256 hashing, the
manifest-is-current check and the CAD / freecadcmd discovery logic.
"""

import json
import os

from cad import export_manifest as em

# --- _sha256 -----------------------------------------------------------------

def test_sha256_is_prefixed_and_stable(tmp_path):
    f = tmp_path / "part.FCStd"
    f.write_bytes(b"hello cad")
    h1 = em._sha256(str(f))
    h2 = em._sha256(str(f))
    assert h1 == h2
    assert h1.startswith("sha256:")
    assert len(h1) == len("sha256:") + 64


def test_sha256_changes_with_content(tmp_path):
    f = tmp_path / "part.FCStd"
    f.write_bytes(b"a")
    h1 = em._sha256(str(f))
    f.write_bytes(b"b")
    h2 = em._sha256(str(f))
    assert h1 != h2


# --- manifest_is_current -----------------------------------------------------

def _write_manifest(path, source_hash):
    path.write_text(json.dumps({"meta": {"source_hash": source_hash}}))


def test_manifest_is_current_missing_file(tmp_path):
    assert em.manifest_is_current(str(tmp_path / "nope.json"), "sha256:x") is False


def test_manifest_is_current_matching_hash(tmp_path):
    m = tmp_path / "cad_manifest.json"
    _write_manifest(m, "sha256:abc")
    assert em.manifest_is_current(str(m), "sha256:abc") is True


def test_manifest_is_current_stale_hash(tmp_path):
    m = tmp_path / "cad_manifest.json"
    _write_manifest(m, "sha256:old")
    assert em.manifest_is_current(str(m), "sha256:new") is False


def test_manifest_is_current_malformed_json(tmp_path):
    m = tmp_path / "cad_manifest.json"
    m.write_text("{not json")
    assert em.manifest_is_current(str(m), "sha256:abc") is False


def test_manifest_is_current_missing_meta(tmp_path):
    m = tmp_path / "cad_manifest.json"
    m.write_text(json.dumps({"bodies": []}))
    assert em.manifest_is_current(str(m), "sha256:abc") is False


# --- find_cad_file -----------------------------------------------------------

def test_find_cad_file_explicit(tmp_path):
    ws = tmp_path / "ws"
    ws.mkdir()
    cad = tmp_path / "explicit.FCStd"
    cad.write_bytes(b"x")
    assert em.find_cad_file(str(ws), explicit=str(cad)) == os.path.abspath(str(cad))


def test_find_cad_file_from_env(tmp_path, monkeypatch):
    ws = tmp_path / "ws"
    ws.mkdir()
    cad = tmp_path / "env.FCStd"
    cad.write_bytes(b"x")
    monkeypatch.setenv("MARITIME_CAD", str(cad))
    assert em.find_cad_file(str(ws)) == os.path.abspath(str(cad))


def test_find_cad_file_in_workspace(tmp_path, monkeypatch):
    monkeypatch.delenv("MARITIME_CAD", raising=False)
    ws = tmp_path / "ws"
    (ws / "cad").mkdir(parents=True)
    cand = ws / "cad" / "wec_assembly.FCStd"
    cand.write_bytes(b"x")
    assert em.find_cad_file(str(ws)) == os.path.abspath(str(cand))


def test_find_cad_file_absent(tmp_path, monkeypatch):
    monkeypatch.delenv("MARITIME_CAD", raising=False)
    ws = tmp_path / "ws"
    ws.mkdir()
    assert em.find_cad_file(str(ws)) is None


# --- find_freecadcmd ---------------------------------------------------------

def test_find_freecadcmd_from_env(tmp_path, monkeypatch):
    exe = tmp_path / "freecadcmd"
    exe.write_text("#!/bin/sh\n")
    exe.chmod(0o755)
    monkeypatch.setenv("FREECADCMD", str(exe))
    assert em.find_freecadcmd() == str(exe)


def test_find_freecadcmd_none_when_absent(monkeypatch):
    monkeypatch.delenv("FREECADCMD", raising=False)
    # nothing named freecadcmd / FreeCADCmd on PATH in CI
    monkeypatch.setattr(em.shutil, "which", lambda _name: None)
    assert em.find_freecadcmd() is None
