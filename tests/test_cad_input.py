"""Unit tests for cad.cad_input -- riconoscimento e discovery dell'input CAD.

Nessun FreeCAD e nessuna conversione STEP reale: si testano solo le decisioni
(tipo di file, scelta del CAD, errori su input ambigui).
"""

import pytest
from cad import cad_input as ci


def _ws(tmp_path):
    ws = tmp_path / "ws"
    (ws / "cad").mkdir(parents=True)
    return ws


def _touch(path, data=b"x"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


# --- sniff_kind: il contenuto conta piu' dell'estensione ---------------------

def test_sniff_freecad_by_zip_magic(tmp_path):
    f = _touch(tmp_path / "anything.bin", b"PK\x03\x04rest")
    assert ci.sniff_kind(str(f)) == ci.FREECAD


def test_sniff_step_by_iso_header(tmp_path):
    f = _touch(tmp_path / "part.stp", b"ISO-10303-21;\nHEADER;\n")
    assert ci.sniff_kind(str(f)) == ci.STEP


def test_sniff_rejects_unknown(tmp_path):
    f = _touch(tmp_path / "part.stl", b"solid foo\n")
    with pytest.raises(ci.CadInputError, match="formato non riconosciuto"):
        ci.sniff_kind(str(f))


def test_sniff_rejects_missing(tmp_path):
    with pytest.raises(ci.CadInputError, match="non esistente"):
        ci.sniff_kind(str(tmp_path / "nope.FCStd"))


# --- discover ----------------------------------------------------------------

def test_discover_none_when_cad_dir_empty(tmp_path, monkeypatch):
    monkeypatch.delenv("MARITIME_CAD", raising=False)
    assert ci.discover(str(_ws(tmp_path))) is None


def test_discover_single_file_is_picked(tmp_path, monkeypatch):
    monkeypatch.delenv("MARITIME_CAD", raising=False)
    ws = _ws(tmp_path)
    f = _touch(ws / "cad" / "motor.step")
    assert ci.discover(str(ws)) == str(f)


def test_discover_prefers_wec_assembly_among_many(tmp_path, monkeypatch):
    monkeypatch.delenv("MARITIME_CAD", raising=False)
    ws = _ws(tmp_path)
    _touch(ws / "cad" / "assembly_test.FCStd")
    canon = _touch(ws / "cad" / "wec_assembly.FCStd")
    assert ci.discover(str(ws)) == str(canon)


def test_discover_ambiguous_is_an_error(tmp_path, monkeypatch):
    monkeypatch.delenv("MARITIME_CAD", raising=False)
    ws = _ws(tmp_path)
    _touch(ws / "cad" / "a.FCStd")
    _touch(ws / "cad" / "b.step")
    with pytest.raises(ci.CadInputError, match="piu' CAD"):
        ci.discover(str(ws))


def test_discover_explicit_argument_wins(tmp_path, monkeypatch):
    monkeypatch.setenv("MARITIME_CAD", str(tmp_path / "from_env.FCStd"))
    ws = _ws(tmp_path)
    _touch(ws / "cad" / "a.FCStd")
    _touch(ws / "cad" / "b.step")
    chosen = _touch(ws / "cad" / "b.step")
    assert ci.discover(str(ws), "cad/b.step") == str(chosen)


def test_discover_env_used_when_no_argument(tmp_path, monkeypatch):
    ws = _ws(tmp_path)
    f = _touch(tmp_path / "ext" / "part.STEP")
    monkeypatch.setenv("MARITIME_CAD", str(f))
    assert ci.discover(str(ws)) == str(f)


def test_discover_missing_explicit_is_an_error(tmp_path, monkeypatch):
    monkeypatch.delenv("MARITIME_CAD", raising=False)
    with pytest.raises(ci.CadInputError, match="non esiste"):
        ci.discover(str(_ws(tmp_path)), "cad/ghost.FCStd")


# --- scelta della radice per STEP senza contratto -----------------------------

def test_choose_root_is_largest_mass_body():
    doc = {"bodies": [
        {"name": "small", "solids": [{"mass_kg": 1.0}]},
        {"name": "big", "solids": [{"mass_kg": 5.0}, {"mass_kg": 4.0}]},
    ]}
    assert ci._choose_root(doc) == "big"


def test_choose_root_empty_document_is_none():
    assert ci._choose_root({"bodies": []}) is None


def test_sync_failure_is_reported_not_raised(tmp_path, monkeypatch):
    """Un'eccezione in config_sync deve diventare False (errore loggato), non un traceback."""
    from cad import config_sync
    from cad import export_manifest as em

    def boom(**_kwargs):
        raise ModuleNotFoundError("No module named 'mooring'")

    monkeypatch.setattr(config_sync, "sync_from_manifest", boom)
    assert em.sync_configs(str(tmp_path), write_mooring_derived=False) is False
