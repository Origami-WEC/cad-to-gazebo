"""Unit tests for cad.material_props -- pure density/material resolution.

Runs without FreeCAD: only plain Python values, strings and lightweight
stand-ins for FreeCAD ``Quantity`` / ``ShapeMaterial`` objects are used.
"""

import pytest
from cad import material_props as mp


class _Quantity:
    """Stand-in for a FreeCAD Quantity: .Value in kg/mm^3, .UserString text."""

    def __init__(self, value, user="kg/mm^3"):
        self.Value = value
        self.UserString = user


class _ShapeMaterial:
    """Stand-in for FreeCAD ShapeMaterial with physical/appearance props."""

    def __init__(self, name="", props=None, appearance=None):
        self.Name = name
        self.Label = name
        self._props = props or {}
        self._appearance = appearance or {}

    def hasPhysicalProperty(self, key):
        return key in self._props

    def getPhysicalValue(self, key):
        return self._props.get(key)

    def hasAppearanceProperty(self, key):
        return key in self._appearance

    def getAppearanceValue(self, key):
        return self._appearance.get(key)


# --- _quantity_to_si ---------------------------------------------------------

def test_quantity_plain_float_is_kg_m3():
    rho, converted, _ = mp._quantity_to_si(7850.0)
    assert rho == pytest.approx(7850.0)
    assert converted is False


def test_quantity_small_float_treated_as_kg_mm3():
    # 7.85e-6 kg/mm^3 == 7850 kg/m^3
    rho, converted, _ = mp._quantity_to_si(7.85e-6)
    assert rho == pytest.approx(7850.0)
    assert converted is True


def test_quantity_freecad_object_value_is_internal_kg_mm3():
    rho, converted, _ = mp._quantity_to_si(_Quantity(7.85e-6))
    assert rho == pytest.approx(7850.0)
    assert converted is True


def test_quantity_string_with_unit():
    rho, converted, _ = mp._quantity_to_si("2.7 g/cm^3")
    assert rho == pytest.approx(2700.0)
    assert converted is True


def test_quantity_string_kg_mm3():
    rho, converted, _ = mp._quantity_to_si("7.85e-6 kg/mm^3")
    assert rho == pytest.approx(7850.0)
    assert converted is True


def test_quantity_unparseable_string_raises():
    with pytest.raises(ValueError):
        mp._quantity_to_si("heavy")


# --- keyword_density ---------------------------------------------------------

def test_keyword_density_steel():
    rho, kw = mp.keyword_density("Steel")
    assert rho == pytest.approx(7850.0)
    assert kw == "steel"


def test_keyword_density_italian_aluminium():
    rho, kw = mp.keyword_density("Alluminio 6061")
    assert rho == pytest.approx(2700.0)


def test_keyword_density_prefers_longer_keyword():
    # "alluminio" must win over "alu"
    _, kw = mp.keyword_density("Alluminio")
    assert kw == "alluminio"


def test_keyword_density_no_match():
    rho, kw = mp.keyword_density("mystery")
    assert rho is None
    assert kw is None


# --- normalize_density -------------------------------------------------------

def test_normalize_density_none_raises_density_error():
    with pytest.raises(mp.DensityError):
        mp.normalize_density(None, label="hull")


def test_normalize_density_positive_ok():
    res = mp.normalize_density(1000.0, label="hull")
    assert res.ok
    assert res.density_kg_m3 == pytest.approx(1000.0)
    assert res.warnings == []


def test_normalize_density_out_of_range_warns():
    # 5.0 kg/m^3 is below the plausible range -> warning, no unit conversion
    res = mp.normalize_density(5.0, label="ghost")
    assert res.converted is False
    assert any("fuori range" in w for w in res.warnings)

    # a value below KG_MM3_THRESHOLD is read as kg/mm^3 and converted
    res2 = mp.normalize_density(5e-3, label="tiny")
    assert res2.converted is True
    assert res2.density_kg_m3 == pytest.approx(5e6)

    res3 = mp.normalize_density(90000.0, label="lead")
    assert any("fuori range" in w for w in res3.warnings)


def test_normalize_density_non_positive_raises():
    with pytest.raises(mp.DensityError):
        mp.normalize_density(-3.0, label="bad")


# --- resolve_density ---------------------------------------------------------

def test_resolve_density_from_custom_property():
    class Obj:
        Label = "hull"
        Density = 2700.0

    res = mp.resolve_density(obj=Obj())
    assert res.density_kg_m3 == pytest.approx(2700.0)
    assert res.source == "property:Density"


def test_resolve_density_from_card():
    sm = _ShapeMaterial(name="Steel", props={"Density": _Quantity(7.85e-6)})

    class Obj:
        Label = "plate"
        ShapeMaterial = sm

    res = mp.resolve_density(obj=Obj())
    assert res.density_kg_m3 == pytest.approx(7850.0)
    assert res.card == "Steel"
    assert res.source.startswith("card:Steel:")


def test_resolve_density_placeholder_card_is_ignored():
    # FreeCAD "Default" card has 1e-09 kg/mm^3 -> 1 kg/m^3, treated as absent.
    sm = _ShapeMaterial(name="Default", props={"Density": _Quantity(1e-9)})

    class Obj:
        Label = "plate"
        ShapeMaterial = sm

    inherited = mp.DensityResolution(7800.0, "parent")
    res = mp.resolve_density(obj=Obj(), inherited=inherited)
    # the placeholder card must NOT win: the inherited density is used instead
    assert res.density_kg_m3 == pytest.approx(7800.0)
    assert res.source.startswith("inherited")


def test_resolve_density_inherited():
    inherited = mp.DensityResolution(7850.0, "card:Steel:Density")
    res = mp.resolve_density(inherited=inherited)
    assert res.density_kg_m3 == pytest.approx(7850.0)
    assert "ereditata" in " ".join(res.warnings)


def test_resolve_density_keyword_on_label():
    class Obj:
        Label = "Ballast Block"
        ShapeMaterial = None

    res = mp.resolve_density(obj=Obj())
    assert res.density_kg_m3 == pytest.approx(11340.0)
    assert res.source.startswith("label_keyword:")


def test_resolve_density_nothing_raises():
    class Obj:
        Label = "unknown part"
        ShapeMaterial = None

    with pytest.raises(mp.DensityError):
        mp.resolve_density(obj=Obj())


# --- material_colors ---------------------------------------------------------

def test_material_colors_fallback():
    info = mp.material_colors(None)
    assert info["diffuse"] == [0.6, 0.6, 0.6, 1.0]
    assert info["ambient"] == [0.6, 0.6, 0.6, 1.0]


def test_material_colors_from_shape_material():
    sm = _ShapeMaterial(
        name="Steel",
        appearance={"DiffuseColor": "0.1 0.2 0.3 1.0",
                    "AmbientColor": "0.05 0.05 0.05 1.0"},
    )
    info = mp.material_colors(sm)
    assert info["card"] == "Steel"
    assert info["diffuse"] == pytest.approx([0.1, 0.2, 0.3, 1.0])
    assert info["ambient"] == pytest.approx([0.05, 0.05, 0.05, 1.0])


def test_material_colors_fallback_override():
    info = mp.material_colors(None, fallback={"diffuse": [1.0, 0.0, 0.0, 1.0]})
    assert info["diffuse"] == [1.0, 0.0, 0.0, 1.0]


def test_unitless_quantity_above_physical_max_is_kg_m3_not_kg_mm3():
    """Card scritta senza unita' (35 = schiuma poliuretanica 35 kg/m^3).

    Letta come kg/mm^3 darebbe 3.5e10 kg/m^3 e masse da 8e9 kg. Un valore
    impossibile in kg/mm^3 deve essere letto come kg/m^3.
    """
    from material_props import normalize_density

    class _Q:
        def __init__(self, value):
            self.Value = value
            self.UserString = ""

    res = normalize_density(_Q(35.0), label="polyuretane")
    assert res.density_kg_m3 == 35.0
    assert res.converted is False
    assert any("fuori range" in w for w in res.warnings)


def test_unitless_quantity_steel_still_converted_from_kg_mm3():
    from material_props import normalize_density

    class _Q:
        def __init__(self, value):
            self.Value = value
            self.UserString = ""

    res = normalize_density(_Q(7.8e-06), label="steel")
    assert abs(res.density_kg_m3 - 7800.0) < 1e-6
    assert res.converted is True
