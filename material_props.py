#!/usr/bin/env python3
"""
material_props.py  --  Risoluzione densita'/materiale dei corpi CAD (Layer 2)

Logica pura, testabile senza FreeCAD. L'exporter FreeCAD raccoglie i dati
grezzi (proprieta' Density, ShapeMaterial, label, card) e li passa qui.

Catena di risoluzione densita' (primo match vince):
  1. proprieta' custom `Density` (float, kg/m^3) sull'oggetto shape;
  2. valore fisico della card materiale (`ShapeMaterial` con modello Density);
  3. densita' ereditata dal parent (Body -> App::Part -> App::Part.Material);
  4. keyword nel nome della card (warning forte);
  5. keyword nel Label (warning forte);
  6. altrimenti DensityError: il corpo SENZA materiale/densita' non e'
     esportabile (nessun default silenzioso).

Unita'
------
FreeCAD memorizza le densita' dei materiali in unita' interne kg/mm^3.
`Quantity.Value` e' SEMPRE in kg/mm^3: la conversione SI e' `Value * 1e9`.
Un float nudo si interpreta kg/m^3, con euristica di plausibilita' che
riconverte valori tipici di kg/mm^3 o g/cm^3 e segnala la correzione.
"""

from dataclasses import dataclass, field
import re

MM3_TO_M3 = 1.0e-9   # kg/mm^3 -> kg/m^3
GCC_TO_KGM3 = 1.0e3  # g/cm^3  -> kg/m^3

# Range tipico dei solidi da ingegneria (kg/m^3): fuori range -> warning.
DENSITY_WARN_MIN = 50.0
DENSITY_WARN_MAX = 20000.0

# Sotto questa soglia un float "densita'" e' quasi certamente kg/mm^3.
KG_MM3_THRESHOLD = 1.0e-2

DENSITY_KEYWORDS = {
    "steel": 7850.0, "acciaio": 7850.0,
    "aluminium": 2700.0, "aluminum": 2700.0, "alluminio": 2700.0,
    "alu": 2700.0,
    "lead": 11340.0, "piombo": 11340.0, "ballast": 11340.0,
    "zavorra": 11340.0,
    "polyurethane": 950.0, "poliuretano": 950.0,
    "hdpe": 950.0, "polietilene": 950.0,
    "plastic": 1000.0, "plastica": 1000.0,
    "wood": 700.0, "legno": 700.0,
    "concrete": 2400.0, "cemento": 2400.0, "calcestruzzo": 2400.0,
}


class DensityError(Exception):
    """Corpo senza materiale o densita' risolvibile."""


@dataclass
class DensityResolution:
    density_kg_m3: float
    source: str
    card: str = ""
    warnings: list = field(default_factory=list)
    converted: bool = False
    note: str = ""

    @property
    def ok(self):
        return self.density_kg_m3 > 0.0


def _norm_key(name):
    """Tokenizzazione semplice per match keyword a parola intera."""
    return re.sub(r"[^a-z0-9]+", " ", (name or "").lower()).split()


def keyword_density(text, keywords=None):
    """Match keyword a parola intera su nome card/label. None se nessun match."""
    keywords = keywords or DENSITY_KEYWORDS
    tokens = set(_norm_key(text))
    # preferisci keyword piu' lunghe ("alluminio" su "alu")
    for key in sorted(keywords, key=len, reverse=True):
        if key in tokens or key in (text or "").lower().replace(" ", ""):
            # match a parola intera: la keyword deve comparire come token
            # oppure come sottostringa del nome normalizzato unito
            joined = " ".join(tokens)
            if re.search(r"\b{}\b".format(re.escape(key)), joined):
                return float(keywords[key]), key
    return None, None


def _quantity_to_si(raw, unit_hint=""):
    """
    Converte un valore grezzo in kg/m^3.
    Accetta: Quantity FreeCAD (.Value in kg/mm^3, .UserString), float, stringa.
    Ritorna (rho_si, converted, note).
    """
    note = ""
    unit = (unit_hint or "").replace(" ", "").lower()

    # Quantity-like FreeCAD: ha .Value (unita' interne) e spesso .UserString
    if hasattr(raw, "Value") and not isinstance(raw, (int, float)):
        raw_f = float(raw.Value)
        user = str(getattr(raw, "UserString", "") or "")
        user_n = user.replace(" ", "").lower()
        if "kg/mm" in user_n or "kg/mm" in unit:
            # raw_f e' kg/mm^3 -> kg/m^3 = raw_f / 1e-9
            rho = raw_f / MM3_TO_M3
            note = "Quantity.Value {} {} -> {} kg/m^3".format(
                raw_f, user or "kg/mm^3", rho)
            return rho, True, note
        # Value e' SEMPRE kg/mm^3 nelle unita' interne FreeCAD
        rho = raw_f / MM3_TO_M3
        note = "Quantity.Value {} {} (interno kg/mm^3) -> {} kg/m^3".format(
            raw_f, user or "?", rho)
        return rho, True, note

    if isinstance(raw, str):
        s = raw.strip().lower().replace(",", ".")
        m = re.match(
            r"^([+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?)\s*"
            r"(kg/mm\^?3|kg/m\^?3|g/cm\^?3|g/cc|kg/dm\^?3)?$", s)
        if not m:
            raise ValueError("densita' non interpretabile: {!r}".format(raw))
        raw_f = float(m.group(1))
        unit = m.group(2) or unit or "kg/m^3"
        unit = unit.replace(" ", "").lower()
    else:
        raw_f = float(raw)

    if "kg/mm" in unit:
        rho = raw_f / MM3_TO_M3
        return rho, True, "kg/mm^3 -> kg/m^3 (x1e9)"
    if unit in ("g/cm^3", "g/cc", "g/cm3", "kg/dm^3", "kg/dm3"):
        rho = raw_f * GCC_TO_KGM3
        return rho, True, "{} -> kg/m^3 (x1000)".format(unit)
    if unit in ("kg/m^3", "kg/m3"):
        return raw_f, False, "kg/m^3"

    # nessuna unita' esplicita: euristica
    if 0.0 < raw_f < KG_MM3_THRESHOLD:
        rho = raw_f / MM3_TO_M3
        return rho, True, \
            "valore {}<{} interpretato kg/mm^3 -> {} kg/m^3".format(
                raw_f, KG_MM3_THRESHOLD, rho)
    return raw_f, False, note


def normalize_density(raw, unit_hint="", label="",
                      warn_min=DENSITY_WARN_MIN, warn_max=DENSITY_WARN_MAX):
    """
    Normalizza una densita' grezza a kg/m^3 con warning di plausibilita'.
    Ritorna DensityResolution. Lancia ValueError su input marci.
    """
    if raw is None:
        raise DensityError(
            "nessuna densita' per '{}': materiale/densita' assente".format(label))
    rho, converted, note = _quantity_to_si(raw, unit_hint)
    warnings = []
    if converted:
        warnings.append(
            "densita' di '{}': conversione unita' applicata ({})".format(
                label or "?", note))
    if rho <= 0.0:
        raise DensityError(
            "densita' non positiva ({}) per '{}'".format(rho, label or "?"))
    if rho < warn_min or rho > warn_max:
        warnings.append(
            "densita' di '{}' = {:.6g} kg/m^3 fuori range tipico "
            "[{:.0f}, {:.0f}]: verificare unita' o materiale".format(
                label or "?", rho, warn_min, warn_max))
    return DensityResolution(
        density_kg_m3=float(rho), source="", card="", warnings=warnings,
        converted=converted, note=note)


def _first_density_property(obj, prop_names=("Density",)):
    """Legge una proprieta' float/Quantity 'Density' se presente e valida."""
    for name in prop_names:
        prop = getattr(obj, name, None)
        if prop is None:
            continue
        try:
            return prop, "property:{}".format(name)
        except Exception:
            continue
    return None, ""


def _card_density(sm, placeholder_min=10.0):
    """
    Estrae la densita' da ShapeMaterial/Materials.Material.
    Solo se la card ha davvero il modello fisico Density.
    I placeholder FreeCAD (es. Default con 1e-09 kg/mm^3 = 1 kg/m^3)
    sono trattati come "densita' assente".
    Ritorna (raw_value, card_name, key) oppure (None, card_name, "").
    """
    if sm is None:
        return None, "", ""
    card_name = getattr(sm, "Name", "") or getattr(sm, "Label", "") or ""

    has_prop = getattr(sm, "hasPhysicalProperty", None)
    get_val = getattr(sm, "getPhysicalValue", None)
    if get_val is None:
        return None, card_name, ""

    keys = ("Density", "Density (kg/m^3)", "MassDensity", "Density (kg/mm^3)")
    for key in keys:
        try:
            if callable(has_prop) and not has_prop(key):
                continue
        except Exception:
            pass
        try:
            val = get_val(key)
        except Exception:
            continue
        if val is None:
            continue
        try:
            raw_f = float(getattr(val, "Value", val))
        except (TypeError, ValueError):
            continue
        if raw_f <= 0.0:
            continue
        # placeholder tipico delle card senza modello fisico:
        # 1e-09 kg/mm^3 (= 1 kg/m^3) o valori comunque non strutturali
        try:
            probe = normalize_density(val, label=card_name or "card")
            if probe.density_kg_m3 < placeholder_min:
                continue
        except (DensityError, ValueError):
            continue
        return val, card_name, key
    return None, card_name, ""


def resolve_density(obj=None, inherited=None, label="", card_hint="",
                    warn_min=DENSITY_WARN_MIN, warn_max=DENSITY_WARN_MAX):
    """
    Risolve la densita' di un corpo/solido.

    obj       : oggetto FreeCAD-like (opzionale) con possibili
                proprieta' Density / ShapeMaterial / Material / Label
    inherited : DensityResolution gia' risolta sul parent (opzionale)
    card_hint : nome card materiale se gia' noto
    label     : Label dell'oggetto per warning/errore

    Ritorna DensityResolution. Lancia DensityError se nulla di valido.
    """
    warnings = []
    label = label or (getattr(obj, "Label", "") if obj is not None else "") or "?"

    # 1. proprieta' custom Density
    if obj is not None:
        prop, src = _first_density_property(obj)
        if prop is not None:
            res = normalize_density(prop, label=label,
                                    warn_min=warn_min, warn_max=warn_max)
            res.source = src
            warnings.extend(res.warnings)
            res.warnings = warnings
            return res

    # 2. card materiale dell'oggetto
    card_name = card_hint or ""
    sm = getattr(obj, "ShapeMaterial", None) if obj is not None else None
    raw, card_name2, key = _card_density(sm)
    if raw is not None:
        card_name = card_name2 or card_name
        res = normalize_density(raw, label=label,
                                warn_min=warn_min, warn_max=warn_max)
        res.source = "card:{}:{}".format(card_name, key)
        res.card = card_name
        warnings.extend(res.warnings)
        res.warnings = warnings
        return res
    if card_name2:
        card_name = card_name2

    # 3. densita' ereditata dal parent
    if inherited is not None:
        if isinstance(inherited, DensityResolution):
            res = DensityResolution(
                density_kg_m3=inherited.density_kg_m3,
                source="inherited:{}".format(inherited.source),
                card=inherited.card, warnings=[],
                converted=inherited.converted, note=inherited.note)
        else:
            res = normalize_density(inherited, label=label,
                                    warn_min=warn_min, warn_max=warn_max)
            res.source = "inherited"
        warnings.append(
            "densita' di '{}' ereditata dal parent ({:.6g} kg/m^3, "
            "sorgente {})".format(label, res.density_kg_m3, res.source))
        warnings.extend(res.warnings)
        res.warnings = warnings
        return res

    # 4. keyword sul nome card
    if card_name:
        rho, kw = keyword_density(card_name)
        if rho is not None:
            res = normalize_density(rho, label=label,
                                    warn_min=warn_min, warn_max=warn_max)
            res.source = "card_keyword:{}".format(kw)
            res.card = card_name
            warnings.append(
                "[ATTENZIONE] densita' di '{}' STIMATA da keyword card "
                "'{}' -> {:.6g} kg/m^3 ({}): impostare la proprieta' "
                "Density o una card con modello fisico".format(
                    label, card_name, res.density_kg_m3, kw))
            warnings.extend(res.warnings)
            res.warnings = warnings
            return res

    # 5. keyword sul Label
    rho, kw = keyword_density(label)
    if rho is not None:
        res = normalize_density(rho, label=label,
                                warn_min=warn_min, warn_max=warn_max)
        res.source = "label_keyword:{}".format(kw)
        res.card = card_name
        warnings.append(
            "[ATTENZIONE] densita' di '{}' STIMATA da keyword label "
            "-> {:.6g} kg/m^3 ({}): impostare la proprieta' Density o "
            "una card con modello fisico".format(
                label, res.density_kg_m3, kw))
        warnings.extend(res.warnings)
        res.warnings = warnings
        return res

    raise DensityError(
        "corpo '{}' senza materiale o densita' associati: aggiungere la "
        "proprieta' Density (float, kg/m^3) oppure una card materiale con "
        "modello fisico Density (o ereditarla dal parent App::Part)".format(
            label))


def material_colors(sm, fallback=None):
    """Colori appearance da ShapeMaterial. Ritorna dict diffuse/ambient."""
    info = {"card": "", "diffuse": [0.6, 0.6, 0.6, 1.0],
            "ambient": [0.6, 0.6, 0.6, 1.0]}
    if fallback:
        info.update(fallback)
    if sm is None:
        return info
    info["card"] = getattr(sm, "Name", "") or info.get("card", "")
    for src_key, dst_key in (("DiffuseColor", "diffuse"),
                             ("AmbientColor", "ambient")):
        try:
            if sm.hasAppearanceProperty(src_key):
                col = sm.getAppearanceValue(src_key)
                if isinstance(col, str):
                    nums = re.findall(r"[+-]?\d*\.?\d+(?:e[+-]?\d+)?", col)
                    col = [float(n) for n in nums[:4]]
                info[dst_key] = [float(col[0]), float(col[1]),
                                 float(col[2]), float(col[3])]
        except Exception:
            pass
    return info
