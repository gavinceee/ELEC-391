# ----------------------------
# Physical key → motor degree mapping
# Measured: E3 = -1457 deg, E5 = -208 deg  (24 semitone steps, linear)
# ----------------------------
_KEY_DEGREES = {
    "E3":  -1457.00,
    "F3":  -1404.96,
    "F#3": -1352.92,
    "G3":  -1300.88,
    "G#3": -1248.83,
    "A3":  -1196.79,
    "A#3": -1144.75,
    "B3":  -1092.71,
    "C4":  -1040.67,
    "C#4":  -988.62,
    "D4":   -936.58,
    "D#4":  -884.54,
    "E4":   -832.50,
    "F4":   -780.46,
    "F#4":  -728.42,
    "G4":   -676.38,
    "G#4":  -624.33,
    "A4":   -572.29,
    "A#4":  -520.25,
    "B4":   -468.21,
    "C5":   -416.17,
    "C#5":  -364.12,
    "D5":   -312.08,
    "D#5":  -260.04,
    "E5":   -208.00,
}

# Ordered list of valid note names (E3 → E5)
_KEY_ORDER = list(_KEY_DEGREES.keys())

# Flat → sharp normalisation
_FLAT_MAP = {
    "DB": "C#", "EB": "D#", "GB": "F#", "AB": "G#", "BB": "A#",
    "DB3": "C#3", "EB3": "D#3", "GB3": "F#3", "AB3": "G#3", "BB3": "A#3",
    "DB4": "C#4", "EB4": "D#4", "GB4": "F#4", "AB4": "G#4", "BB4": "A#4",
    "DB5": "C#5", "EB5": "D#5", "GB5": "F#5", "AB5": "G#5", "BB5": "A#5",
}


def _normalise_note(note_name: str) -> str:
    """Upper-case and convert flats to sharps."""
    n = note_name.strip().upper()
    return _FLAT_MAP.get(n, n)


def note_to_angle(note_name: str) -> float:
    """Return motor degrees for a note in E3–E5. Raises ValueError if out of range."""
    n = _normalise_note(note_name)
    if n not in _KEY_DEGREES:
        raise ValueError(
            f"Note {note_name!r} is outside the E3–E5 range. "
            f"Valid keys: {', '.join(_KEY_ORDER)}"
        )
    return _KEY_DEGREES[n]


def angle_to_note(degrees: float) -> str:
    """Return the closest note name for a given motor position."""
    closest = min(_KEY_DEGREES.items(), key=lambda kv: abs(kv[1] - degrees))
    return closest[0]