# ----------------------------
# Physical key → motor degree mapping
# Measured: E3 = -1457 deg, E5 = -208 deg  (24 semitone steps, linear)
# ----------------------------
_KEY_DEGREES = {
    # anchor
    "G4": 427.0,

    # ---------- 向下 ----------
    "F#4": 427.0 + 58,
    "F4": 427.0 + 2 * 58,
    "E4": 427.0 + 3 * 58,
    "D#4": 427.0 + 4 * 58,
    "D4": 427.0 + 5 * 58,
    "C#4": 427.0 + 6 * 58,
    "C4": 810,

    "B3": 427.0 + 8 * 58,
    "A#3": 427.0 + 9 * 58,
    "A3": 427.0 + 10 * 58,
    "G#3": 427.0 + 11 * 58,
    "G3": 427.0 + 12 * 58,
    "F#3": 427.0 + 13 * 58,
    "F3": 427.0 + 14 * 58,
    "E3": 427.0 + 15 * 58,

    # ---------- 向上 ----------
    "G#4": 427.0 - 1 * 58,
    "A4": 427.0 - 2 * 58,
    "A#4": 427.0 - 3 * 58,
    "B4": 427.0 - 4 * 58,
    "C5": 427.0 - 5 * 58,
    "C#5": 427.0 - 6 * 58,
    "D5": 427.0 - 7 * 58,
    "D#5": 427.0 - 8 * 58,
    "E5": 427.0 - 9 * 58,
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

