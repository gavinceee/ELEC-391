# ----------------------------
# Physical key → motor degree mapping
# Measured: E3 = -1457 deg, E5 = -208 deg  (24 semitone steps, linear)
# ----------------------------
_KEY_DEGREES = {
    # -------- lower --------
    "E3": 988 + 5 * 57.9,
    "F3": 988 + 4 * 57.9,
    "F#3": 988 + 3 * 57.9,
    "G3": 988 + 2 * 57.9,
    "G#3": 988 + 1 * 57.9,
    "A3": 988,
    "A#3": 988 - 57.9,
    "B3": 988 - 2 * 57.9,

    # -------- mid --------
    "C4": 985,
    "C#4": 988 - 57.9,
    "D4": 988 - 2 * 57.9,
    "D#4": 988 - 3 * 57.9,
    "E4": 988 - 4 * 57.9,
    "F4": 988 - 5 * 57.9,
    "F#4": 988 - 6 * 57.9,

    # -------- anchor --------
    "G4": 583,

    # -------- upper --------
    "G#4": 583 - 66.6,
    "A4": 583 - 2 * 66.6,
    "A#4": 583 - 3 * 66.6,
    "B4": 583 - 4 * 66.6,

    "C5": 250,
    "C#5": 250 - 66.6,
    "D5": 250 - 2 * 66.6,
    "D#5": 250 - 3 * 66.6,
    "E5": 250 - 4 * 66.6,
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

