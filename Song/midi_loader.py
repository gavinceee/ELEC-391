# midi_loader.py
from mido import MidiFile, tick2second, merge_tracks

NOTE_NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']

def midi_note_to_name(note_num: int) -> str:
    name = NOTE_NAMES[note_num % 12]
    octave = (note_num // 12) - 1
    return f"{name}{octave}"

def load_song_from_midi(path: str,
                        default_tempo: int = 500000,
                        min_duration: float = 0.08,
                        rest_merge: bool = True):
    """
    return: list of (note_name, duration_sec)
    example: [("C4", 0.5), ("D4", 0.5), ("REST", 0.2)]
    """

    mid = MidiFile(path)
    track = merge_tracks(mid.tracks)   # 把多轨按时间顺序合并

    tempo = default_tempo
    abs_time = 0.0

    active_notes = {}   # note_num -> start_time_sec
    events = []         # (start, end, note_name)

    for msg in track:
        # msg.time 在 MIDI 文件里是 delta ticks
        delta_sec = tick2second(msg.time, mid.ticks_per_beat, tempo)
        abs_time += delta_sec

        if msg.type == 'set_tempo':
            tempo = msg.tempo

        elif msg.type == 'note_on' and msg.velocity > 0:
            # 记录这个音开始的绝对时间
            active_notes[msg.note] = abs_time

        elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
            if msg.note in active_notes:
                start_t = active_notes.pop(msg.note)
                end_t = abs_time
                dur = end_t - start_t
                if dur >= min_duration:
                    events.append((start_t, end_t, midi_note_to_name(msg.note)))

    # 按开始时间排序
    events.sort(key=lambda x: x[0])

    # 第一版：只支持单音旋律，不支持和弦
    # 如果同一时刻有多个音，只保留最高音
    melody = []
    i = 0
    while i < len(events):
        same_start = [events[i]]
        j = i + 1
        while j < len(events) and abs(events[j][0] - events[i][0]) < 1e-6:
            same_start.append(events[j])
            j += 1

        chosen = max(same_start, key=lambda e: note_name_to_midi_num(e[2]))
        melody.append(chosen)
        i = j

    # 转成 [(note, duration)]，并补 REST
    song = []
    prev_end = 0.0

    for start_t, end_t, note_name in melody:
        if start_t > prev_end:
            rest_dur = start_t - prev_end
            if rest_dur >= min_duration:
                if rest_merge and song and song[-1][0] == "REST":
                    song[-1] = ("REST", song[-1][1] + rest_dur)
                else:
                    song.append(("REST", rest_dur))

        note_dur = end_t - start_t
        song.append((note_name, note_dur))
        prev_end = max(prev_end, end_t)

    return song

def note_name_to_midi_num(name: str) -> int:
    # 例如 C4, D#5
    if len(name) < 2:
        raise ValueError(f"Bad note name: {name}")

    if name[1] == '#':
        pitch = name[:2]
        octave = int(name[2:])
    else:
        pitch = name[:1]
        octave = int(name[1:])

    return NOTE_NAMES.index(pitch) + (octave + 1) * 12


if __name__ == "__main__":
    midi_path = r"E:\ELEC 391\code\test1\test1\Python\Song\Song_list\twinkle-twinkle-little-star.mid"
    song = load_song_from_midi(midi_path)
    print([(note, round(dur, 2)) for note, dur in song])