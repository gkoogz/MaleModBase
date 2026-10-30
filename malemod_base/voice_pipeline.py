"""Character-independent voice IDs, scripts and reviewed PCM delivery manifests."""
import csv
import hashlib
import json
import re
import wave
from pathlib import Path, PurePosixPath

COLUMNS = ('tag','line_id','character_id','cue','variant','text','revision','status','recording')
ID = re.compile(r'MM-VOICE-[0-9]{4,}')
CHARACTER = re.compile(r'[a-z][a-z0-9_-]*')
STATUSES = {'needs_recording','recorded','approved','retired'}


def numeric_tag(identity):
    if not ID.fullmatch(identity):raise ValueError('Invalid voice ID for numeric tag')
    number=int(identity.rsplit('-',1)[1])
    if number<1 or identity!='MM-VOICE-'+str(number).zfill(4):raise ValueError('Noncanonical voice ID for numeric tag')
    return str(number)


def catalog(path):
    path=Path(path);data=json.loads(path.read_text(encoding='utf-8-sig'))
    if data.get('schema')!='malemod.voice-catalog/1':raise ValueError('Unknown voice catalog')
    ids=set();slots=set()
    for row in data['lines']:
        identity=row['line_id'];slot=(row['cue'],row['variant'])
        if not ID.fullmatch(identity) or identity in ids:raise ValueError('Duplicate or invalid voice ID')
        numeric_tag(identity)
        if not re.fullmatch(r'[a-z][a-z0-9_.]*',row['cue']) or type(row['variant'])is not int or row['variant']<1 or slot in slots:
            raise ValueError('Duplicate or invalid cue slot')
        if row['status']not in ('active','retired'):raise ValueError('Invalid catalog status')
        ids.add(identity);slots.add(slot)
    if not ids:raise ValueError('Empty voice catalog')
    lock=path.with_suffix('.lock.json')
    if lock.exists():
        frozen=json.loads(lock.read_text(encoding='utf-8'))
        if frozen.get('schema')!='malemod.voice-id-lock/1':raise ValueError('Unknown permanent ID lock')
        current={r['line_id']:(r['cue'],r['variant'])for r in data['lines']}
        for row in frozen['lines']:
            if current.get(row['line_id'])!=(row['cue'],row['variant']):raise ValueError('Published voice ID removed or reassigned: '+row['line_id'])
    return data


def next_id(data):
    return 'MM-VOICE-'+str(max(int(r['line_id'].rsplit('-',1)[1])for r in data['lines'])+1).zfill(4)


def scaffold(data, character):
    if not CHARACTER.fullmatch(character):raise ValueError('Invalid character ID')
    return [dict(tag=numeric_tag(r['line_id']),line_id=r['line_id'],character_id=character,cue=r['cue'],variant=str(r['variant']),
                 text='',revision='1',status='needs_recording',recording='')for r in data['lines']if r['status']=='active']


def read_script(path):
    with Path(path).open(encoding='utf-8-sig',newline='')as f:
        reader=csv.DictReader(f)
        fields=tuple(reader.fieldnames or[])
        if fields not in (COLUMNS,COLUMNS[1:]):raise ValueError('Voice CSV columns differ from the pipeline contract')
        rows=list(reader)
    if any(None in row or any(v is None for v in row.values())for row in rows):raise ValueError('Malformed voice CSV row')
    # Earlier eight-column scripts remain readable without rewriting their text.
    if fields==COLUMNS[1:]:
        rows=[dict(tag=numeric_tag(row['line_id']),**row)for row in rows]
    return rows


def pcm(path):
    with wave.open(str(path),'rb')as audio:
        count=audio.getnframes()
        if (audio.getnchannels(),audio.getframerate(),audio.getsampwidth(),audio.getcomptype())!=(1,44100,2,'NONE')or count<1:
            raise ValueError('Voice recording must be nonempty mono 44100 Hz 16-bit PCM WAV')
        if len(audio.readframes(count))!=count*2:raise ValueError('Truncated voice recording')
    return dict(frames=count,sampleRate=44100,channels=1,sampleBytes=2,durationSeconds=count/44100)


def validate(data, rows, audio_root=None, require_approved=False):
    expected={r['line_id']:r for r in data['lines']if r['status']=='active'}
    seen=set();characters=set();clips=[];statuses={}
    root=Path(audio_root).resolve()if audio_root is not None else None
    for row in rows:
        identity=row['line_id']
        if identity not in expected or identity in seen:raise ValueError('Unknown, retired or duplicate voice ID: '+identity)
        if row.get('tag')!=numeric_tag(identity):raise ValueError('Numeric tag differs from its permanent voice ID: '+identity)
        seen.add(identity);original=expected[identity]
        if row['cue']!=original['cue']or row['variant']!=str(original['variant']):raise ValueError('Voice ID was reassigned to another cue')
        if not CHARACTER.fullmatch(row['character_id']):raise ValueError('Invalid character ID')
        characters.add(row['character_id'])
        if not re.fullmatch(r'[1-9][0-9]*',row['revision']):raise ValueError('Invalid line revision')
        if row['status']not in STATUSES or row['status']=='retired':raise ValueError('Invalid active line status')
        statuses[row['status']]=statuses.get(row['status'],0)+1
        if require_approved and row['status']!='approved':raise ValueError('Voice line is not approved: '+identity)
        if row['status']=='needs_recording':
            if row['recording']:raise ValueError('Unreviewed line must not bind an old recording')
            continue
        if not row['text'].strip()or not row['recording']:raise ValueError('Recorded lines require text and a recording')
        relative=PurePosixPath(row['recording'])
        if ('\\'in row['recording']or ':'in row['recording']or relative.is_absolute()or '..'in relative.parts
                or relative.as_posix()!=row['recording']or relative.suffix!='.wav'):
            raise ValueError('Recording must be a normalized relative WAV path')
        if root is None:raise ValueError('An explicit private audio root is required')
        path=root/relative
        if not path.resolve().is_relative_to(root)or path.is_symlink()or not path.is_file():raise ValueError('Recording escapes its private audio root or is missing')
        details=pcm(path)
        if row['status']=='approved':clips.append(dict(tag=int(row['tag']),line_id=identity,character_id=row['character_id'],cue=row['cue'],
            variant=original['variant'],revision=int(row['revision']),recording=relative.as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),**details))
    if set(expected)!=seen:raise ValueError('Voice script omits catalog IDs')
    if len(characters)!=1:raise ValueError('One character per voice script is required')
    return dict(schema='malemod.voice-delivery/1',character_id=next(iter(characters)),lineCount=len(rows),statuses=statuses,
                ready=len(clips)==len(rows),clips=clips)
