import copy,tempfile,unittest,wave,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.voice_pipeline import catalog,next_id,scaffold,validate

class VoicePipelineTests(unittest.TestCase):
    def setUp(self):
        self.data={'schema':'malemod.voice-catalog/1','lines':[{'line_id':'MM-VOICE-0001','cue':'sequence.phase1','variant':1,'status':'active'}]}
        self.rows=scaffold(self.data,'geralt');self.rows[0]['text']='Existing source line.'
    def test_character_retake_preserves_identity(self):
        other=scaffold(self.data,'future_character')
        self.assertEqual(other[0]['line_id'],self.rows[0]['line_id'])
        self.rows[0]['revision']='2'
        self.assertFalse(validate(self.data,self.rows)['ready'])
    def test_silent_plan_cannot_be_packaged_as_approved(self):
        with self.assertRaisesRegex(ValueError,'not approved'):validate(self.data,self.rows,require_approved=True)
    def test_duplicate_and_missing_ids_are_rejected(self):
        for rows in [[],self.rows+self.rows]:
            with self.assertRaises(ValueError):validate(self.data,rows)
    def test_id_cannot_change_cue(self):
        self.rows[0]['cue']='different.cue'
        with self.assertRaisesRegex(ValueError,'reassigned'):validate(self.data,self.rows)
    def test_retired_ids_are_never_reused_by_allocator(self):
        self.data['lines'][0]['status']='retired'
        self.assertEqual(next_id(self.data),'MM-VOICE-0002')
    def test_old_recording_cannot_survive_needs_recording_state(self):
        self.rows[0]['recording']='old.wav'
        with self.assertRaisesRegex(ValueError,'old recording'):validate(self.data,self.rows)
    def test_approved_pcm_receipt_and_wrong_format(self):
        with tempfile.TemporaryDirectory()as d:
            root=Path(d);path=root/'MM-VOICE-0001.wav'
            def write(channels):
                with wave.open(str(path),'wb')as w:w.setnchannels(channels);w.setsampwidth(2);w.setframerate(44100);w.writeframes(b'\0\0'*100*channels)
            write(1);self.rows[0].update(status='approved',recording=path.name)
            result=validate(self.data,self.rows,root,True)
            self.assertTrue(result['ready']);self.assertEqual(result['clips'][0]['frames'],100)
            self.assertEqual(len(result['clips'][0]['sha256']),64)
            write(2)
            with self.assertRaisesRegex(ValueError,'mono'):validate(self.data,self.rows,root,True)
    def test_recording_traversal_rejected(self):
        self.rows[0]['status']='approved'
        for bad in ['../outside.wav','C:/outside.wav','/outside.wav','a\\outside.wav']:
            self.rows[0]['recording']=bad
            with self.assertRaisesRegex(ValueError,'relative WAV'):validate(self.data,self.rows,Path.cwd(),True)
    def test_one_character_and_positive_revision(self):
        self.rows[0]['revision']='0'
        with self.assertRaisesRegex(ValueError,'revision'):validate(self.data,self.rows)
    def test_duplicate_catalog_ids_rejected(self):
        with tempfile.TemporaryDirectory()as d:
            path=Path(d)/'catalog.json';self.data['lines']*=2;path.write_text(json.dumps(self.data))
            with self.assertRaisesRegex(ValueError,'Duplicate'):catalog(path)
    def test_published_id_lock_prevents_reassignment_and_deletion(self):
        with tempfile.TemporaryDirectory()as d:
            path=Path(d)/'catalog.json';frozen=copy.deepcopy(self.data['lines'])
            path.with_suffix('.lock.json').write_text(json.dumps(dict(schema='malemod.voice-id-lock/1',lines=frozen)))
            self.data['lines'][0]['cue']='other.cue';path.write_text(json.dumps(self.data))
            with self.assertRaisesRegex(ValueError,'reassigned'):catalog(path)
            self.data['lines'][0]['cue']='sequence.phase1';self.data['lines'][0]['status']='retired';path.write_text(json.dumps(self.data))
            self.assertEqual(catalog(path)['lines'][0]['status'],'retired')
if __name__=='__main__':unittest.main()
