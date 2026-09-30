#pragma once
// Play user-supplied patient-study recordings at the two timeline cues.
// Recordings live outside the redistributable package.
namespace teaching {
struct PatientAudio {
 static const unsigned capacity=64;
 char paths[2][capacity][MAX_PATH]{};unsigned count[2]{};bool fired[2]{};bool active=false,loaded=false,playing=false;
 unsigned state=0x7f4a7c15u;
 void Seed(){LARGE_INTEGER now{};QueryPerformanceCounter(&now);state=(unsigned)now.LowPart^(unsigned)now.HighPart^GetTickCount()^GetCurrentProcessId()^GetCurrentThreadId();if(!state)state=0x7f4a7c15u;}
 static unsigned ReadLE16(const unsigned char* b){return unsigned(b[0])|(unsigned(b[1])<<8);}
 static unsigned ReadLE32(const unsigned char* b){return unsigned(b[0])|(unsigned(b[1])<<8)|(unsigned(b[2])<<16)|(unsigned(b[3])<<24);}
 static bool ValidPcmWav(const char* path){
  FILE* f=nullptr;fopen_s(&f,path,"rb");if(!f)return false;_fseeki64(f,0,SEEK_END);__int64 size=_ftelli64(f);_fseeki64(f,0,SEEK_SET);
  unsigned char root[12]{};bool rootOkay=size>=44&&fread(root,1,12,f)==12&&!memcmp(root,"RIFF",4)&&!memcmp(root+8,"WAVE",4);bool formatOkay=false,dataOkay=false;
  for(__int64 pos=12;rootOkay&&pos+8<=size;){if(_fseeki64(f,pos,SEEK_SET)!=0)break;unsigned char head[8]{};if(fread(head,1,8,f)!=8)break;unsigned bytes=ReadLE32(head+4);__int64 next=pos+8+bytes+(bytes&1);if(next>size)break;
   if(!memcmp(head,"fmt ",4)&&bytes>=16){unsigned char fmt[16];if(fread(fmt,1,16,f)!=16)break;formatOkay=ReadLE16(fmt)==1&&ReadLE16(fmt+2)==1&&ReadLE32(fmt+4)==44100&&ReadLE16(fmt+12)==2&&ReadLE16(fmt+14)==16;}
   if(!memcmp(head,"data",4))dataOkay=bytes>=2;pos=next;
  }
  fclose(f);return rootOkay&&formatOkay&&dataOkay;
 }
 void Load(){
  if(loaded)return;loaded=true;
  char root[MAX_PATH];GetModuleFileNameA((HMODULE)&__ImageBase,root,MAX_PATH);char* slash=strrchr(root,'\\');if(!slash)return;slash[1]=0;Seed();
  for(unsigned phase=0;phase<2;phase++){
   char pattern[MAX_PATH];sprintf_s(pattern,"%sTeachingAudio\\Phase%u\\*.wav",root,phase+1);WIN32_FIND_DATAA file{};HANDLE h=FindFirstFileA(pattern,&file);
   if(h==INVALID_HANDLE_VALUE)continue;
   do{if(file.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY)continue;if(count[phase]>=capacity)break;
    char candidate[MAX_PATH];sprintf_s(candidate,"%sTeachingAudio\\Phase%u\\%s",root,phase+1,file.cFileName);
    if(!ValidPcmWav(candidate)){Log("patient audio phase %u ignored unsupported WAV",phase+1);continue;}
    unsigned n=count[phase]++;strcpy_s(paths[phase][n],candidate);
   }while(FindNextFileA(h,&file));FindClose(h);
  }
  Log("patient audio pools loaded: phase1=%u phase2=%u (PCM WAV)",count[0],count[1]);
 }
 static float Cue(int phase){return phase==0?peaks[0]+1.f:peaks[4];}
 void Begin(){fired[0]=fired[1]=false;active=true;playing=false;}
 void End(bool stop){if(stop&&playing)PlaySoundA(nullptr,nullptr,0);active=false;playing=false;}
 unsigned Select(unsigned phase){state^=state<<13;state^=state>>17;state^=state<<5;return count[phase]?state%count[phase]:0;}
 bool Crossing(int phase,float previous,float current)const{return active&&!fired[phase]&&previous<Cue(phase)&&current>=Cue(phase);}
 void Advance(float previous,float current){for(int phase=0;phase<2;phase++)if(Crossing(phase,previous,current)){
  fired[phase]=true;if(!count[phase]){Log("patient audio phase %d cue reached; no WAVs installed",phase+1);continue;}
  unsigned pick=Select(phase);if(PlaySoundA(paths[phase][pick],nullptr,SND_FILENAME|SND_ASYNC|SND_NODEFAULT)){playing=true;Log("patient audio phase %d played clip %u of %u",phase+1,pick+1,count[phase]);}else Log("patient audio phase %d could not play clip %u",phase+1,pick+1);
 }}
};
}

