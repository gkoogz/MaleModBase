#pragma once
// CPU wall time, inclusive of nested scopes; these are not GPU frame times.
struct PerfMetric {const char* name;double total=0,peak=0;unsigned count=0;};
static PerfMetric perfMetrics[]={{"hook"},{"overlay"},{"physics"},{"menu surface"},{"rest shape"},{"refinement"},{"pelvic attachment"},{"rounded surface"},{"gameplay surface"},{"collision"},{"texture load"},{"pouch surface"},{"structural skin"},{"neck render"},{"collar solve"},{"junction finish"}};
static double PerfClock(){static double scale=[](){LARGE_INTEGER f;QueryPerformanceFrequency(&f);return 1000./double(f.QuadPart);}();LARGE_INTEGER t;QueryPerformanceCounter(&t);return double(t.QuadPart)*scale;}
struct PerfScope {int id;double begin;PerfScope(int value):id(value),begin(PerfClock()){}~PerfScope(){double elapsed=PerfClock()-begin;auto& p=perfMetrics[id];p.total+=elapsed;p.peak=p.peak>elapsed?p.peak:elapsed;++p.count;}};
