"""Compile/check the portable final-topology lighting utility without game SDKs."""
import hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    cmake=ROOT/'build/cmake-deps/cmake/data/bin/cmake.exe'
    job=ROOT/'build/target-lighting';job.mkdir(parents=True,exist_ok=True)
    (job/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\nproject(TargetLighting LANGUAGES CXX)\n'
        +'add_executable(target_lighting "'+(ROOT/'tests/surface_lighting_test.cpp').as_posix()+'")\n'
        +'target_include_directories(target_lighting PRIVATE "'+(ROOT/'include').as_posix()+'")\n'
        +'target_compile_features(target_lighting PRIVATE cxx_std_17)\n')
    subprocess.run([str(cmake),'-S',str(job),'-B',str(job/'cmake'),'-A','x64'],check=True)
    subprocess.run([str(cmake),'--build',str(job/'cmake'),'--config','Release'],check=True)
    exe=job/'cmake/Release/target_lighting.exe';r=subprocess.run([str(exe)],capture_output=True,text=True,check=True)
    report=job/'report.txt';report.write_text(r.stdout+r.stderr);print(r.stdout,end='')
    manifest=dict(files=[dict(path=p,sha256=sha(ROOT/p)) for p in ['include/malemod/surface/lighting.hpp','tests/surface_lighting_test.cpp','tools/verify_target_lighting.py']],
                  sdkFree=True,executableSHA256=sha(exe),reportSHA256=sha(report),exitCode=r.returncode,
                  sourceSolverChanged=False,wolverineChanged=False,observedGameplay=False)
    (ROOT/'provenance/target-lighting.json').write_text(json.dumps(manifest,indent=2)+'\n')
if __name__=='__main__':main()
