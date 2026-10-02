"""Exercise the actual cached CG Anem driver with a bounded Stream timing model.

Stage the pinned libraries offline first (tools/prepare_libraries.py).
No device access, downloads, or changes to the driver are performed.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from run_tests import sketch_translation_unit


def main():
    root = Path(__file__).resolve().parents[1]
    driver = root / ".build/libraries/ClimateGuard_CG_Anem-1.0.3/src"
    if not (driver / "cgAnem.cpp").is_file():
        raise SystemExit("Stage the pinned CG Anem 1.0.3 library offline before this test")
    compiler = shutil.which(os.environ.get("CXX", "g++"))
    if not compiler:
        raise SystemExit("C++ compiler unavailable")
    env = os.environ.copy()
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="windos-driver-") as folder:
        build = Path(folder)
        for source in (root / "tests/host_stubs").glob("*.h"):
            shutil.copyfile(source, build / source.name)
        mock = (build / "MockHardware.h").read_text()
        begin = mock.index("#define ANEM_I2C_ADDR")
        end = mock.index("struct sensors_event_t", begin)
        mock = mock[:begin] + '#include "' + (driver / "cgAnem.h").as_posix() + '"\n' + mock[end:]
        mock = mock.replace("  bool selectionFails = false;", r"""
  int failAt = 0, streamReads = 0;
  bool bufferMissing = false;
  int requestFrom(uint8_t, int n) {
    ++requests;
    bufferMissing = failAt && requests >= failAt;
    return bufferMissing ? 0 : n;
  }
  size_t readBytes(uint8_t *out, size_t n) {
    ++streamReads;
    if (bufferMissing) { delay(streamTimeout); return 0; }
    *out = pendingValue == 0x08 ? 32 : pendingValue == 0x11 ? 200 : 0;
    return n;
  }
  bool selectionFails = false;""", 1)
        (build / "MockHardware.h").write_text(mock)
        (build / "cgAnem.h").write_text('#include "' + (driver / "cgAnem.h").as_posix() + '"\n')
        unit = sketch_translation_unit(root, False)
        unit += '#include "' + (driver / "cgAnem.cpp").as_posix() + '"\n'
        unit += r"""
#include "TestChecks.h"
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv) {
  CHECK(argc == 2);
  startupComplete = active = runDataEntryComplete = trialDataEntryComplete = true;
  originalRunTime = 15; numTrials = 1; baroyes = loadcellyes = false;
  Wire.setWireTimeout(25000, true);
  updateAirflow(); CHECK(airflowSample.fresh(mockMillis));
  beginTrial(); motorGo(1500);
  Wire.requests = Wire.streamReads = 0; Wire.failAt = atoi(argv[1]);
  const uint32_t start = mockMillis;
  loop();
  CHECK(stopLatched && motor.lastPulse == windos::kStopPulseUs);
  CHECK(Wire.streamTimeout == 25 && mockMillis - start <= 150);
  printf("PASS: pinned driver disconnect at request %d: reads=%d elapsedMs=%u\n",
         Wire.failAt, Wire.streamReads, unsigned(mockMillis - start));
}
"""
        path = build / "driver.cpp"
        path.write_text(unit)
        exe = build / ("driver.exe" if os.name == "nt" else "driver")
        subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror=return-type",
                        "-Wno-unused-variable", "-Wno-sign-compare", "-Wno-parentheses",
                        "-I" + str(build), "-I" + str(root), "-I" + str(root / "tests"),
                        str(path), "-o", str(exe)], check=True, env=env, timeout=90)
        for fail_at in range(1, 8):
            subprocess.run([str(exe), str(fail_at)], check=True, env=env, timeout=10)


if __name__ == "__main__":
    main()
