"""System health sampling: CPU/mem (psutil), SoC temps (thermal zones),
BPU load (/sys/devices/system/bpu/ratio), WiFi RSSI (/proc/net/wireless).

Verified on this RDK X5: thermal_zone0 = thermal-ddr, thermal_zone1 =
thermal-cpu (milli-degC); bpu/ratio is a bare integer percentage."""

import glob
import os

import psutil

_BPU_PATHS = ("/sys/devices/system/bpu/ratio",
              "/sys/devices/system/bpu/bpu0/ratio")


class HealthSampler:
    def __init__(self):
        self._zones = []
        for tz in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
            try:
                with open(os.path.join(tz, "type")) as f:
                    name = f.read().strip().replace("thermal-", "")
                self._zones.append((name, os.path.join(tz, "temp")))
            except OSError:
                pass
        self._bpu = next((p for p in _BPU_PATHS if os.path.exists(p)), None)
        psutil.cpu_percent(None)   # prime the delta-based counter

    @staticmethod
    def _read_int(path):
        try:
            with open(path) as f:
                return int(f.read().strip())
        except (OSError, ValueError):
            return None

    def _wifi_dbm(self):
        try:
            with open("/proc/net/wireless") as f:
                for line in f:
                    if ":" in line:
                        # "wlan0: 0000   70.  -27.  -256 ..."
                        parts = line.split()
                        return int(float(parts[3].rstrip(".")))
        except (OSError, ValueError, IndexError):
            pass
        return None

    def sample(self):
        out = {"cpu": psutil.cpu_percent(None),
               "mem": psutil.virtual_memory().percent}
        for name, path in self._zones:
            v = self._read_int(path)
            out[f"temp_{name}_c"] = round(v / 1000.0, 1) if v is not None else None
        out["bpu_pct"] = self._read_int(self._bpu) if self._bpu else None
        out["wifi_dbm"] = self._wifi_dbm()
        return out
