# utils/config.py
import yaml
import os

DEFAULT_CONFIG = {
    "radio": {
        "host": "0.0.0.0",
        "port": 9000,
    },
    "sitl": {
        "enabled": False,
        # example: "udpout:127.0.0.1:14550" or "/dev/ttyUSB0"
        "connection_url": "127.0.0.1:14550"
    },
    "gcs": {
        "controller_name": "GCS",
        "log_level": "INFO"
    }
}

def load_config(path="config/config.yaml"):
    if not os.path.exists(path):
        # create a minimal config file
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            yaml.safe_dump(DEFAULT_CONFIG, f)
        return DEFAULT_CONFIG.copy()
    with open(path, "r") as f:
        cfg = yaml.safe_load(f) or {}
    # merge defaults
    def merge(dflt, v):
        out = {}
        for k, val in dflt.items():
            if k in v and isinstance(val, dict):
                out[k] = merge(val, v.get(k, {}))
            else:
                out[k] = v.get(k, val)
        return out
    return merge(DEFAULT_CONFIG, cfg)
