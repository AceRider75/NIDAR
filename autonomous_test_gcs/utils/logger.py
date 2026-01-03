# utils/logger.py
import logging
from logging import StreamHandler, Formatter

def setup_logger(name="gcs", level="INFO"):
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    handler = StreamHandler()
    handler.setFormatter(Formatter("%(asctime)s | %(levelname)-5s | %(name)s | %(message)s"))
    logger.addHandler(handler)
    logger.propagate = False
    return logger
