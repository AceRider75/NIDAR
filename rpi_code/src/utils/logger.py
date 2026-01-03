"""Logging utilities for the application."""
import logging
import sys
from pathlib import Path
from datetime import datetime


def get_logger(name: str = None, level: int = logging.INFO) -> logging.Logger:
    """
    Get or create a logger instance.
    
    Args:
        name: Logger name (defaults to calling module)
        level: Logging level
        
    Returns:
        Configured logger instance
    """
    logger_name = name if name else __name__
    logger = logging.getLogger(logger_name)
    
    if not logger.handlers:
        logger.setLevel(level)
        
        # Console handler
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(level)
        
        # Formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        
        logger.addHandler(handler)
    
    return logger
