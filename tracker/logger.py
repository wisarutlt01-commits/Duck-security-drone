"""
tracker/logger.py
=================
Configures a logger with both console and rotating file handlers.
"""

import logging
import os
from logging.handlers import RotatingFileHandler


def setup_logger(
    name:         str,
    log_file:     str = "logs/tracker.log",
    level:        int = logging.DEBUG,
    max_bytes:    int = 5 * 1024 * 1024,  # 5 MB per file
    backup_count: int = 3,
) -> logging.Logger:
    """
    Configure the root logger once, then return a named child logger.

    Because all loggers propagate to root by default, every module that calls
    logging.getLogger("anything") will automatically have its messages routed
    to the console and log file — no per-module setup required.
    """
    os.makedirs(
        os.path.dirname(log_file) if os.path.dirname(log_file) else ".",
        exist_ok=True,
    )

    root = logging.getLogger()
    if not root.handlers:
        fmt = logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
            datefmt="%H:%M:%S",
        )

        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(fmt)
        root.addHandler(ch)

        fh = RotatingFileHandler(
            log_file, maxBytes=max_bytes, backupCount=backup_count
        )
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(fmt)
        root.addHandler(fh)

        root.setLevel(logging.DEBUG)

    return logging.getLogger(name)
