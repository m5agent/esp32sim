#!/usr/bin/env python3
from runpy import run_path
from pathlib import Path

run_path(str(Path(__file__).with_name("run-c6-docs.py")))["run"]("nesso-n1")
