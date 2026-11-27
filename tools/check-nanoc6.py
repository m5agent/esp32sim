#!/usr/bin/env python3
from pathlib import Path
from runpy import run_path

raise SystemExit(run_path(str(Path(__file__).with_name("check-c6-docs.py")))["main"]("nanoc6"))
