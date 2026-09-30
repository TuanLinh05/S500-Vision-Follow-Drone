#!/usr/bin/env python3
import sys

from s500_companion.cli import main


if __name__ == "__main__":
    raise SystemExit(main(["record", *sys.argv[1:]]))

