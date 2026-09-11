"""Run the hardware-free checks from any working directory, using Python 3."""
import sys
import unittest
from pathlib import Path

tests = Path(__file__).resolve().parent
client_files = tests.parent
server_files = client_files.parent / "server_files"
sys.path[:0] = [str(client_files), str(server_files)]

if __name__ == "__main__":
    suite = unittest.defaultTestLoader.discover(str(tests))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
