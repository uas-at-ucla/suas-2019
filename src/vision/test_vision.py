import sys
sys.dont_write_bytecode = True

from unittest import TestLoader, TestSuite, TextTestRunner

if __name__ == "__main__":
    loader = TestLoader()
    suite = TestSuite((
    ))

    runner = TextTestRunner(verbosity = 2)
    runner.run(suite)
