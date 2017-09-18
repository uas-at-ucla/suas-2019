import sys
sys.dont_write_bytecode = True

from unittest import TestLoader, TestSuite, TextTestRunner
from photographer.test_photographer import TestPhotographer
from segmenter.test_segmenter import TestSegmenter
from classifier.test_classifier import TestClassifier

if __name__ == "__main__":
    loader = TestLoader()
    suite = TestSuite((
        loader.loadTestsFromTestCase(TestPhotographer),
        loader.loadTestsFromTestCase(TestSegmenter),
        loader.loadTestsFromTestCase(TestClassifier),
    ))

    runner = TextTestRunner(verbosity = 2)
    runner.run(suite)
