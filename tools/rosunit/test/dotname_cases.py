import unittest


class CaseA(unittest.TestCase):

    def runTest(self):
        self.assertTrue(True)


class CaseB(unittest.TestCase):

    def runTest(self):
        self.assertTrue(True)


class DotnameLoadingSuite(unittest.TestSuite):

    def __init__(self):
        super(DotnameLoadingSuite, self).__init__()
        self.addTest(CaseA())
        self.addTest(CaseB())


class DotnameLoadingTest(unittest.TestCase):

    def test_a(self):
        self.assertTrue(True)

    def test_b(self):
        self.assertTrue(True)


class NotTestCase():

    def not_test(self):
        pass
