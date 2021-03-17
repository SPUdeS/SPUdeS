import unittest
from stewartPlatform import stewartPlatform
import configFile as config

class TestStewartPlatformMethods(unittest.TestCase):

    def setUp(cls):
        cls.sp = stewartPlatform()

    def testOriginPositionPlatform(self):
        self.assertEqual(self.sp.platform.origin, config.platformHomePosition)

    def testGetRotationMatrix(self):
        self.assertEqual(self.sp.getBasePlatformRotationMatrix, config.platformHomePosition)

if __name__ == '__main__':
    unittest.main()