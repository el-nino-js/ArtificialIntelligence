import unittest
import IntelligenceArtificial1

class MyTest(unittest.TestCase):
    def test(self):
        self.assertEqual(board_moves([["_","O","O","O","_"],["O","_","O","_","O"],["_","O","_","O","_"],["O","_","O","_","_"],["_","O","_","_","_"]]), [[(0, 2), (0, 0)], [(0, 2), (0, 4)], [(0, 2), (2, 2)]] )
