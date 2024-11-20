import unittest
from unittest.mock import patch
from main import Robot

class TestRobot(unittest.TestCase):
    
    @patch('builtins.input', side_effect=['d'])
    def test_rotatecube_down_correct_input(self, mock_input):
        robot = Robot()
        with patch('builtins.print') as mock_print:
            robot.rotatecube_down()
            mock_print.assert_called_with("Please rotate the cube down and press 'd' when done.")
        self.assertEqual(mock_input.call_count, 1)

    @patch('builtins.input', side_effect=['x', 'd'])
    def test_rotatecube_down_incorrect_then_correct_input(self, mock_input):
        robot = Robot()
        with patch('builtins.print') as mock_print:
            robot.rotatecube_down()
            mock_print.assert_called_with("Please rotate the cube down and press 'd' when done.")
        self.assertEqual(mock_input.call_count, 2)

if __name__ == '__main__':
    unittest.main()