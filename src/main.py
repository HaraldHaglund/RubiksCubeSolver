from Rubiks_cube_class.rubickscube import RubiksCube
from Rubiks_cube_scanner.scanner import Scanner
import kociemba



class Robot:
    def rotatecube_up(self):
        print("Please rotate the cube up and press 'u' when done.")
        while input().strip().lower() != 'u':
            pass

    def rotatecube_down(self):
        print("Please rotate the cube down and press 'd' when done.")
        while input().strip().lower() != 'd':
            pass

    def rotatecube_right(self):
        print("Please rotate the cube right and press 'r' when done.")
        while input().strip().lower() != 'r':
            pass


def moveToString(move):
    move_descriptions = {
        "F": 'front layer a quarter turn clockwise.',
        "F'": 'front layer a quarter turn counter-clockwise.',
        "F2": 'front layer 180 degrees.',
        "D": 'bottom layer a quarter turn clockwise.',
        "D'": 'bottom layer a quarter turn counter-clockwise.',
        "D2": 'bottom layer 180 degrees.',
        "U": 'top layer a quarter turn clockwise.',
        "U'": 'top layer a quarter turn counter-clockwise.',
        "U2": 'top layer 180 degrees.',
        "R": 'right layer a quarter turn clockwise.',
        "R'": 'right layer a quarter turn counter-clockwise.',
        "R2": 'right layer 180 degrees.',
        "L": 'left layer a quarter turn clockwise.',
        "L'": 'left layer a quarter turn counter-clockwise.',
        "L2": 'left layer 180 degrees.',
        "B": 'back layer a quarter turn clockwise.',
        "B'": 'back layer a quarter turn counter-clockwise.',
        "B2": 'back layer 180 degrees.'
    }

    return 'Turn the ' + move_descriptions.get(move)


def read_cube(scanner, robot, rubiks_cube):
    # Scan the upper face
    upp_face = scanner.scan_face()
    print("Upper Face:")
    rubiks_cube.update_face('U', upp_face)

    # Rotate the cube to scan the front face
    robot.rotatecube_down()
    front_face = scanner.scan_face()
    rubiks_cube.update_face('F', front_face)

    # Rotate the cube to scan the right face
    robot.rotatecube_right()
    right_face = scanner.scan_face()
    rubiks_cube.update_face('R', right_face)

    # Rotate the cube to scan the back face
    robot.rotatecube_right()
    back_face = scanner.scan_face()
    rubiks_cube.update_face('B', back_face)

    # Rotate the cube to scan the left face
    robot.rotatecube_right()
    left_face = scanner.scan_face()
    rubiks_cube.update_face('L', left_face)

    # Rotate the cube to scan the down face
    robot.rotatecube_right()
    robot.rotatecube_down()
    down_face = scanner.scan_face()
    rubiks_cube.update_face('D', down_face)


def main():
    scanner = Scanner()
    robot = Robot()
    rubiks_cube = RubiksCube()


    # rubiks_cube.update_face('U', [['w']*3 for _ in range(3)])
    # rubiks_cube.update_face('L', [['o']*3 for _ in range(3)])
    # rubiks_cube.update_face('F', [['g']*3 for _ in range(3)])
    # rubiks_cube.update_face('R', [['r']*3 for _ in range(3)])
    # rubiks_cube.update_face('B', [['b']*3 for _ in range(3)])
    # rubiks_cube.update_face('D', [['y']*3 for _ in range(3)])

    # new_back = [
    #     ['o', 'o', 'b'],
    #     ['o', 'r', 'o'],
    #     ['o', 'w', 'o']
    # ]
    
    # # Update the Back face
    # rubiks_cube.update_face('B', new_back)

    read_cube(scanner, robot, rubiks_cube)

    while rubiks_cube.check_cube_validity() == False:
        print("Invalid cube. Scaning again.")
        read_cube(scanner, robot, rubiks_cube)

    # Rotate the cube so front face is facing up
    robot.rotatecube_up()


    # read_cube(scanner, robot, rubiks_cube)
    rubiks_cube.print_matrix()

    # Update and print face notation matrix
    rubiks_cube.update_face_notation_matrix()
    rubiks_cube.print_output_matrix()

    # Get face notation string
    notation_str = rubiks_cube.get_face_notation_string()
    print(f"Face Notation String: {notation_str}")
    solution = kociemba.solve(notation_str)


    # Print the tutorial in human-readable format
    # for iteration, move in enumerate(solution.split()): 
    #     print(f'{iteration + 1}. {moveToString(move)}')


if __name__ == '__main__':
    main()

