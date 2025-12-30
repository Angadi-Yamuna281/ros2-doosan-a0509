import rclpy
from rclpy.node import Node
import random
import sys
import time

# -------------------------------
# Robot setup for A0509
# -------------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "a0509"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class TicTacToe:
    def __init__(self):
        self.board = [[" " for _ in range(3)] for _ in range(3)]
        self.user_symbol = "X"
        self.robot_symbol = "O"

    def print_board(self):
        print("\nCurrent Board:", flush=True)
        for i, row in enumerate(self.board):
            print(" " + " | ".join(row) + " ", flush=True)
            if i < 2:
                print("---+---+---", flush=True)
        print("", flush=True)

    def check_winner(self, symbol):
        b = self.board
        for i in range(3):
            if all(b[i][j] == symbol for j in range(3)):  # row
                return True
            if all(b[j][i] == symbol for j in range(3)):  # column
                return True
        if all(b[i][i] == symbol for i in range(3)):      # diagonal
            return True
        if all(b[i][2 - i] == symbol for i in range(3)):  # reverse diagonal
            return True
        return False

    def is_full(self):
        return all(cell != " " for row in self.board for cell in row)

    def user_move(self):
        while True:
            try:
                sys.stdout.write("🧍 Enter your move (row col) e.g. '1 3': ")
                sys.stdout.flush()
                line = sys.stdin.readline()
                if not line:
                    print("\nNo input detected. Exiting game.", flush=True)
                    return None
                parts = line.strip().split()
                if len(parts) != 2:
                    print("⚠️  Enter two numbers separated by space.", flush=True)
                    continue
                r, c = map(int, parts)
                if 1 <= r <= 3 and 1 <= c <= 3 and self.board[r - 1][c - 1] == " ":
                    self.board[r - 1][c - 1] = self.user_symbol
                    return (r - 1, c - 1)
                else:
                    print("❌ Invalid move or occupied cell. Try again.", flush=True)
            except ValueError:
                print("⚠️  Enter valid integers between 1 and 3.", flush=True)

    def robot_move(self):
        empty = [(i, j) for i in range(3) for j in range(3) if self.board[i][j] == " "]
        if not empty:
            return None
        move = random.choice(empty)
        self.board[move[0]][move[1]] = self.robot_symbol
        return move


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dsr_tictactoe_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, move_periodic, move_spiral,
            set_velx, set_accx,
            DR_BASE, DR_TOOL, DR_AXIS_X
        )
    except ImportError as e:
        print(f"❌ Error importing DSR_ROBOT2 : {e}", flush=True)
        rclpy.shutdown()
        return

    node.get_logger().info("✅ DSR_ROBOT2 imported successfully. Starting Tic Tac Toe on A0509...")

    # Set motion parameters for A0509
    set_velx(20, 15)
    set_accx(50, 30)

    # Ready pose
    JReady = [0, -10, 90, 0, 60, 0]
    movej(JReady, v=20, a=20)

    # -------------------------------
    # ✅ Fine-tuned Tic Tac Toe grid positions for A0509
    # Row 1 = front (closest)
    # Row 3 = back (farthest)
    # -------------------------------
    grid_positions = [
        # Row 1 – Front
        [[260, -110, 200, 180, 0, 180],
         [260,    0, 200, 180, 0, 180],
         [260,  110, 200, 180, 0, 180]],

        # Row 2 – Middle
        [[430, -110, 200, 180, 0, 180],
         [430,    0, 200, 180, 0, 180],
         [430,  110, 200, 180, 0, 180]],

        # Row 3 – Back
        [[580, -110, 200, 180, 0, 180],
         [580,    0, 200, 180, 0, 180],
         [580,  110, 200, 180, 0, 180]],
    ]

    game = TicTacToe()

    try:
        while True:
            game.print_board()
            print("🧍 Your turn!", flush=True)
            user_rc = game.user_move()
            if user_rc is None:
                break

            game.print_board()
            ur, uc = user_rc
            user_pos = grid_positions[ur][uc]
            print(f"➡️  A0509 marking your position ({ur+1},{uc+1}) at {user_pos}", flush=True)

            # Move sequence
            movel([user_pos[0], user_pos[1], user_pos[2] + 80, *user_pos[3:]],
                  v=[50, 50], a=[150, 150], ref=DR_BASE)
            movel(user_pos, v=[50, 50], a=[150, 150], ref=DR_BASE)
            movel([user_pos[0], user_pos[1], user_pos[2] + 80, *user_pos[3:]],
                  v=[50, 50], a=[150, 150], ref=DR_BASE)
            movej(JReady, v=30, a=30)

            if game.check_winner(game.user_symbol):
                print("🎉 You WIN! A0509 celebrates!", flush=True)
                move_periodic([0, 0, 0, 15, 15, 0],
                              [0, 0, 0, 1, 1, 0],
                              0, 1, ref=DR_TOOL)
                break
            if game.is_full():
                print("🤝 Draw! No more moves.", flush=True)
                break

            print("🤖 A0509's turn...", flush=True)
            robot_move = game.robot_move()
            if robot_move is None:
                break

            rr, rc = robot_move
            robot_pos = grid_positions[rr][rc]
            print(f"🦾 A0509 marking its position ({rr+1},{rc+1}) at {robot_pos}", flush=True)

            movel([robot_pos[0], robot_pos[1], robot_pos[2] + 80, *robot_pos[3:]],
                  v=[50, 50], a=[150, 150], ref=DR_BASE)
            movel(robot_pos, v=[50, 50], a=[150, 150], ref=DR_BASE)
            movel([robot_pos[0], robot_pos[1], robot_pos[2] + 80, *robot_pos[3:]],
                  v=[50, 50], a=[150, 150], ref=DR_BASE)
            movej(JReady, v=30, a=30)

            game.print_board()

            if game.check_winner(game.robot_symbol):
                print("🤖 A0509 WINS! Performing victory dance!", flush=True)
                move_periodic([0, 0, 0, 20, 20, 0],
                              [0, 0, 0, 1, 1, 0],
                              0, 1, ref=DR_TOOL)
                move_spiral(rev=2, rmax=60, lmax=30,
                            v=[200, 200], a=[100, 100],
                            axis=DR_AXIS_X, ref=DR_TOOL)
                break
            if game.is_full():
                print("🤝 Draw! No more moves.", flush=True)
                break

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user. Exiting...", flush=True)

    finally:
        print("\n🔁 Resetting A0509 to ready pose...", flush=True)
        movej(JReady, v=20, a=20)
        node.get_logger().info("✅ Game over. Shutting down node.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
