#!/usr/bin/env python


def get_offset(offset, pos):
    delta = offset - pos
    return delta % 0.5


if __name__ == "__main__":
    while True:
        offset = input("Offset in Phoneix Tuner: ")
        pos = input("Absolute position required for module to be zeroed: ")
        print("set offset in code to: ", get_offset(float(offset), float(pos)))
