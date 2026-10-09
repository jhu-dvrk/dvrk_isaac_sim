#!/usr/bin/env python3
"""Print the historical Isaac cart layout using the shared frame generator."""

from dvrk_simulator_base.cart_frames import yaml_frames


def main():
    print(yaml_frames({"PSM1": (180.0, 30.0), "PSM2": (0.0, 30.0),
                       "PSM3": (0.0, 60.0), "ECM": (0.0, 0.0)}))


if __name__ == "__main__":
    main()
