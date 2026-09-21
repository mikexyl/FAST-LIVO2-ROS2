"""Stable robot colours for dataset-independent reports and Rerun."""
import colorsys


def robot_colors(robots):
    fixed = {'Alpha': [235, 85, 80], 'Bob': [70, 170, 245], 'Carol': [95, 205, 130]}
    palette = [[235, 85, 80], [70, 170, 245], [95, 205, 130], [190, 120, 235],
               [235, 180, 60], [60, 200, 200]]
    return {robot: fixed.get(robot, palette[i] if i < len(palette) else
            [round(255 * x) for x in colorsys.hsv_to_rgb((i * .618034) % 1, .65, .9)])
            for i, robot in enumerate(robots)}
