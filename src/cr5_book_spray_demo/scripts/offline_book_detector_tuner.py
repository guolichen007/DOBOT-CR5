#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline 2-D tuner for the green-background rectangle detector.

This is only a visual tuning aid; it cannot estimate 3-D pose without D455 depth.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_triplet(text):
    values = [int(v.strip()) for v in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("Expected H,S,V")
    return np.array(values, dtype=np.uint8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+")
    parser.add_argument("--lower", type=parse_triplet, default=np.array([30, 35, 20], np.uint8))
    parser.add_argument("--upper", type=parse_triplet, default=np.array([100, 255, 255], np.uint8))
    parser.add_argument("--output", default="offline_detector_output")
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    for image_path in args.images:
        path = Path(image_path)
        image = cv2.imread(str(path))
        if image is None:
            print("SKIP unreadable:", path)
            continue
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        background = cv2.inRange(hsv, args.lower, args.upper)
        mask = cv2.bitwise_not(background)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        annotated = image.copy()
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            image_area = image.shape[0] * image.shape[1]
            if area < 0.01 * image_area or area > 0.60 * image_area:
                continue
            rect = cv2.minAreaRect(contour)
            (_, _), (w, h), _ = rect
            if w < 2 or h < 2:
                continue
            rectangularity = area / (w * h)
            aspect = max(w, h) / max(min(w, h), 1.0)
            box = cv2.boxPoints(rect)
            min_x, min_y = np.min(box, axis=0)
            max_x, max_y = np.max(box, axis=0)
            border_sides = sum([
                min_x <= 8,
                min_y <= 8,
                max_x >= image.shape[1] - 9,
                max_y >= image.shape[0] - 9,
            ])
            if border_sides > 1:
                continue
            if rectangularity >= 0.65 and 1.1 <= aspect <= 1.9:
                center = np.array(rect[0], dtype=np.float64)
                image_center = np.array([image.shape[1] * 0.5, image.shape[0] * 0.5])
                center_distance = np.linalg.norm((center - image_center) / image_center)
                score = area * rectangularity * np.exp(-1.5 * center_distance)
                candidates.append((score, rect, rectangularity, aspect))
        if candidates:
            _, rect, rectangularity, aspect = max(candidates, key=lambda item: item[0])
            box = np.int32(np.round(cv2.boxPoints(rect)))
            cv2.polylines(annotated, [box], True, (0, 255, 0), 6)
            cv2.putText(
                annotated,
                "candidate rect=%.2f aspect=%.2f" % (rectangularity, aspect),
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                3,
            )
        else:
            cv2.putText(annotated, "NO CANDIDATE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        cv2.imwrite(str(output / (path.stem + "_mask.png")), mask)
        cv2.imwrite(str(output / (path.stem + "_annotated.jpg")), annotated)
        print("wrote", path.stem)


if __name__ == "__main__":
    main()
