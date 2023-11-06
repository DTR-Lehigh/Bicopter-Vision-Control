"""
Author       : Hanqing Qi & Jiawei Xu & Karen Li
Date         : 2023-11-03 18:53:44
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-05 17:10:24
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/main.py
Description  : The main python file for blob detection and tracking running on Nicla Vision
"""

import sensor, image, time
import omv
from pyb import LED
from machine import Pin
import mjpeg, pyb
import random
import math

# Macros
GREEN = [(31, 43, -36, -14, 10, 28)]
PURPLE = [(35, 52, -8, 10, -33, -3)]
GRAY = [(0, 20)]
DYNAMIC_THRESHOLD = False
THRESHOLD_UPDATE_RATE = 0.0
WAIT_TIME_US = 50000

### ------------------------------------------------ Below are raw codes ------------------------------------------------ ###



def blob_tracking(
    reference_blob,
    thresholds,
    clock,
    blob_type=1,
    norm_level=1,
    feature_dist_threshold=200,
):
    """The blob tracker initialization for balloons
    blob_type=1 for balloons
    blob_type=2 for goals
    """
    tracked_blob = TrackedBlob(
        reference_blob,
        norm_level=norm_level,
        feature_dist_threshold=feature_dist_threshold,
    )
    if blob_type == 1:
        blob_tracker = BlobTracker(tracked_blob, thresholds, clock)
    elif blob_type == 2:
        blob_tracker = GoalTracker(tracked_blob, thresholds, clock)
    else:
        exit(1)
    return blob_tracker


def init_sensor_target(isColored=True, framesize=sensor.HQVGA, windowsize=None) -> None:
    sensor.reset()  # Initialize the camera sensor.
    if isColored:
        sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
    else:
        sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(framesize)
    if windowsize is not None:  # Set windowing to reduce the resolution of the image
        sensor.set_windowing(windowsize)
    # sensor.skip_frames(time=1000)         # Let new settings take affect.
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)
    #    sensor.__write_reg(0xad, 0b01001100) # R ratio
    #    sensor.__write_reg(0xae, 0b01010100) # G ratio
    #    sensor.__write_reg(0xaf, 0b01101000) # B ratio
    # RGB gains
    sensor.__write_reg(0xFE, 0b00000000)  # change to registers at page 0
    sensor.__write_reg(
        0x80, 0b10111100
    )  # enable gamma, CC, edge enhancer, interpolation, de-noise
    sensor.__write_reg(
        0x81, 0b01101100
    )  # enable BLK dither mode, low light Y stretch, autogray enable
    sensor.__write_reg(0x82, 0b00000100)  # enable anti blur, disable AWB
    sensor.__write_reg(0x03, 0b00000010)  # high bits of exposure control
    sensor.__write_reg(0x04, 0b11110000)  # low bits of exposure control
    sensor.__write_reg(0xB0, 0b11100000)  # global gain

    # RGB gains
    sensor.__write_reg(0xA3, 0b01110000)  # G gain odd
    sensor.__write_reg(0xA4, 0b01110000)  # G gain even
    sensor.__write_reg(0xA5, 0b10000000)  # R gain odd
    sensor.__write_reg(0xA6, 0b10000000)  # R gain even
    sensor.__write_reg(0xA7, 0b10000000)  # B gain odd
    sensor.__write_reg(0xA8, 0b10000000)  # B gain even
    sensor.__write_reg(0xA9, 0b10000000)  # G gain odd 2
    sensor.__write_reg(0xAA, 0b10000000)  # G gain even 2
    sensor.__write_reg(0xFE, 0b00000010)  # change to registers at page 2
    # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
    # strangely constrained by auto saturation
    sensor.__write_reg(0xD1, 0b01000000)  # change Cb saturation
    sensor.__write_reg(0xD2, 0b01000000)  # change Cr saturation
    sensor.__write_reg(0xD3, 0b01001000)  # luma contrast
    # sensor.__write_reg(0xd5, 0b00000000) # luma offset
    # sensor.skip_frames(time=2000) # Let the camera adjust.




def mode_initialization(input_mode, mode):
    """Switching between blinking goal tracker and balloon tracker"""
    if mode == input_mode:
        print("already in the mode")
        return None
    else:
        if input_mode == 0:
            # balloon tracking mode
            init_sensor_target(isColored=True)
            thresholds = GREEN
            reference_blob, statistics = find_reference(clock, thresholds, blink=False)
            tracker = blob_tracking(
                reference_blob,
                thresholds,
                clock,
                blob_type=1,
                feature_dist_threshold=300,
            )
        elif input_mode == 1:
            init_sensor_target(isColored=False)
            # Find reference
            thresholds = GRAY
            reference_blob, statistics = find_reference(
                clock, thresholds, roundness_threshold=0.55, blink=True
            )
            tracker = blob_tracking(
                reference_blob,
                thresholds,
                clock,
                blob_type=2,
                feature_dist_threshold=200,
            )

        return input_mode, tracker


if __name__ == "__main__":
    clock = time.clock()

    mode = 0
    # Sensor initialization

    mode, tracker = mode_initialization(mode, -1)

    while True:
        print(mode)
        tracker.track()
        if tracker.tracked_blob.feature_vector:
            roi = tracker.roi.get_roi()
            feature_vec = tracker.tracked_blob.feature_vector
            x_value = roi[0] + roi[2] // 2
            y_value = roi[1] + roi[3] // 2
            w_value = int(feature_vec[2])
            h_value = int(feature_vec[3])

