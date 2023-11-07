"""
Author       : Jiawei Xu, Hanqing Qi, Karen Li
Date         : 2023-11-05 20:22:47
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-07 16:40:21
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/main.py
Description  : The main program for bicoper vision control.
"""

from lib.tracker import BLOBTracker, GoalTracker
from lib.Ibus import IBus
import sensor
import time

# Macros
## Color thresholds
GREEN = [(28, 40, -24, -4, -2, 28)]  # Color threshold for green balloons
PURPLE = [(8, 19, 7, 18, -24, -5)]  # Color threshold for purple balloons
GRAY = [(0, 20)]  # Color threshold for goals with blinking method
BALLON = PURPLE  # The current color threshold for ballon detection
# BALLON = GREEN + PURPLE # For both green and purple balloons

## Tracker Tresholds
SHOW = True  # Whether to show the blob
MAX_UNTRACKED_FRAMES_BALLOON = 15  # Maximum number of frames to be untracked before the tracker is reset
FEATURE_DISTANCE_THRESHOLD_BALLOON = 200  # Maximum distance between two features to be considered the same feature
MAX_UNTRACKED_FRAMES_GOAL = 5  # Maximum number of frames to be
FEATURE_DISTANCE_THRESHOLD_GOAL = 200  # Maximum distance between two features to be considered the same feature
FACTORS_BALLON = [0.1, 0.1, 0.1, 0.1]
FACTORS_GOAL = [0.1, 0.1, 0.1, 0.1]


# Functions
def init_sensor(isColored: bool = True, framesize=sensor.HQVGA, windowsize=None) -> None:
    """
    @description: Initialize the sensor for detection
    @param       {bool} isColored: Whether the sensor is colored (default to True)
    @param       {*} framesize: The size of the frame (default to sensor.HQVGA)
    @param       {*} windowsize: The size of the window (default to None)
    @return      {*} None
    """
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
    # sensor.__write_reg(0xad, 0b01001100) # R ratio
    # sensor.__write_reg(0xae, 0b01010100) # G ratio
    # sensor.__write_reg(0xaf, 0b01101000) # B ratio
    # RGB gains
    sensor.__write_reg(0xFE, 0b00000000)  # change to registers at page 0
    sensor.__write_reg(0x80, 0b10111100)  # enable gamma, CC, edge enhancer, interpolation, de-noise
    sensor.__write_reg(0x81, 0b01101100)  # enable BLK dither mode, low light Y stretch, autogray enable
    sensor.__write_reg(0x82, 0b00000100)  # enable anti blur, disable AWB
    sensor.__write_reg(0x03, 0b00000000)  # high bits of exposure control
    sensor.__write_reg(0x04, 0b01000000)  # low bits of exposure control
    sensor.__write_reg(0xB0, 0b01100000)  # global gain

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
    # sensor.__write_reg(0xd0, 0b00000000) # change global saturation, strangely constrained by auto saturation
    sensor.__write_reg(0xD1, 0b01000000)  # change Cb saturation
    sensor.__write_reg(0xD2, 0b01000000)  # change Cr saturation
    sensor.__write_reg(0xD3, 0b01001000)  # luma contrast
    # sensor.__write_reg(0xd5, 0b00000000) # luma offset
    # sensor.skip_frames(time=2000) # Let the camera adjust.


def set_mode(current_mode: str, desired_mode: str, mytracker=None) -> tuple:
    """
    @description: Set the mode of the detection
    @param       {str} current_mode: The current mode of the detection
    @param       {str} desired_mode: The desired mode of the detection
    @param       {*} mytracker: The tracker object (default to None)
    @return      {tuple} The current mode and the tracker
    """

    def change_mode(mode):
        init_sensor(isColored=(mode == "B"))
        thresholds = BALLON if mode == "B" else GRAY

        # Initialize the tracker
        if mode == "B":
            blob_tracker = BLOBTracker(
                thresholds,
                myclock,
                show=SHOW,
                max_untracked_frames=MAX_UNTRACKED_FRAMES_BALLOON,
                feature_distance_threshold=FEATURE_DISTANCE_THRESHOLD_BALLOON,
                factors=FACTORS_BALLON,
            )
        elif mode == "G":
            blob_tracker = GoalTracker(
                thresholds,
                myclock,
                show=SHOW,
                max_untracked_frames=MAX_UNTRACKED_FRAMES_GOAL,
                feature_distance_threshold=FEATURE_DISTANCE_THRESHOLD_GOAL,
                factors=FACTORS_GOAL,
            )
        else:
            raise ValueError("Invalid blob type!")
        return blob_tracker

    # Check if the mode is valid
    if desired_mode not in ["B", "G"]:
        print("Invalid mode! Defaulting to ballon mode 'B'.")
        desired_mode = "B"

    # If no tracker or a mode change is required, update the tracker
    if not mytracker or current_mode != desired_mode:
        mytracker = change_mode(desired_mode)
        print(f"Switched to {'ballon' if desired_mode == 'B' else 'goal'} tracking mode.")
    else:
        print("Already in the desired mode, no change required.")

    # Return the (possibly updated) mode and tracker
    return desired_mode, mytracker


if __name__ == "__main__":
    myclock = time.clock()  # Create a clock object to track the FPS
    detection_mode = "B"  # Default to ballon mode
    myibus = IBus()  # Initialize inter-board communication
    detection_mode, mytracker = set_mode(None, detection_mode)  # Initialize the tracker

    while True:
        mytracker.track()
        if mytracker.tracked_blob.feature_vector:
            roi = mytracker.roi.get_roi()
            blob = mytracker.tracked_blob.feature_vector
            x_roi = round(roi[0] + roi[2] / 2)
            y_roi = round(roi[1] + roi[3] / 2)
            w_roi = round(roi[2])
            h_roi = round(roi[3])

            x_blob = round(blob[0] + blob[2] / 2)
            y_blob = round(blob[1] + blob[3] / 2)
            w_blob = round(blob[2])
            h_blob = round(blob[3])
            flag = 0 if detection_mode == "B" else 1
            myibus.send([flag, x_roi, y_roi, w_roi, h_roi, x_blob, y_blob, w_blob, h_blob])
        else:
            myibus.send([-1, 0, 0, 0, 0, 0, 0, 0, 0])
        received_mode = myibus.receive()
        detection_mode, mytracker = set_mode(detection_mode, received_mode, mytracker)
