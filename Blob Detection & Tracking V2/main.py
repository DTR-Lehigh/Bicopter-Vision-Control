"""
Author       : Jiawei Xu, Hanqing Qi
Date         : 2023-11-05 20:22:47
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-05 22:35:11
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/main.py
Description  : The main program for bicoper vision control.
"""

from lib.tracker import BLOBTracker, GoalTracker
from lib.Ibus import IBus
import sensor
import time

# Macros
GREEN = [(28, 40, -24, -4, -2, 28)]
PURPLE = [(8, 19, 7, 18, -24, -5)]
GRAY = [(0, 20)]

def blob_tracking(thresholds, clock, blob_type=1):
    """The blob tracker initialization for balloons
    blob_type=1 for balloons
    blob_type=2 for goals
    """
    if blob_type == 1:
        blob_tracker = BLOBTracker(thresholds, clock)
    elif blob_type == 2:
        blob_tracker = GoalTracker(thresholds, clock)
    else:
        raise ValueError("Invalid blob type!")
    return blob_tracker


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


def set_mode(current_mode:str, desired_mode:str)->tuple:
    """
    @description: Set the mode of the detection
    @param       {str} current_mode: The current mode of the detection
    @param       {str} desired_mode: The desired mode of the detection
    @return      {tuple} The current mode and the tracker
    """
    if current_mode == desired_mode:
        print("Already in the desired mode!")
        return current_mode, None
    else:
        if desired_mode == "T":
            # Target tracking mode
            init_sensor(isColored=True) # Initialize the sensor
            thresholds = PURPLE # Set the thresholds
            mytracker = blob_tracking(thresholds, myclock, blob_type=1)
        elif desired_mode == "G":
            init_sensor(isColored=False) # Initialize the sensor
            thresholds = GRAY # Set the thresholds
            mytracker = blob_tracking(thresholds, myclock, blob_type=2)
        else:
            print("Invalid mode!")
            return current_mode, None
        return desired_mode, mytracker


if __name__ == "__main__":
    myclock = time.clock() # Create a clock object to track the FPS
    detection_mode = "T" # Default to target mode
    myibus = IBus() # Initialize inter-board communication
    detection_mode, mytracker = set_mode(detection_mode, detection_mode) # Set the mode

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
            flag = 0 if detection_mode == "T" else 1
            myibus.send([flag, x_roi, y_roi, w_roi, h_roi, x_blob, y_blob, w_blob, h_blob])
        else:
            myibus.send([-1, 0, 0, 0, 0, 0, 0, 0, 0])
        received_mode = myibus.receive()
        if received_mode != detection_mode:
            detection_mode, mytracker = set_mode(detection_mode, received_mode)
