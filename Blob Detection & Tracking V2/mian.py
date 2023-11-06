"""
Author       : Jiawei Xu, Hanqing Qi
Date         : 2023-11-05 20:22:47
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-05 22:06:25
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/mian.py
Description  : The main program for bicoper vision control.
"""
from lib.Ibus import IBus
import time
import sensor
from lib.tracker import BLOBTracker, GoalTracker
from lib.Ibus import IBus


def blob_tracking(thresholds, clock, blob_type=1):
    """ The blob tracker initialization for balloons
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


def init_sensor_target(isColored=True, framesize=sensor.HQVGA, windowsize=None) -> None:
    sensor.reset()                        # Initialize the camera sensor.
    if isColored:
        sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
    else:
        sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(framesize)
    if windowsize is not None:            # Set windowing to reduce the resolution of the image
        sensor.set_windowing(windowsize)
    # sensor.skip_frames(time=1000)         # Let new settings take affect.
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)
#    sensor.__write_reg(0xad, 0b01001100) # R ratio
#    sensor.__write_reg(0xae, 0b01010100) # G ratio
#    sensor.__write_reg(0xaf, 0b01101000) # B ratio
    # RGB gains
    sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
    sensor.__write_reg(0x80, 0b10111100) # enable gamma, CC, edge enhancer, interpolation, de-noise
    sensor.__write_reg(0x81, 0b01101100) # enable BLK dither mode, low light Y stretch, autogray enable
    sensor.__write_reg(0x82, 0b00000100) # enable anti blur, disable AWB
    sensor.__write_reg(0x03, 0b00000000) # high bits of exposure control
    sensor.__write_reg(0x04, 0b01000000) # low bits of exposure control
    sensor.__write_reg(0xb0, 0b01100000) # global gain

    # RGB gains
    sensor.__write_reg(0xa3, 0b01110000) # G gain odd
    sensor.__write_reg(0xa4, 0b01110000) # G gain even
    sensor.__write_reg(0xa5, 0b10000000) # R gain odd
    sensor.__write_reg(0xa6, 0b10000000) # R gain even
    sensor.__write_reg(0xa7, 0b10000000) # B gain odd
    sensor.__write_reg(0xa8, 0b10000000) # B gain even
    sensor.__write_reg(0xa9, 0b10000000) # G gain odd 2
    sensor.__write_reg(0xaa, 0b10000000) # G gain even 2
    sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
    # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
                                           # strangely constrained by auto saturation
    sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
    sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
    sensor.__write_reg(0xd3, 0b01001000) # luma contrast
    # sensor.__write_reg(0xd5, 0b00000000) # luma offset
    # sensor.skip_frames(time=2000) # Let the camera adjust.


def mode_initialization(input_mode, mode):
    """ Switching between blinking goal tracker and balloon tracker
    """
    if mode == input_mode:
        print("already in the mode")
        return None
    else:
        if input_mode == 0:
            # balloon tracking mode
            init_sensor_target(isColored=True)
            thresholds = PURPLE
            tracker = blob_tracking(thresholds, clock, blob_type=1)
        elif input_mode == 1:
            init_sensor_target(isColored=False)
            # Find reference
            thresholds = GRAY
            tracker = blob_tracking(thresholds, clock, blob_type=2)

        return input_mode, tracker


if __name__ == "__main__":
    ### Macros
    GREEN = [(28, 40, -24, -4, -2, 28)]
    PURPLE = [(8, 19, 7, 18, -24, -5)]
    GRAY = [(0, 20)]
    THRESHOLD_UPDATE_RATE = 0.0
    WAIT_TIME_US = 50000
    ### End Macros

    clock = time.clock()
    mode = 1

    # Initialize inter-board communication
    ibus = IBus()
    # Sensor initialization

    mode, tracker = mode_initialization(mode, -1)

    while True:
        tracker.track()
        if tracker.tracked_blob.feature_vector:
            roi = tracker.roi.get_roi()
            feature_vec = tracker.tracked_blob.feature_vector
            x_roi = roi[0] + roi[2]//2
            y_roi = roi[1] + roi[3]//2
            w_roi = roi[2]
            h_roi = roi[3]

            x_value = int(feature_vec[0] + feature_vec[2]/2)
            y_value = int(feature_vec[1] + feature_vec[3]/2)
            w_value = int(feature_vec[2])
            h_value = int(feature_vec[3])
            ibus.send([mode, x_roi, y_roi, w_roi, h_roi, x_value, y_value, w_value, h_value])
        else:
            ibus.send([-1, 0, 0, 0, 0, 0, 0, 0, 0])

        received_msg = ibus.receive()
        if received_msg == 0x80:
            res = mode_initialization(0, mode)
            if res:
                mode, tracker = res
        elif received_msg == 0x81:
            res = mode_initialization(1, mode)
            if res:
                mode, tracker = res
