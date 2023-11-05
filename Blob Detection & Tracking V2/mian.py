"""
Author       : Hanqing Qi & Jiawei Xu & Karen Li
Date         : 2023-11-03 18:53:44
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-04 22:11:36
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/mian.py
Description  : The main python file for blob detection and tracking running on Nicla Vision
"""

import sensor, image, time
import omv
from pyb import LED
from pyb import UART
from machine import I2C
from machine import Pin
from vl53l1x import VL53L1X
import mjpeg, pyb
import random
import math


### ------------------------------------------------ Below are raw codes ------------------------------------------------ ###



class BlobTracker(Tracker):
    """BlobTracker class that initializes with a single TrackedBlob
    and tracks it with dynamic threshold
    TODO: track multiple blobs
    """

    def track(self):
        """Detect blobs with tracking capabilities
        :input: tracked_blob: a TrackedBlob class object
                thresholds: the list of color thresholds we want to track
                show: True if we want to visualize the tracked blobs
                clock: clock
        """
        # initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            reference_blob, statistics = find_reference(
                self.clock, self.original_thresholds, time_show_us=0
            )
            blue_led.on()
            self.tracked_blob.reinit(reference_blob)
            # update the adaptive threshold
            new_threshold = comp_new_threshold(statistics, 2.0)
            for i in range(len(self.current_thresholds)):
                self.current_thresholds[i] = comp_weighted_avg(
                    self.current_thresholds[i],
                    new_threshold,
                    1 - THRESHOLD_UPDATE_RATE,
                    THRESHOLD_UPDATE_RATE,
                )

            # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
            self.roi.update(
                True,
                self.tracked_blob.feature_vector[0],
                self.tracked_blob.feature_vector[1],
                self.tracked_blob.feature_vector[2],
                self.tracked_blob.feature_vector[3],
            )
            return self.tracked_blob.feature_vector, True
        else:
            # O.W. update the blob
            img = sensor.snapshot()
            self.clock.tick()
            blobs = img.find_blobs(
                self.current_thresholds,
                merge=True,
                pixels_threshold=75,
                area_threshold=100,
                margin=20,
                roi=self.roi.get_roi(),
                x_stride=1,
                y_stride=1,
            )
            blue_led.on()
            roi = self.tracked_blob.update(blobs)

            if self.tracked_blob.untracked_frames >= 15:
                # if the blob fails to track for 15 frames, reset the tracking
                red_led.off()
                green_led.on()
                self.tracked_blob.reset()
                # self.roi.reset()
                blue_led.off()
                print("boom!")
                self.current_thresholds = [
                    threshold for threshold in self.original_thresholds
                ]
                return None, False
            else:
                if roi:
                    green_led.off()
                    red_led.off()
                    self.roi.update(True, roi[0], roi[1], roi[2], roi[3])
                    statistics = img.get_statistics(roi=roi)
                    new_threshold = comp_new_threshold(statistics, 3.0)
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(
                            self.current_thresholds[i],
                            new_threshold,
                            1 - THRESHOLD_UPDATE_RATE,
                            THRESHOLD_UPDATE_RATE,
                        )
                else:
                    green_led.off()
                    red_led.on()
                    self.roi.update()
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(
                            self.original_thresholds[i], self.current_thresholds[i]
                        )
                # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
                if self.show:
                    x0, y0, w, h = [
                        math.floor(self.tracked_blob.feature_vector[i])
                        for i in range(4)
                    ]
                    img.draw_rectangle(x0, y0, w, h)
                    img.draw_rectangle(self.roi.get_roi(), color=(255, 255, 0))
                    st = "FPS: {}".format(str(round(self.clock.fps(), 2)))
                    img.draw_string(0, 0, st, color=(255, 0, 0))
                return self.tracked_blob.feature_vector, True


class GoalTracker(Tracker):
    """GoalTracker class that initializes with a single TrackedBlob and tracks it
    with dynamic threshold and ROI, the track function is specific for targets
    which involve turning LEDs on and off
    TODO: track multiple blobs
    """

    def track(self, edge_removal=True):
        """Detect blobs with tracking capabilities
        :input: tracked_blob: a TrackedBlob class object
                thresholds: the list of color thresholds we want to track
                show: True if we want to visualize the tracked blobs
                clock: clock
        """
        # initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            reference_blob, statistics = find_reference(
                self.clock, self.original_thresholds, time_show_us=0, blink=True
            )
            blue_led.on()
            self.tracked_blob.reinit(reference_blob)
            # update the adaptive threshold
            new_threshold = comp_new_threshold(statistics, 2.0)
            for i in range(len(self.current_thresholds)):
                self.current_thresholds[i] = comp_weighted_avg(
                    self.current_thresholds[i],
                    new_threshold,
                    1 - THRESHOLD_UPDATE_RATE,
                    THRESHOLD_UPDATE_RATE,
                )

            # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
            self.roi.update(
                True,
                self.tracked_blob.feature_vector[0],
                self.tracked_blob.feature_vector[1],
                self.tracked_blob.feature_vector[2],
                self.tracked_blob.feature_vector[3],
            )
            return self.tracked_blob.feature_vector, True
        else:
            # O.W. update the blob
            blue_led.on()
            img, blobs = goal_blob_detection(
                self.current_thresholds, edge_removal=edge_removal
            )
            roi = self.tracked_blob.update(blobs)

            if self.tracked_blob.untracked_frames >= 15:
                # if the blob fails to track for 15 frames, reset the tracking
                red_led.off()
                green_led.on()
                self.tracked_blob.reset()
                # self.roi.reset()
                blue_led.off()
                print("boom!")
                self.current_thresholds = [
                    threshold for threshold in self.original_thresholds
                ]
                return None, False
            else:
                if roi:
                    green_led.off()
                    red_led.off()
                    self.roi.update(True, roi[0], roi[1], roi[2], roi[3])
                    statistics = img.get_statistics(roi=roi)
                    new_threshold = comp_new_threshold(statistics, 3.0)
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(
                            self.current_thresholds[i],
                            new_threshold,
                            1 - THRESHOLD_UPDATE_RATE,
                            THRESHOLD_UPDATE_RATE,
                        )
                else:
                    green_led.off()
                    red_led.on()
                    self.roi.update()
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(
                            self.original_thresholds[i], self.current_thresholds[i]
                        )
                # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
                if self.show:
                    x0, y0, w, h = [
                        math.floor(self.tracked_blob.feature_vector[i])
                        for i in range(4)
                    ]
                    img.draw_rectangle(x0, y0, w, h, color=(255, 0, 0))
                    img.draw_rectangle(self.roi.get_roi(), color=(128, 128, 0))
                    st = "FPS: {}".format(str(round(self.clock.fps(), 2)))
                    img.draw_string(0, 0, st, color=(0, 0, 0))
                    img.flush()
                return self.tracked_blob.feature_vector, True


def hold_up_for_sensor_refresh(last_time_stamp, wait_time) -> None:
    """
    description: wait for the sensor for some time from the
                 last snapshot to avoid a partial new image
    return  {*}: None
    """
    elapsed = wait_time - (int((time.time_ns() - last_time_stamp) / 1000))
    if elapsed > 0:
        time.sleep_us(elapsed)

    return None


def goal_blob_detection(goal_thresholds, isColored=False, edge_removal=True):
    """Detecting retroreflective goals with a blinking IR LED"""
    omv.disable_fb(True)  # no show on screen

    # get an extra frame buffer and take a snapshot
    if isColored:
        extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
    else:
        extra_fb = sensor.alloc_extra_fb(
            sensor.width(), sensor.height(), sensor.GRAYSCALE
        )
    extra_fb.replace(sensor.snapshot())

    # turn on the LED
    led_pin.value(1)
    time_last_snapshot = time.time_ns()  # wait for the sensor to capture a new image
    # time block 1:
    # Do something other than wait, preferrably detection filtering and tracking
    hold_up_for_sensor_refresh(time_last_snapshot, WAIT_TIME_US)
    img = sensor.snapshot()

    # turn off the LED
    led_pin.value(0)
    time_last_snapshot = time.time_ns()

    # time block 2:
    # Do something other than wait, preferrably raw detection
    img.sub(extra_fb, reverse=False)

    # remove the edge noises
    edge_mask = None
    if edge_removal:
        if isColored:
            extra_fb.to_grayscale().find_edges(image.EDGE_SIMPLE)
        else:
            extra_fb.find_edges(image.EDGE_SIMPLE)
        edge_mask = extra_fb.dilate(3, 3).negate()

    img.negate()
    blobs = blobs = img.find_blobs(
        goal_thresholds,
        area_threshold=40,
        pixels_threshold=20,
        margin=10,
        merge=True,
        mask=edge_mask,
    )
    sensor.dealloc_extra_fb()
    omv.disable_fb(False)
    img.flush()
    hold_up_for_sensor_refresh(time_last_snapshot, WAIT_TIME_US)
    return img, blobs


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


def find_reference(
    clock,
    thresholds,
    density_threshold=0.25,
    roundness_threshold=0.35,
    time_show_us=50000,
    blink=False,
):
    """Find a reference blob that is dense and round,
    also return the color statistics in the bounding box
    """
    biggest_blob = None
    while not biggest_blob:
        blob_list = []
        clock.tick()
        if blink:
            img, blob_list = goal_blob_detection(thresholds)
        else:
            img = sensor.snapshot()
            b_blobs = img.find_blobs(
                thresholds,
                merge=True,
                pixels_threshold=30,
                area_threshold=50,
                margin=20,
                x_stride=1,
                y_stride=1,
            )
            for blob in b_blobs:
                # find a good initial blob by filtering out the not-so-dense and not-so-round blobs
                if (
                    blob.density() > density_threshold
                    and blob.roundness() > roundness_threshold
                ):
                    blob_list.append(blob)
        biggest_blob = find_max(blob_list)

    draw_initial_blob(img, biggest_blob, time_show_us)
    statistics = img.get_statistics(roi=biggest_blob.rect())
    return biggest_blob, statistics


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
    ### Macros
    GREEN = [(31, 43, -36, -14, 10, 28)]
    PURPLE = [(35, 52, -8, 10, -33, -3)]
    GRAY = [(0, 20)]
    THRESHOLD_UPDATE_RATE = 0.0
    WAIT_TIME_US = 50000
    ### End Macros

    led_pin = Pin("PG12", Pin.OUT)
    led_pin.value(0)
    red_led = pyb.LED(1)
    green_led = pyb.LED(2)
    blue_led = pyb.LED(3)
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

