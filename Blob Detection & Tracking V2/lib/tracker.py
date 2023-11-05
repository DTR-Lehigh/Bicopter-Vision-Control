"""
Author       : Hanqing Qi
Date         : 2023-11-04 19:02:37
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-05 17:05:51
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/lib/tracker.py
Description  : The parent class for bolbtacker and goaltracker.
"""

from machine import Pin
import sensor, image
from pyb import LED
import curblob
import memroi
import time
import math
import omv


class Tracker:
    def __init__(
        self,
        tracked_blob: curblob.CurBLOB,
        thresholds: list,
        clock: time.clock,
        show: bool = True,
        max_untracked_frames: int = 0,
        dynamic_threshold: bool = False,
        threshold_update_rate: int = 0,
    ) -> None:
        """
        @description: Constructor of the Tracker class
        @param       {*} self:
        @param       {curblob} tracked_blob: The blob to be tracked
        @param       {list} thresholds: The list of thresholds for goal or balloon
        @param       {time} clock: The clock to track the time
        @param       {bool} show: Whether to show the image (default: True)
        @param       {int} max_untracked_frames: The maximum number of untracked frames until the tracker resets (default: 0)
        @param       {bool} dynamic_threshold: Whether to use dynamic threshold (default: False)
        @param       {int} threshold_update_rate: The rate of threshold update (default: 0)
        @return      {*} None
        """
        self.tracked_blob = tracked_blob  # The blob to be tracked
        self.original_thresholds = [threshold for threshold in thresholds]  # Deep copy the thresholds
        self.current_thresholds = [threshold for threshold in thresholds]  # Deep copy the thresholds
        self.clock = clock  # The clock to track the time
        self.show = show  # Whether to show the image
        self.roi = memroi.MemROI(ffp=0.1, ffs=0.1, gfp=0.2, gfs=0.2)  # The ROI of the blob
        self.max_untracked_frames = max_untracked_frames  # The maximum number of untracked frames
        self.dynamic_threshold = dynamic_threshold  # Whether to use dynamic threshold
        self.threshold_update_rate = threshold_update_rate  # The rate of threshold update
        self.r_LED = pyb.LED(1)  # The red LED
        self.g_LED = pyb.LED(2)  # The green LED
        self.b_LED = pyb.LED(3)  # The blue LED

    def track(self):
        # TODO: Implement this function in the child class
        pass

    def find_reference(self):
        # TODO: Implement this function in the child class
        pass

    def draw_blob(img: image, blob: image.blob, sleep_us: int = 500000) -> None:
        """
        @description:
        @param       {image} img: The image to be drawn on
        @param       {image.blob} blob: The blob to be drawn
        @param       {int} sleep_us: The time to sleep after drawing the blob (default: 500000)
        @return      {*} None
        """
        if not blob or sleep_us < 41000:
            # No need to show anything if we do not want to show
            # sleep_us is beyond human's 24fps classy eyes' capability
            return
        else:
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)
            # Sleep for 500ms for initial blob debut
            time.sleep_us(sleep_us)

    def _find_max(nice_blobs: list) -> image.blob:
        """
        @description: Find the blob with the largest area
        @param       {list} nice_blobs: The list of blobs to be compared
        @return      {image.blob} The blob with the largest area
        """
        max_blob = None
        max_area = 0
        for blob in nice_blobs:
            if blob.area() > max_area:
                max_blob = blob
                max_area = blob.pixels()
        return max_blob

    def _comp_new_threshold(statistics: image.statistics, mul_stdev: float = 3) -> tuple:
        """
        @description: Compute the new threshold based on the color statistics
        @param       {image} statistics: The color statistics of the blob
        @param       {float} mul_stdev: The multiplier of the standard deviation
        @return      {tuple} The new threshold
        WARNING: Could be deprecated
        """
        L_mean = statistics.l_mean()
        L_stdev = statistics.l_stdev()
        A_mean = statistics.a_mean()
        A_stdev = statistics.a_stdev()
        B_mean = statistics.b_mean()
        B_stdev = statistics.b_stdev()

        new_threshold = (
            L_mean - mul_stdev * L_stdev,
            L_mean + mul_stdev * L_stdev,
            A_mean - mul_stdev * A_stdev,
            A_mean + mul_stdev * A_stdev,
            B_mean - mul_stdev * B_stdev,
            B_mean + mul_stdev * B_stdev,
        )
        return new_threshold

    def _comp_weighted_avg(new_vec: tuple, orig_vec: tuple, w1: float = 0.1, w2: float = 0.9) -> tuple:
        """
        @description: Compute the weighted average of two vectors
        @param       {tuple} new_vec: The new vector
        @param       {tuple} orig_vec: The original vector to be averaged with
        @param       {float} w1: The weight of the new vector (default: 0.1)
        @param       {float} w2: The weight of the original vector (default: 0.9)
        @return      {tuple} The weighted average of the two vectors
        WARNING: Could be deprecated
        """
        weighted_avg = [int(w1 * new_vec[i] + w2 * orig_vec[i]) for i in range(len(new_vec))]
        return tuple(weighted_avg)

    def update_thresholds(
        self,
        statistics: image.statistics = None,
        recall: bool = False,
        reset: bool = False,
    ) -> None:
        """
        @description: Update the thresholds
        @param       {*} self:
        @param       {image.statistics} statistics: The color statistics of the blob
        @param       {bool} recall: If we want to recall the original threshold (default: False)
        @param       {bool} reset: If we want to reset the threshold (default: False)
        @return      {*} None
        """
        if not self.dynamic_threshold:
            return
        if recall:
            new_threshold = [threshold for threshold in self.original_thresholds]  # Deep copy the original thresholds
        else:
            new_threshold = self._comp_new_threshold(statistics) if statistics else self.original_thresholds
        # Calculate the weighted average of the new threshold and the original threshold
        for i in range(len(self.current_thresholds)):
            self.current_thresholds[i] = self._comp_weighted_avg(
                new_threshold,
                self.current_thresholds[i],
                self.threshold_update_rate if not reset else 0,
                1 - self.threshold_update_rate if not reset else 1,
            )

    def update_leds(self, tracking: bool = False, detecting: bool = False, lost: bool = True) -> None:
        """
        @description: Update the LEDs state
        @param       {*} self:
        @param       {bool} tracking: If we are tracking the blob in the roi
        @param       {bool} detecting: If we are actually detecting the blob
        @param       {bool} lost: If we lost the blob
        @return      {*} None
        """
        self.g_LED.off()
        self.r_LED.off()
        self.b_LED.off()
        if tracking and detecting and not lost:
            self.b_LED.on()
        elif tracking and not detecting and not lost:
            self.b_LED.on()
            self.r_LED.on()
        elif not tracking and not detecting and lost:
            self.g_LED.on()
        else:
            print("Error: Invalid LED state")
            pass


class BLOBTracker(Tracker):
    def __init__(
        self,
        tracked_blob: curblob.CurBLOB,
        thresholds: list,
        clock: time.clock,
        show: bool = True,
        max_untracked_frames: int = 15,
        dynamic_threshold: bool = False,
        threshold_update_rate: float = 0,
    ) -> None:
        """
        @description: Constructor of the BLOBTracker class
        @param       {*} self:
        @param       {curblob} tracked_blob: The blob to be tracked
        @param       {list} thresholds: The list of thresholds (green, purple)
        @param       {time} clock: The clock to track the time
        @param       {bool} show: Whether to show the image (default: True)
        @param       {int} max_untracked_frames: The maximum number of untracked frames until the tracker resets (default: 15)
        @param       {bool} dynamic_threshold: Whether to use dynamic threshold (default: False)
        @param       {float} threshold_update_rate: The rate of threshold update (default: 0)
        @return      {*} None
        """
        super().__init__(
            tracked_blob,
            thresholds,
            clock,
            show,
            max_untracked_frames,
            dynamic_threshold,
            threshold_update_rate,
        )  # Initialize the parent class

    def track(self):
        """
        @description: Track the blob with dynamic threshold and ROI
        @return      {tuple} The feature vector of the tracked blob and whether the blob is tracked
        """
        self.update_leds(tracking=True, detecting=True, lost=False)  # Set the LEDs to indicate tracking
        # Initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            # There is no blob history, initialize the blob
            reference_blob, statistics = self.find_reference(
                self.clock, self.original_thresholds, time_show_us=0
            )  # Find the blob with the largest area
            self.tracked_blob.reinit(reference_blob)  # Initialize the tracked blob with the reference blob
            self.update_thresholds(statistics)  # Update the dynamic threshold
            self.roi.update(self.tracked_blob.feature_vector[0:4])  # Update the ROI
            return (
                self.tracked_blob.feature_vector,
                True,
            )  # Return the feature vector and True
        # Track the blob
        self.clock.tick()
        img = sensor.snapshot()
        list_of_blobs = img.find_blobs(
            self.current_thresholds,
            merge=True,
            pixels_threshold=75,
            area_threshold=100,
            margin=20,
            roi=self.roi.get_roi(),
            x_stride=1,
            y_stride=1,
        )
        blob_rect = self.tracked_blob.update(list_of_blobs)

        if self.tracked_blob.untracked_frames >= self.max_untracked_frames:
            # If the blob fails to track for 15 frames, reset the tracking and find a new reference blob
            # self.roi.reset() (NOTE: ROI is not reset since we are assuming that the blob tends to appear in the same region when it is lost)
            self.tracked_blob.reset()
            self.update_leds(tracking=False, detecting=False, lost=True)
            print("Blob lost")
            self.update_thresholds(reset=True)  # Reset the dynamic threshold
            return None, False
        if blob_rect:
            # If we discover the reference blob again
            self.roi.update(blob_rect)  # Update the ROI
            statistics = img.get_statistics(roi=blob_rect)
            self.update_thresholds(statistics)  # Update the dynamic threshold
        else:
            # If we do not discover the reference blob
            self.update_leds(
                tracking=True, detecting=False, lost=False
            )  # Set the LEDs to indicate tracking but not detecting
            self.roi.update()  # Reset the ROI
            self.update_thresholds(recall=True)  # Recall the original threshold

        if self.show:
            x0, y0, w, h = [math.floor(self.tracked_blob.feature_vector[i]) for i in range(4)]
            img.draw_rectangle(x0, y0, w, h)
            img.draw_rectangle(self.roi.get_roi(), color=(255, 255, 0))
            st = "FPS: {}".format(str(round(self.clock.fps(), 2)))
            img.draw_string(0, 0, st, color=(255, 0, 0))
        return self.tracked_blob.feature_vector, True

    def find_reference(
        self,
        density_threshold: float = 0.25,
        roundness_threshold: float = 0.35,
        time_show_us: int = 50000,
    ) -> tuple:
        """
        @description: Find the a good blob to be the reference blob
        @param       {*} self:
        @param       {float} density_threshold: The density threshold of the blob
        @param       {float} roundness_threshold: The roundness threshold of the blob
        @param       {int} time_show_us: The time to show the blob on the screen
        @return      {tuple} The reference blob and its color statistics
        """
        while True:
            nice_blobs = []  # A list of good blobs
            self.clock.tick()
            img = sensor.snapshot()
            list_of_blob = img.find_blobs(
                self.original_thresholds,
                merge=True,
                pixels_threshold=30,
                area_threshold=50,
                margin=20,
                x_stride=1,
                y_stride=1,
            )
            for blob in list_of_blob:
                # Find a set of good initial blobs by filtering out the not-so-dense and not-so-round blobs
                if blob.density() > density_threshold and blob.roundness() > roundness_threshold:
                    nice_blobs.append(blob)
            if nice_blobs:  # If we find a good blob, break the loop
                break
        best_blob = self._find_max(nice_blobs)  # Find the best blob
        self.draw_initial_blob(img, best_blob, time_show_us)  # Draw the initial blob
        statistics = img.get_statistics(roi=best_blob.rect())  # Get the color statistics of the blob in actual image
        return best_blob, statistics


class GoalTracker(Tracker):
    def __init__(
        self,
        tracked_blob: curblob.CurBLOB,
        thresholds: list,
        clock: time.clock,
        show: bool = True,
        max_untracked_frames: int = 5,
        dynamic_threshold: bool = False,
        threshold_update_rate: float = 0,
        LEDpin: str = "PG12",
        sensor_sleep_time: int = 50000,
    ) -> None:
        """
        @description:
        @param       {*} self:
        @param       {curblob} tracked_blob: The blob to be tracked
        @param       {list} thresholds: The list of thresholds for the goal
        @param       {time} clock: The clock to track the time
        @param       {bool} show: Whether to show the image (default: True)
        @param       {int} max_untracked_frames: The maximum number of untracked frames until the tracker resets (default: 5)
        @param       {bool} dynamic_threshold: Whether to use dynamic threshold (default: False)
        @param       {float} threshold_update_rate: The rate of threshold update (default: 0)
        @param       {str} LEDpin: The pin of the IR LED (default: "PG12")
        @param       {int} sensor_sleep_time: The time to sleep after the sensor captures a new image (default: 50000)
        @return      {*}
        """
        super().__init__(
            tracked_blob,
            thresholds,
            clock,
            show,
            max_untracked_frames,
            dynamic_threshold,
            threshold_update_rate,
        )
        self.IR_LED = Pin(LEDpin, Pin.OUT)
        self.IR_LED.value(0)
        self.sensor_sleep_time = sensor_sleep_time

    def track(self, edge_removal: bool = True) -> tuple:
        """
        @description: Track the blob with dynamic threshold and ROI
        @param       {*} self:
        @param       {bool} edge_removal: Whether to remove the edge noises (default: True)
        @return      {tuple} The feature vector of the tracked blob and whether the blob is tracked
        """
        self.update_leds(tracking=True, detecting=True, lost=False)  # Set the LEDs to indicate tracking
        # Initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            reference_blob, statistics = self.find_reference(time_show_us=0)  # Find the blob with the largest area
            self.tracked_blob.reinit(reference_blob)  # Initialize the tracked blob with the reference blob
            self.update_thresholds(statistics)  # Update the dynamic threshold
            self.roi.update(self.tracked_blob.feature_vector[0:4])  # Update the ROI
            return self.tracked_blob.feature_vector, True
        # Track the blob
        img, list_of_blobs = self.detect(isColored=True, edge_removal=edge_removal)
        blob_rect = self.tracked_blob.update(list_of_blobs)

        if self.tracked_blob.untracked_frames >= self.max_untracked_frames:
            # If the blob fails to track for 15 frames, reset the tracking and find a new reference blob
            self.update_leds(tracking=False, detecting=False, lost=True)
            self.tracked_blob.reset()
            # self.roi.reset() (NOTE: ROI is not reset since we are assuming that the blob tends to appear in the same region when it is lost)
            print("Goal lost")
            self.update_thresholds(reset=True)
            return None, False
        if blob_rect:
            # If we discover the reference blob again
            self.roi.update(blob_rect)
            statistics = img.get_statistics(roi=blob_rect)
            self.update_thresholds(statistics)
        else:
            # If we do not discover the reference blob
            self.update_leds(tracking=True, detecting=False, lost=False)
            self.roi.update()
            self.update_thresholds(recall=True)
        if self.show:
            x0, y0, w, h = [math.floor(self.tracked_blob.feature_vector[i]) for i in range(4)]
            img.draw_rectangle(x0, y0, w, h, color=(255, 0, 0))
            img.draw_rectangle(self.roi.get_roi(), color=(128, 128, 0))
            st = "FPS: {}".format(str(round(self.clock.fps(), 2)))
            img.draw_string(0, 0, st, color=(0, 0, 0))
            img.flush()
        return self.tracked_blob.feature_vector, True

    def find_reference(
        self,
        time_show_us: int = 50000,
    ) -> tuple:
        """
        @description: Find the a good blob to be the reference blob
        @param       {*} self:
        @param       {int} time_show_us: The time to show the blob on the screen
        @return      {tuple} The reference blob and its color statistics
        """
        while True:
            self.clock.tick()
            img, nice_blobs = self.detect(isColored=True, edge_removal=False)
            if nice_blobs:
                break
        best_blob = self._find_max(nice_blobs)  # Find the best blob, will never return None if nice_blobs is not empty
        self.draw_initial_blob(img, best_blob, time_show_us)  # Draw the initial blob
        statistics = img.get_statistics(roi=best_blob.rect())  # Get the color statistics of the blob in actual image
        return best_blob, statistics

    def detect(self, isColored=False, edge_removal=True):
        omv.disable_fb(True)  # No show on screen
        # Get an extra frame buffer and take a snapshot
        if isColored:
            extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
        else:
            extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
        extra_fb.replace(sensor.snapshot())

        # Turn on the Infrared LED
        self.IR_LED.value(1)
        time_last_snapshot = time.time_ns()  # wait for the sensor to capture a new image
        # Time block 1:
        # Do something other than wait, preferrably detection filtering and tracking
        self.sensor_sleep(time_last_snapshot)
        img = sensor.snapshot()

        # Turn off the Infrared LED
        self.IR_LED.value(0)
        time_last_snapshot = time.time_ns()

        # time block 2:
        # Do something other than wait, preferrably raw detection
        img.sub(extra_fb, reverse=False)

        # Remove the edge noises
        edge_mask = None
        if edge_removal:
            if isColored:
                extra_fb.to_grayscale().find_edges(image.EDGE_SIMPLE)
            else:
                extra_fb.find_edges(image.EDGE_SIMPLE)
            edge_mask = extra_fb.dilate(3, 3).negate()

        img.negate()
        list_of_blob = list_of_blob = img.find_blobs(
            self.current_thresholds,
            area_threshold=40,
            pixels_threshold=20,
            margin=10,
            merge=True,
            mask=edge_mask,
        )
        sensor.dealloc_extra_fb()
        omv.disable_fb(False)
        img.flush()
        self.sensor_sleep(time_last_snapshot)
        return img, list_of_blob

    def sensor_sleep(self, last_time_stamp) -> None:
        """
        @description: Wait for the sensor for some time from the last snapshot to avoid a partial new image
        @param       {*} last_time_stamp: The time stamp of the last snapshot
        @return      {*} None
        """
        elapsed = self.sensor_sleep_time - (int((time.time_ns() - last_time_stamp) / 1000))
        if elapsed > 0:
            time.sleep_us(elapsed)
        return None
