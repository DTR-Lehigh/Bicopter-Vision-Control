"""
Author       : Hanqing Qi
Date         : 2023-11-04 15:07:52
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-04 18:15:20
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/lib/curBlob.py
Description  : 
"""

import image
import math

# Macros
NORM_LEVEL = 2  # Default to use L2 norm, change to L1 to reduce computation
MAX_FEATURE_DIST = 32767  # The maximum feature distance


class curBLOB:
    def __init__(
        self,
        initial_blob,
        norm_level: int = NORM_LEVEL,
        feature_dist_threshold: int = 100,
        window_size=3,
        blob_id=0,
    ) -> None:
        """
        @description: Constructor of the blob object that memorizes previous states.
        @param       {*} self:
        @param       {*} initial_blob: The first blob appeared after the reset
        @param       {int} norm_level: The norm level for the feature distance (default to L2)
        @param       {int} feature_dist_threshold: The threshold for the feature distance (default to 100)
        @param       {*} window_size: The window size for the moving average (default to 3)
        @param       {*} blob_id: The id of the blob
        @return      {*} None
        """
        self.blob_history = [initial_blob]
        self.feature_vector = [
            initial_blob.x(),
            initial_blob.y(),
            initial_blob.w(),
            initial_blob.h(),
            initial_blob.rotation_deg(),
        ]
        self.norm_level = norm_level
        self.untracked_frames = 0  # number of frames that the blob is not tracked
        self.feature_dist_threshold = (
            feature_dist_threshold  # threshold for feature distance
        )
        self.window_size = window_size  # window size for moving average
        self.id = blob_id  # id of the blob

    def reset(self) -> None:
        """
        @description: Reset the current blob
        @param       {*} self:
        @return      {*} None
        """
        self.blob_history = None
        self.feature_vector = None
        self.untracked_frames = 0

    def reinit(self, blob: image.blob) -> None:
        """
        @description: Reinitialize the current blob with a new blob
        @param       {*} self:
        @param       {image.blob} blob: The new blob to be reinitialized with
        @return      {*} None
        """
        self.blob_history = [blob]  # reset the blob history
        self.feature_vector = [
            blob.x(),
            blob.y(),
            blob.w(),
            blob.h(),
            blob.rotation_deg(),
        ]
        self.untracked_frames = 0  # reset the untracked frames

    def compare(self, new_blob: image.blob) -> int:
        """
        @description: Compare the feature distance between the current blob and a new blob
        @param       {*} self:
        @param       {image.blob} new_blob: The new blob to be compared with
        @return      {int} The feature distance between the current blob and the new blob
        """
        new_feature = (
            new_blob.x(),
            new_blob.y(),
            new_blob.w(),
            new_blob.h(),
            new_blob.rotation_deg(),
        )  # get the feature vector of the new blob
        old_feature = self.feature_vector  # get the feature vector of the current blob
        if (
            not new_blob.code() == self.blob_history[-1].code()
        ):  # Check if the color is the same
            return MAX_FEATURE_DIST  # Different colors automatically grant a maximum distance
        elif self.norm_level == 1:  # The norm level is L1
            return sum([abs(new_feature[i] - old_feature[i]) for i in range(5)])
        elif self.norm_level == 2:  # The norm level is L2
            return math.sqrt(
                sum([(new_feature[i] - old_feature[i]) ** 2 for i in range(5)]) ** 0.5
            )

    def update(self, list_of_blob: list) -> list:
        """
        @description: Update the current blob with the best candidate blob in the list of blobs
        @param       {*} self:
        @param       {list} list_of_blob: The list of blobs to be compared with
        @return      {list} The rectangle of the best candidate blob
        """
        if list_of_blob is None:  # For the case that no blob is detected
            self.untracked_frames += 1
            return None

        min_dist = 32767
        candidate_blob = None
        # Find the blob with minimum feature distance
        for b in list_of_blob:  # This should reference the input parameter 'list_of_blob', not 'blobs'
            dist = self.compare(b)
            if dist < min_dist:
                min_dist = dist
                candidate_blob = b

        if min_dist < self.feature_dist_threshold:
            # Update the feature history if the feature distance is below the threshold
            self.untracked_frames = 0  # Reset the number of untracked frames
            history_size = len(
                self.blob_history
            )  # Get the number of blobs in the history
            self.blob_history.append(candidate_blob)
            # Calculate the feature vector of the candidate blob
            candidate_feature = (
                candidate_blob.x(),
                candidate_blob.y(),
                candidate_blob.w(),
                candidate_blob.h(),
                candidate_blob.rotation_deg(),
            )

            if history_size < self.window_size:
                # Calculate the moving average directly if the blob history is not filled
                self.feature_vector = [
                    (current + new) / (history_size + 1)
                    for current, new in zip(self.feature_vector, candidate_feature)
                ]
            else:
                # Remove the oldest blob from the history and calculate the moving average
                oldest_blob = self.blob_history[0]
                oldest_feature = (
                    oldest_blob.x(),
                    oldest_blob.y(),
                    oldest_blob.w(),
                    oldest_blob.h(),
                    oldest_blob.rotation_deg(),
                )
                self.feature_vector = [
                    (current * self.window_size - old + new) / history_size
                    for current, old, new in zip(
                        self.feature_vector, oldest_feature, candidate_feature
                    )
                ]
            return candidate_blob.rect()
        else:
            # Increase the number of untracked frames if no good candidate is found
            self.untracked_frames += 1
            return None
