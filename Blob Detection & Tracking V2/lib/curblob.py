"""
Author       : Hanqing Qi
Date         : 2023-11-04 15:07:52
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-04 16:27:19
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/lib/blob.py
Description  : 
"""

NORM_LEVEL = 2 # Default to use L2 norm, change to L1 to reduce computation

class curBLOB:
    def __init__(self, initial_blob, norm_level: int=NORM_LEVEL, feature_dist_threshold:int=100, window_size=3,blob_id=0)->None:
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
        self.untracked_frames = 0 # number of frames that the blob is not tracked
        self.feature_dist_threshold = feature_dist_threshold # threshold for feature distance
        self.window_size = window_size # window size for moving average
        self.id = blob_id # id of the blob

    def reset(self)->None:
        self.blob_history = None
        self.feature_vector = None
        self.untracked_frames = 0

    def reinit(self, blob):
        """reinitialize a reset blob by populate its history list and
        feature vector with a new blob
        """
        self.blob_history = [blob]
        self.feature_vector = [
            blob.x(),
            blob.y(),
            blob.w(),
            blob.h(),
            blob.rotation_deg(),
        ]
        self.untracked_frames = 0

    def compare(self, new_blob):
        """Compare a new blob with a tracked blob in terms of
        their feature vector distance
        """
        new_feature = (
            new_blob.x(),
            new_blob.y(),
            new_blob.w(),
            new_blob.h(),
            new_blob.rotation_deg(),
        )
        old_feature = self.feature_vector
        if not new_blob.code() == self.blob_history[-1].code(): # Check if the color is the same
            # Different colors automatically grant a maximum distance
            return 32767
        elif self.norm_level == 1: # The norm level is L1

    def update(self, blobs):
        """Update a tracked blob with a list of new blobs in terms of their feature distance.
        Upon a new candidate blob, we update the tracking history based on whether the
        histroy list is already filled or not
        """
        if blobs is None:
            # auto fail if None is fed
            self.untracked_frames += 1
            return None

        min_dist = 32767
        candidate_blob = None
        for b in blobs:
            # find the blob with minimum feature distance
            dist = self.compare(b)
            if dist < min_dist:
                min_dist = dist
                candidate_blob = b

        if min_dist < self.feature_dist_threshold:
            # update the feature history if the feature distance is below the threshold
            self.untracked_frames = 0
            # print("Successful Update! Distance: {}".format(min_dist))
            history_size = len(self.blob_history)
            self.blob_history.append(candidate_blob)
            feature = (
                candidate_blob.x(),
                candidate_blob.y(),
                candidate_blob.w(),
                candidate_blob.h(),
                candidate_blob.rotation_deg(),
            )

            if history_size < self.window_size:
                # populate the history list if the number of history blobs is below the
                # window size
                for i in range(5):
                    # calculate the moving average
                    self.feature_vector[i] = (
                        self.feature_vector[i] * history_size + feature[i]
                    ) / (history_size + 1)
            else:
                # O.W. pop the oldest and push a new one
                oldest_blob = self.blob_history[0]
                oldest_feature = (
                    oldest_blob.x(),
                    oldest_blob.y(),
                    oldest_blob.w(),
                    oldest_blob.h(),
                    oldest_blob.rotation_deg(),
                )
                for i in range(5):
                    self.feature_vector[i] = (
                        self.feature_vector[i] * self.window_size
                        + feature[i]
                        - oldest_feature[i]
                    ) / self.window_size
                self.blob_history.pop(0)
            return candidate_blob.rect()
        else:
            self.untracked_frames += 1
            return None
