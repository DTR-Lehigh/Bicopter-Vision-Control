"""
Author       : Hanqing Qi
Date         : 2023-11-04 13:32:00
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-04 14:36:38
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/lib/roi.py
Description  : The ROI(region of interest) library for the Bicopter Vision Control project.
"""

# Macros
FRAME_PARAMS = [0, 0, 240, 160] # Upper left corner x, y, width, height
FF_POSITION = 0.5 # The forgetting factor for the position
FF_SIZE = 0.5 # The forgetting factor for the size

class ROI:
    def __init__(self, frame_params:list = FRAME_PARAMS, min_windowsize:int=20, ffp:float=FF_POSITION, ffs:float=FF_SIZE)->None:
        """
        @description: Constructor of the ROI object.
        @param       {*} self: 
        @param       {list} frame_params: The parameters of the frame [x0, y0, max_w, max_h]
        @param       {int} min_windowsize: The minimum size of the tracking window
        @param       {float} ffp: The forgetting factor for the position
        @param       {float} ffs: The forgetting factor for the size
        @return      {*} None
        """
        self.roi = frame_params # [x0, y0, w, h]
        self.frame_params = frame_params  # [x0, y0, max_w, max_h]
        self.min_windowsize = min_windowsize
        self.ffp = ffp
        self.ffs = ffs

    def _clamp(self)->None:
        """
        @description: Clamp the ROI to be within the frame.
        @param       {*} self: 
        @return      {*} None
        """
        # Ensure the ROI's top-left corner is within the bounds.
        self.roi[0] = max(self.frame_params[0], self.roi[0])
        self.roi[1] = max(self.frame_params[1], self.roi[1])

        # Ensure the ROI's bottom-right corner is within the bounds.
        self.roi[2] = min(self.frame_params[2] - self.roi[0], self.roi[2])
        self.roi[3] = min(self.frame_params[3] - self.roi[1], self.roi[3])

    def _center(self, rect:list)->tuple:
        """
        @description: Calculate the center of the rectangle.
        @param       {*} self: -
        @param       {list} rect: The rectangle to be calculated [Upper left corner x, y, w, h]  
        @return      {tuple} The center of the rectangle
        """
        if len(rect) != 4:
            raise ValueError("Cannot calculate the center of the rectangle! The rectangle must be in the form of [x0, y0, w, h]")
        return (rect[0] + rect[2] / 2, rect[1] + rect[3] / 2)
    
    def _map(self, rect1:list, rect2:list)->list:
        """
        @description: Map rect1 to rect2 by the forgetting factors.
        @param       {*} self: 
        @param       {list} rect1: Rectangle to be mapped [x0, y0, w, h]
        @param       {list} rect2: Rectangle to be mapped to [x0, y0, w, h]
        @return      {list} The mapped rectangle [x0, y0, w, h]
        """
        # Get the centers of the rectangles
        cx1, cy1 = self._center(rect1) # Center x, y
        cx2, cy2 = self._center(rect2) # Center x, y
        
        # Calculate new center by shifting rect1's center towards rect2's center by alpha
        new_cx = cx1 + self.ffp * (cx2 - cx1)
        new_cy = cy1 + self.ffp * (cy2 - cy1)

        # Shift the size of rect1 towards rect2's size by beta
        new_w = rect1[2] + self.ffs * (rect2[2] - rect1[2])
        new_h = rect1[3] + self.ffs * (rect2[3] - rect1[3])
        return [new_cx - new_w / 2, new_cy - new_h / 2, new_w, new_h]


    def update(self, new_roi=None)->None:
        if new_roi is None: # No new detection is found in the maximum tracking window
            self.roi = self._map(self.roi, self.frame_params) # Map the ROI to the frame by the forgetting factors
        else:
            # Scale up the new_roi
            new_roi[0] = new_roi[0] - 0.15 * new_roi[2]
            new_roi[1] = new_roi[1] - 0.15 * new_roi[3]
            new_roi[2] = 1.3 * new_roi[2]
            new_roi[3] = 1.3 * new_roi[3]
            self.roi = self._map(self.roi, new_roi) # Map the ROI to the new detection by the forgetting factors
        self._clamp() # Clamp the ROI to be within the frame

    def reset(self)->None:
        """
        @description: Reset the ROI to the frame.
        @param       {*} self: 
        @return      {*} None
        """
        self.roi = self.frame_params

    def get_roi(self)->list:
        """
        @description: Get the ROI.
        @param       {*} self: 
        @return      {list} The ROI [x0, y0, w, h]
        """
        return [round(value) for value in self.roi]
