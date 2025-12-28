import numpy as np
import matplotlib.pyplot as plt
import cv2

class LaneDetector: 
    """Calculates the error of the target position to the actual position of the Duckiebot"""

    def __init__ (self, image):
        self.image = image
    
    def detect(self, image):
        self.image = image
        image = cv2.imread(image)
        #Turn the image to grayscale
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #Find the center of the image itself (essentially the direction of the Duckiebot)
        #Detect all shapes in the image
        #Turn it back to color, keep the rectangles that are yellow
        #Find the closest yellow rectangle to the center of the image
        #Calculate the error from the center of the image to the closest yellow rectangle (angle?)
        #Return the error




    return error

class PIDControl: 

class LaneFollower: