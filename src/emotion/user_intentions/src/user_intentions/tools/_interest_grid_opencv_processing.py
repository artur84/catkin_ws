#! /usr/bin/env python
"""
@author: Jesus Arturo Escobedo Cabello
@contact: arturoescobedo.iq@gmail.com
"""
import cv2.cv as cv
import numpy as np


# class MyTestClass():
#    def __init__(self, x_length=10, y_length=10, origin=(0, 0), resolution=1):
#        self.grid = cv.CreateMat(self.height, self.width, cv.CV_32FC1)  # @var self.grid:  Opencv matrix;The actual grid
#    

    
if __name__ == "__main__":
   
    cv.NamedWindow("mi grid", cv.CV_WINDOW_NORMAL)
    cv.NamedWindow("mi grid2", cv.CV_WINDOW_NORMAL)
    float_img = cv.CreateMat(10, 10, cv.CV_32FC1)
    float_img = cv.Load("interest_grid.xml", cv.CreateMemStorage())
    py_img = 1.0 * np.asarray(float_img)
    py_img = 255 * (py_img / np.max(py_img))
    img = cv.fromarray(py_img.astype(np.uint8))
    rgb_img = cv.CreateImage((img.cols, img.rows), 8, 4)
    """Creating RGB img"""
    img_r = cv.CloneMat(img)
    img_g = cv.CreateImage((img.cols, img.rows), 8, 1)
    img_b = cv.CreateImage((img.cols, img.rows), 8, 1)
    img_a = cv.CreateImage((img.cols, img.rows), 8, 1)
    cv.Set(img_g, 10)
    cv.Set(img_b, 100)
    cv.Set(img_a, 100)
    
    cv.Merge(img_b, img_g, img_r, img_a , rgb_img)
    """Precorner detect"""
    corners = cv.CreateMat(float_img.rows, float_img.cols, float_img.type)
    cv.PreCornerDetect(float_img, corners, 3)
   
    """Canny"""
    edges = cv.CreateImage((img.cols, img.rows), 8, 1)
    print img.rows, img.cols, edges.height
    cv.Canny(img, edges, 20.0, 160.0)
    disp2 = edges
    
    """Good features to track"""
    eig_image = cv.CreateMat(img.rows, img.cols, cv.CV_32FC1)
    temp_image = cv.CreateMat(img.rows, img.cols, cv.CV_32FC1)
    features_x_y_vector = cv.GoodFeaturesToTrack(img, eig_image, temp_image, 10, 0.002, 1.0, useHarris=True)
    disp3 = cv.CreateMat(img.rows, img.cols, cv.CV_8UC1)
    cv.Set(disp3, 0)
    for (x, y) in features_x_y_vector:
        disp3[y, x] = 255
    """Visualization"""
    tmp = 1.0 * np.asarray(corners)
    tmp = 255 * (tmp / np.max(tmp))
    disp1 = cv.fromarray(tmp.astype(np.uint8))
    
    
    cv.ShowImage("mi grid2", rgb_img)
    cv.WaitKey(1000)
    cv.ShowImage("mi grid", disp1)
    cv.WaitKey(1000)
    cv.ShowImage("mi grid", disp2)
    cv.WaitKey(1000)
    cv.ShowImage("mi grid", disp3)
    cv.WaitKey(1000000)
    cv.DestroyAllWindows()
    print "program finished"
