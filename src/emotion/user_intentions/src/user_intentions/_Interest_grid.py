#! /usr/bin/env python
"""
Created on Fri Feb 15 11:28:13 2013
Bressenham algorithm
@author: Jesus Arturo Escobedo Cabello
@contact: arturoescobedo.iq@gmail.com
"""
import cv2.cv as cv
import numpy as np
from copy import deepcopy



class InterestGrid():
    """
        Data structure to store the 3D histogram of most viewed places
        when the user is driving the wheelchair.
        ROS Independent
    """
    def __init__(self, x_length=10, y_length=10, origin=(0, 0), resolution=1):
        """ Initialize our interest grid.

            @param y_length:  int, y_length of the grid [m]
            @param x_length:  int, Columns of the grid [m]
            @param origin: tuple (x,y), where it is going to be located the lower-left corner of the image with respect to the grid.
            @param resolution: float, resolution of the grid in [meters/cells]
        """
        """ MEMBERS """
        self.height = np.int(y_length / resolution)  # @var self.height:  y_length of the grid [cells]
        self.width = np.int(x_length / resolution)  # @var self.width:  x_length of the grid [cells]
        self.origin = origin  # @var self.origin: tuple (x,y) where is it going to be located the upper/left corner of the image with respect to the grid.
        self.resolution = resolution  # @var self.resolution: resolution of the grid in meters/pixel
        self.grid = cv.CreateMat(self.height, self.width, cv.CV_32FC1)  # @var self.grid:  Opencv matrix;The actual grid
        self.__temp_grid__ = cv.CreateMat(self.height, self.width, cv.CV_32FC1)  # @var self.__temp_grid__:  Opencv matrix: A temporal array to do some calculations
        self.typical_dest_list = []  # [meters] This member contains a list of tuples [(x1,y1),(x2,y2) ...] with the locations of interest points in the /map reference frame
        """ INIT """
        cv.Set(self.grid, 0)
        cv.Set(self.__temp_grid__, 0)
        print "An instance of InterestGrid has been created"

    def set_scaled_line(self, x0_map, y0_map, yaw, pitch, value=1):
        """ Uses the pitch as an scale factor to set the length of the line.
            @param x0_map: float, starting position of the line in the map's frame [meters]
            @param y0_map: float, starting position of the line in the map's frame [meters]
            @param yaw: float, angle of the user's head [rads]
            @param pitch: float,  vertical angle of the user's head [rads]
        """
        k = 1  # Scale factor chosen so that the line will have a length of almost 15 meters when the user is looking parallel to the map
        dmap = k * (0.8 - pitch)
        print "scaled distance: ", dmap
        """ Set line boundaries out from the wheelchair area """
        radius = 0.0

        xnew = x0_map + radius * np.cos(yaw)
        ynew = y0_map + radius * np.sin(yaw)
        (x0new, y0new) = self.tfmap2ocv(xnew, ynew)
        # print "( x0, xnew ):= ", (x0_map, xnew)
        # print "( y0, ynew ):= ", (y0_map, ynew)
        (x0ocv, y0ocv) = self.tfmap2ocv(x0_map, y0_map)
        dpix = np.int(dmap / self.resolution)
        (x1ocv, y1ocv) = self.compute_final_point(x0ocv, y0ocv, yaw, dpix)
        """Transformations from grid's reference frame to opencv reference frame
        """
        self.bresenham_line(x0new, y0new, x1ocv, y1ocv, value)

    def set_line(self, x0_map, y0_map, yaw, dmap=15, value=1):
        """ Adds a value to all the elements of the grid that are in the line defined by (x0,y0) and (x1,y1)
            @param x0_map: float, starting position of the line in the map's frame [meters]
            @param y0_map: float, starting position of the line in the map's frame [meters]
            @param yaw: float, angle of the user's head [rads]
            @param dmap: float, Length of the line segment [meters]
            @param value: int, How much to increment the value of each cell.
        """
        (x0ocv, y0ocv) = self.tfmap2ocv(x0_map, y0_map)
        dpix = np.int(dmap / self.resolution)
        (x1ocv, y1ocv) = self.compute_final_point(x0ocv, y0ocv, yaw, dpix)
        """Transformations from grid's reference frame to opencv reference frame"""
        self.bresenham_line(x0ocv, y0ocv, x1ocv, y1ocv, value)

    def set_point(self, xmap, ymap, value=1):
        """ Adds this value to (xmap, ymap) cell
            @param xmap: float, point position in the map's frame [meters]
            @param ymap: float, point  position in the map's frame [meters]
            @param value: int, How much to increment the value of this cell
        """
        (xocv, yocv) = self.tfmap2ocv(xmap, ymap)
        self.grid[yocv, xocv] = self.grid[yocv, xocv] + value

    def tfmap2ocv(self, xmap, ymap):
        """ Transforms from map's reference frame to opencv reference frame
            @param xmap: float, starting position of the line in the map's frame [meters]
            @param ymap: float, starting position of the line in the map's frame [meters]
        """
        (xgrid, ygrid) = self.tfmap2grid(xmap, ymap)
        (xocv, yocv) = self.tfgrid2ocv(xgrid, ygrid)
        return (xocv, yocv)

    def tfmap2grid(self, xmap, ymap):
        """ Transforms from map's reference frame to interest grid reference frame
            @param xmap: float, starting position of the line in the map's frame [meters]
            @param ymap: float, starting position of the line in the map's frame [meters]
            @return (xgrid, ygrid): [meters]
        """
        xgrid = xmap - self.origin[0]
        ygrid = ymap - self.origin[1]
        # print "xgrid= ", xgrid
        # print "ygrid= ", ygrid
        return (xgrid, ygrid)

    def tfgrid2ocv(self, xgrid, ygrid):
        """ Transforms from grid's reference frame to opencv reference frame
            @param xgrid: float, starting position of the line in the grid's frame [meters]
            @param ygrid: float, starting position of the line in the grid's frame [meters]
            @return (xocv, yocv): point in opencv reference frame [cells]
        """
        xocv = np.int(xgrid / self.resolution)
        yocv = np.int(self.height - ygrid / self.resolution)
        # print "xocv= ", xocv
        # print "yocv= ", yocv

        return (xocv, yocv)

    def tfocv2map(self, xocv, yocv):
        """ Transforms x and y in  opencv reference frame to /map reference frame
            @param xocv: int, position of the point in opencv reference frame [cells]
            @param yocv: int, position of the point in opencv reference frame [cells]
            @return (xmap, ymap): float:  point in /map reference frame [meters]
        """
        (xgrid, ygrid) = self.tfocv2grid(xocv, yocv)
        (xmap, ymap) = self.tfgrid2map(xgrid, ygrid)
        return (xmap, ymap)

    def tfocv2grid(self, xocv, yocv):
        """ Transforms x and y in  opencv reference frame to our grid reference frame
            @param xocv: int, position of the point in opencv reference frame [cells]
            @param yocv: int, position of the point in opencv reference frame [cells]
            @return (xgrid, ygrid): (float, float), point in grid's reference frame [meters]
        """
        xgrid = xocv * self.resolution
        ygrid = (self.height - yocv) * self.resolution
        return (xgrid, ygrid)

    def tfgrid2map(self, xgrid, ygrid):
        """ Transforms x and y in  our grid's reference frame to /map reference frame
            @param xgrid: float, position of the point in opencv reference frame [meters]
            @param ygrid: float, position of the point in opencv reference frame [meters]
            @return (xmap, ymap): float, point in /map reference frame [meters]
        """
        xmap = xgrid + self.origin[0]
        ymap = ygrid + self.origin[1]
        return (xmap, ymap)

    def display_image(self):
        """ Prints the interest grid in an opencv window.
        """

    def compute_final_point(self, x0, y0, yaw, d=100):
        """
            Computes the final point of a line segment given the starting point (x0, y0) in the required direction "yaw".
            @param x0: int, starting position of the line in the ocv frame [pixels]
            @param y0: int, starting position of the line in the ocv's frame [pixels]
            @param yaw: int, Angle of the line [rads]
            @param d: int, Length of the line segment [pixels]
            @return: python tuple (x1, y1), final point of the line in the required direction [pixels]

        Transformation between coordinate frames is given by the next relation:
        From here yaw is in the normal Cartesian reference plane and not in the opencv image fram

             |--->ocv (opencv coordinates)
             |
             v

             ^
             |
             |____>grid  coordinates
        """

        """ Special cases """
        eps = 0.000001
        yaw = -np.arctan2(np.sin(yaw), np.cos(yaw))  # Trick to define "yaw" angle within [-pi,pi] range.
        """ Avoid vertical lines """
        if  np.pi / 2 + eps > yaw > np.pi / 2 - eps:
            x1 = x0
            y1 = y0 + d
        elif -np.pi / 2 - eps < yaw < -np.pi / 2 + eps:
            x1 = x0
            y1 = y0 - d
        elif -eps <= yaw <= eps:
            """ Avoid horizontal lines """
            x1 = x0 + d
            y1 = y0
        elif np.pi - eps < yaw < np.pi + eps or -np.pi - eps < yaw < -np.pi + eps:
            x1 = x0 - d
            y1 = y0
        else:
            m = np.tan(yaw)  # gradient of the line
            if 0 < yaw < np.pi / 2:
                x1 = np.int(x0 + d / (np.sqrt(1 + m ** 2)))
                y1 = np.int(y0 + (m * (x1 - x0)))
            elif np.pi / 2 < yaw < np.pi:
                x1 = np.int(x0 - d / (np.sqrt(1 + m ** 2)))
                y1 = np.int(y0 + (m * (x1 - x0)))
            elif -np.pi / 2 < yaw < 0:
                x1 = np.int(x0 + d / (np.sqrt(1 + m ** 2)))
                y1 = np.int(y0 + (m * (x1 - x0)))
            elif -np.pi < yaw < -np.pi / 2:
                x1 = np.int(x0 - d / (np.sqrt(1 + m ** 2)))
                y1 = np.int(y0 + (m * (x1 - x0)))
        return (x1, y1)


    def bresenham_line(self, x0, y0, x1, y1, value):
        """
            This function will draw the line defined by the points (x0,y0) and (x1,y1) using the Bresenham's algorithm
            A value=value will be added to the interest grid along the line.
            @param x0,y0: int, starting position of the line in opencv's reference frame [pixels]
            @param y1,y1: int,  final position of the line in opencv's reference frame [pixels]
            @param value: int, value to be set along the line.
        """
        steep = np.absolute(y1 - y0) > np.absolute(x1 - x0)  # If line's slope is less than 1
        """
        The main loop contains a conditional over the variable steep.
        Since the basic Bresenham algorithm only works for lines with a slope less
        than 1, if the line to be plotted is "steep" then we step along the
        y-direction instead. That is:
        """
        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1
        """
        Swapping end point
        """
        if x0 > x1:
            x0, x1 = x1, x0
            y0, y1 = y1, y0
        """
        The direction in which the y-values are stepped depends on whether
        the end-point of the line is above or below the start-point.
        """
        if y0 < y1:
            ystep = 1
        else:
            ystep = -1
        """ Init
        The error is essentially a function of the gradient of the line
        (i.e. deltay/deltax. But in order to avoid floating point arithmetic,
        the expressions involving error are multiplied through by deltax
        (this may be an unnecessary optimization in Python, but is done here
        for consistency with other implementations).
        """
        deltax = x1 - x0
        deltay = np.absolute(y1 - y0)
        error = 0
        y = y0
        """ main loop
        The basic approach taken in Bresenham's algorithm is to step along the line
        from one end to the other in the x-direction, coloring successive pixels in
        a given row until the error between the actual line and the row being
        colored grows too great.
        """
        print "bresenham loop"
        for x in range(x0, x1 + 1):  # We add 1 to x1 so that the range includes x1
            if steep:
                if 0 < y < self.width and 0 < x < self.height:
                    """Print the point if it is inside the image"""
                    self.grid[x, y] = self.grid[x, y] + value
            else:
                if 0 < x < self.width and 0 < y < self.height:
                    """Print the point if it is inside the image"""
                    self.grid[y, x] = self.grid[y, x] + value
            error = error + deltay
            if (error << 1) >= deltax:
                y = y + ystep
                error = error - deltax

    def set_typical_destinations(self):
        """
        Finds and sets the list of most visited places, for the moment I just look for local maximums in
        the grid. The results are given in the world "/map" reference frame.
        """
        ocv_dest_list = self.get_corners()
        typical_dest_list = []  # [meters] This member contains a list of tuples [(x1,y1),(x2,y2) ...] with the locations of interest points in the /map reference frame
        for ocv_point in ocv_dest_list:
            map_point = self.tfocv2map(ocv_point[0], ocv_point[1])
            typical_dest_list.append(map_point)
        self.typical_dest_list = typical_dest_list

    def get_corners_img(self):
        """
        Gets the an image of the salient points of the grid
        @return: corners_img, cvMat type CV_8UC1, useful to be displayed
        """
        features_x_y_vector = self.get_corners()
        corners_img = cv.CreateMat(self.grid.rows, self.grid.cols, cv.CV_8UC1)
        cv.Set(corners_img, 0)
        for (x, y) in features_x_y_vector:
            corners_img[y, x] = 255
        return corners_img

    def get_corners(self):
        eig_image = cv.CreateMat(self.grid.rows, self.grid.cols, cv.CV_32FC1)
        temp_image = cv.CreateMat(self.grid.rows, self.grid.cols, cv.CV_32FC1)
        features_x_y_vector = cv.GoodFeaturesToTrack(self.grid, eig_image, temp_image, 10, 0.025, 1.0, useHarris=True)
        print "get corners finished"
        return features_x_y_vector

#===============================================================================
# If this code is executed instead of imported we will do this
#===============================================================================
if __name__ == "__main__":
    grid = InterestGrid(300, 300, (0, 0), 1)
    cv.NamedWindow("mi grid")
    for element in range(0, 8):
        print "element \t", element
        grid.set_line(100, 100, element * np.pi / 4 , 50, 30)
        cv.ShowImage("mi grid", grid.grid)
        cv.WaitKey(500)
    cv.WaitKey(1000000)
    cv.DestroyAllWindows()
    print "program finished"
