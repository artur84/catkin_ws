#! /usr/bin/env python
"""
Created on Fri Feb 15 11:28:13 2013
Bressenham algorithm
@author: Jesus Arturo Escobedo Cabello
@contact: arturoescobedo.iq@gmail.com
"""
import cv2.cv as cv
from pyplpath import *
from pypl import *
import numpy as np
from copy import deepcopy
import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from mpl_toolkits.mplot3d.axes3d import Axes3D


class MapPossibleGoalsDetector():
    """
        Data structure to store the 3D histogram of most viewed places
        when the user is driving the wheelchair.
        ROS Independent0
    """
    def __init__(self, map_name="hall_inria"):
        """ Initialize our interest grid.

            @param y_length:  int, y_length of the grid [m]
            @param x_length:  int, Columns of the grid [m]
            @param origin: tuple (x,y), where it is going to be located the lower-left corner of the image with respect to the grid.
            @param resolution: float, resolution of the grid in [meters/cells]
        """
        """ MEMBERS """

        """ INIT """
        self.__read_map__(map_name) #reads the map and the map metadata

        #The image with the detected goal points
        self.goal_map=cv.CreateMat(self.map_img.height, self.map_img.width, cv.CV_8UC1)
        cv.Set(self.goal_map, 255)



        self.goal_list = []
        self.hough_rho =1 #- self.map_img.height/600#Biggest == more lines
        self.hough_threshold = 10 #Biggest==less lines
        print "An instance of InterestGrid has been created"


    def preproc_map_img(self, map_img):
        """ Preprocesses the map image Soft, erode or whtaever it is necessary to improve the input"""
        #Apply threshold to have just black and white
        thresh_img=cv.CreateMat(map_img.height, map_img.width, cv.CV_8UC1)
        cv.Threshold(map_img, thresh_img, 250, 255, cv.CV_THRESH_BINARY)

        #Blur map's thresholded image
        soft_img=cv.CreateMat(map_img.height, map_img.width, cv.CV_8UC1)
        cv.Smooth(thresh_img, soft_img, cv.CV_GAUSSIAN, 9, 9)

        #Dilate the inverse map to get it's skeleton
        dilated_img = cv.CreateMat(map_img.height, map_img.width, cv.CV_8UC1)
        cv.Dilate(soft_img, dilated_img, iterations=20)

        #Create inverse image
#        dilated_inverted_img=cv.CreateMat(map_img.height, map_img.width, cv.CV_8UC1)
#        for r in range(0,dilated_img.rows):
#            for c in range(0,dilated_img.cols):
#                dilated_inverted_img[r,c]=255-dilated_img[r,c]

        #Enhance image edges for hough transformdilated_img
        canny_img=cv.CreateMat(map_img.height, map_img.width, cv.CV_8UC1)
        cv.Canny(soft_img, canny_img, 200,220)

        preprocessed_map = dilated_img
        return preprocessed_map

    def plot_img(self,ocv_img):
        np_img = np.asarray(ocv_img)
        imgplot = plt.imshow(np_img)
        imgplot.set_cmap('hot')
        imgplot = plt.imshow(np_img)

    def find_lines_in_map_probabilistic(self, map_img):
        #Finds lines in the image using the probabilistic hough transform
        lines=cv.CreateMemStorage(0)
        line_img=cv.CreateMat(map_img.height, map_img.width, cv.CV_8UC1)
        cv.Set(line_img, 255)
        lines = cv.HoughLines2(map_img,cv.CreateMemStorage(), cv.CV_HOUGH_PROBABILISTIC, self.hough_rho,np.pi/2,self.hough_threshold)
        np_line=np.asarray(lines)
        print "list of probabilistic lines: ", np_line
#        x0=[]#A vector of all the X0
#        y0=[]#A vector of all the Y0
#        theta0=[]
#        n=0
#        Print the lines so that we can see what is happening
#        for rho,theta in np_line:
#            a = np.cos(theta)
#            b = np.sin(theta)
#            x0.append(a*rho)
#            y0.append(b*rho)
#            theta0.append(theta)
#            x1 = int(x0[n] + 3000*(-b))
#            y1 = int(y0[n] + 3000*(a))
#            x2 = int(x0[n] - 3000*(-b))
#            y2 = int(y0[n] - 3000*(a))
#            cv.Line(map_img,(x1,y1),(x2,y2),0,1)
#            cv.Line(line_img,(x1,y1),(x2,y2),0,1)
#            n = n+1
#
#        create two lists with the x coordinate of vertical lines and y coordinate of horixontal lines.
#        theta_rounded = np.round(theta0, 1)
#        y_sorted = np.sort(np.round(y0,0))
#        ylist=[]
#        xlist=[]
#        n=0
#        for element in theta_rounded:
#            if element == 0:#Horizontal line
#                xlist.append(x0[n])
#            else:
#                ylist.append(y0[n])
#            n=n+1
#
#        Ym=[]
#        Xm=[]
#        ordered_y = np.sort(ylist)
#        ordered_x = np.sort(xlist)
#        print ordered_x
#        last_element =0
#
#        Find middle points between lines
#        for element in ordered_y:
#            delta_element = element - last_element
#            Ym.append(last_element+(delta_element)/2)
#            last_element = deepcopy(element)
#        last_element =0
#        for element in ordered_x:
#            delta_element = element - last_element
#            Xm.append(last_element+(delta_element)/2)
#            last_element = deepcopy(element)
#
#        Printing the points in a map
#        for yi in Ym:
#            for xi in Xm:
#                if self.map_img[yi,xi] >= 250: #If free space
#                    self.goal_list.append([yi,xi])
#                    map_img[yi,xi]=255
#                    self.goal_map[yi,xi]=0

    def find_lines_in_map(self, map_img):
        #Finds lines in the image
        lines=cv.CreateMemStorage(0)
        line_img=cv.CreateMat(map_img.height, map_img.width, cv.CV_8UC1)
        cv.Set(line_img, 255)
        lines = cv.HoughLines2(map_img,cv.CreateMemStorage(), cv.CV_HOUGH_MULTI_SCALE, self.hough_rho,np.pi/2,self.hough_threshold)
        np_line=np.asarray(lines)
        print
        x0=[]#A vector of all the X0
        y0=[]#A vector of all the Y0
        theta0=[]
        n=0
        #Print the lines so that we can see what is happening
        for rho,theta in np_line:
            a = np.cos(theta)
            b = np.sin(theta)
            x0.append(a*rho)
            y0.append(b*rho)
            theta0.append(theta)
            x1 = int(x0[n] + 3000*(-b))
            y1 = int(y0[n] + 3000*(a))
            x2 = int(x0[n] - 3000*(-b))
            y2 = int(y0[n] - 3000*(a))
            cv.Line(map_img,(x1,y1),(x2,y2),0,1)
            cv.Line(line_img,(x1,y1),(x2,y2),0,1)
            n = n+1

        #create two lists with the x coordinate of vertical lines and y coordinate of horixontal lines.
        theta_rounded = np.round(theta0, 1)
        y_sorted = np.sort(np.round(y0,0))
        ylist=[]
        xlist=[]
        n=0
        for element in theta_rounded:
            if element == 0:#Horizontal line
                xlist.append(x0[n])
            else:
                ylist.append(y0[n])
            n=n+1

        Ym=[]
        Xm=[]
        ordered_y = np.sort(ylist)
        ordered_x = np.sort(xlist)
        print ordered_x
        last_element =0

        #Find middle points between lines
        for element in ordered_y:
            delta_element = element - last_element
            Ym.append(last_element+(delta_element)/2)
            last_element = deepcopy(element)
        last_element =0
        for element in ordered_x:
            delta_element = element - last_element
            Xm.append(last_element+(delta_element)/2)
            last_element = deepcopy(element)

        #Printing the points in a map
        for yi in Ym:
            for xi in Xm:
                if self.map_img[yi,xi] >= 250: #If free space
                    self.goal_list.append([yi,xi])
                    map_img[yi,xi]=255
                    self.goal_map[yi,xi]=0


        return map_img
#
    def get_gradient_img(self, input_img):
        # Generate grad_x and grad_y to use Sobel
        grad_x=cv.CreateMat(input_img.height, input_img.width, input_img.type)
        grad_y=cv.CreateMat(input_img.height, input_img.width, input_img.type)
        abs_grad_x=cv.CreateMat(input_img.height, input_img.width, input_img.type)
        abs_grad_y=cv.CreateMat(input_img.height, input_img.width, input_img.type)
        grad =cv.CreateMat(input_img.height, input_img.width, input_img.type)

        #Gradient X
        #Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );#for small kernels use this instead of sobel
        cv.Sobel( input_img, grad_x, 1, 0)
        cv.Abs(grad_x, abs_grad_x )

        #Gradient Y
        #Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
        cv.Sobel( input_img, grad_y, 0, 1)
        cv.Abs( grad_y, abs_grad_y )

        #/// Total Gradient (approximate)
        cv.AddWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad )
        return grad


    def compute_distance_image(self, input_map):
        #Create necessary matrices
        dist_matrix=cv.CreateMat(input_map.height, input_map.width, cv.CV_32FC1)
        dist_img_gray=cv.CreateMat(input_map.height, input_map.width, cv.CV_8UC1)
        #Get the Euclidean distance from every free point in the map to the closest obstacle
        cv.DistTransform(input_map, dist_matrix)
        #To make the pikes of the distance function we use the square of the resulted value
        square_mat=cv.CreateMat(input_map.height, input_map.width, cv.CV_32FC1)
        cv.Pow(dist_matrix,square_mat, 3)
        #We normalize the values of all pixels to print the image.
        matrix = dist_matrix
        max_val = np.max(matrix)
        m=255/max_val
        for r in range(0, matrix.height):
            for c in range(0, matrix.width):
                dist_img_gray[r,c]=np.int(m*matrix[r,c])
        return(dist_img_gray, dist_matrix)

    def __segment_map__(self):
        (dist_img_gray, dist_matrix) = self.compute_distance_image()
        self.segmented_map=self.post_process_distance_img(dist_img_gray)


    def post_process_distance_img(self, dist_img):
        inverted_img=cv.CreateMat(dist_img.height, dist_img.width, cv.CV_8UC1)

        #Blur image
        soft_img=cv.CreateMat(dist_img.height, dist_img.width, cv.CV_8UC1)
        cv.Smooth(dist_img, soft_img, cv.CV_GAUSSIAN, 21, 21)


        #Apply threshold to have just black and white
        thresh_img=cv.CreateMat(dist_img.height, dist_img.width, cv.CV_8UC1)
        cv.Threshold(soft_img, thresh_img, 1, 255, cv.CV_THRESH_BINARY)#CV_THRESH_OTSU is an adaptive thresholding method



#        #Create inverse image
#        for r in range(0,thresh_img.rows):
#            for c in range(0,thresh_img.cols):
#                inverted_img[r,c]=255-thresh_img[r,c]

        #Erode the inverse map to get it's skeleton
        eroded_img = cv.CreateMat(dist_img.height, dist_img.width, cv.CV_8UC1)
        cv.Erode(inverted_img, eroded_img, iterations=10)

        #Create inverse image
        for r in range(0,eroded_img.rows):
            for c in range(0,eroded_img.cols):
                inverted_img[r,c]=255-eroded_img[r,c]

        return inverted_img



    def find_rectangles(self,input_img):
        """ Find contours in the input image.
        input_img: Is a binary image
        """
        contours_img=cv.CreateMat(input_img.height, input_img.width, cv.CV_8UC1) # Image to draw the contours
        copied_img=cv.CreateMat(input_img.height, input_img.width, input_img.type) # Image to draw the contours
        cv.Copy(input_img, copied_img)
        contours = cv.FindContours(copied_img,cv.CreateMemStorage(),cv.CV_RETR_TREE,cv.CV_CHAIN_APPROX_SIMPLE)
        cv.DrawContours(contours_img,contours,255,0,10)
        return contours_img

    def __read_map__(self, map_name):
        yaml_file = map_name+".yaml"
        self.__read_map_metadata__(yaml_file)
        self.map_img=cv.LoadImage(self.map_img_name, cv.CV_LOAD_IMAGE_GRAYSCALE) ## @var opencv image.


    def __read_map_metadata__(self, file_name):
        """ Reads the map metadata included in the yaml file
            @param file_name: String
        """
        file = open(file_name, 'r')
        data = yaml.load(file)
        self.map_resolution = np.asfarray(data['resolution'])
        self.map_origin = np.asfarray(data['origin'])
        self.map_img_name = data['image']
        print "resolultion:"
        print self.map_resolution
        print "map origin:"
        self.map_origin
        file.close()



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

    def find_corners_in_map(self):
        """
        Gets the an image of the salient points of the grid
        @return: corners_img, cvMat type CV_8UC1, useful to be displayed
        """
        corner_img = cv.CreateMat(self.map_img.height, self.map_img.width, cv.CV_8UC3)
        features_x_y_vector = self.get_corners()
        for (x, y) in features_x_y_vector:
            corner_img[y, x] = (0,255,0)
        return corner_img

    def get_corners(self):
        eig_image = cv.CreateMat(self.map_img.height, self.map_img.width, cv.CV_32FC1)
        temp_image = cv.CreateMat(self.map_img.height, self.map_img.width, cv.CV_32FC1)
        features_x_y_vector = cv.GoodFeaturesToTrack(self.map_img, eig_image, temp_image, self.map_img.width, 0.15, 10, useHarris=True)
        print "get corners finished"
        return features_x_y_vector

#===============================================================================
# If this code is executed instead of imported we will do this
#===============================================================================
if __name__ == "__main__":
    grid = MapPossibleGoalsDetector("inria_full")

    """ Preprocess Map"""
    preprocessed_map = grid.preproc_map_img(grid.map_img)
    np_img = np.asarray(preprocessed_map)
    fig_4 =plt.figure(4)
    #plt.subplot(2,1,1)
    imgplot=plt.imshow(np_img, cmap="jet")
    fig_4.colorbar(imgplot)



    """ Plot Distance Image """
    (dist_img_gray, dist_img) = grid.compute_distance_image(preprocessed_map)
    np_img = np.asarray(dist_img)
    fig_1 =plt.figure(1)
    #plt.subplot(2,1,1)
    imgplot=plt.imshow(np_img, cmap="jet")
    fig_1.colorbar(imgplot)



    """ Plot PostProcessed Image """
    dist_img_post = grid.post_process_distance_img(dist_img_gray)
    np_img = np.asarray(dist_img_post)
    fig_2 = plt.figure(2)
    imgplot=plt.imshow(np_img, cmap="gray")#autumn,gray
    fig_2.colorbar(imgplot)


    """ Plot contours """

    contours_img = grid.find_rectangles(dist_img_post)
    np_img = np.asarray(contours_img)
    fig_3 = plt.figure(3)
    imgplot=plt.imshow(np_img, cmap="gray")#autumn,gray
    fig_3.colorbar(imgplot)

#    """ Plot Hough lines of distance image"""
#    lines_img = grid.find_lines_in_map_probabilistic(dist_img_gray)
#    np_img = np.asarray(lines_img)
#    plt.figure(3)
#    imgplot=plt.imshow(np_img, cmap="jet")#autumn,gray




    plt.show([1,2,3, 4])



    print "program finished"
