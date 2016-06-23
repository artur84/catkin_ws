#! /usr/bin/env python
"""
Created on Fri Feb 15 11:28:13 2013
Bressenham algorithm
@author: Jesus Arturo Escobedo Cabello
@contact: arturoescobedo.iq@gmail.com
"""
import numpy as np

class FormationType():
    """ Enumeration with the values defined in social_fcns.h from package social_filter.
    """
    NONE = 0
    VIS_VIS = 1
    V_FORM = 2
    L_FORM = 3
    C_FORM = 4
    CIRCULAR = 5
    HOI = 6
    I_FORM = 7
    text = {0: "Not in formation", 1:"Vis-a-Vis", 2:"V-form", 3:"L-form", 4:"C-form", 5:"Circular", 6:"Human-Object_Interaction", 7:"I-form"}


class Person():
    def __init__(self, pose = np.asarray([0.0, 0.0, 0.0])):
        """
        Initialize the person class
            A person will be defined by its (x,y) position with respect to the global map and orientation
            of his local reference frame with respect to the global frame. Using ROS conventions.
            The x_local axis points in the direction of the face of the person and the y_local axis follows the
            right hand convention when z points to the top. In that way phi is the angle meassured from the
            positive part of x_global to the positive part of x_local.

            y_local
            ^
            ^
            |
           /0\
          ( * --> > x_local
           \0/


            following the ros conventions where the
            @param pose:  numpy array, (x,y,phi) position of the person with respect to the global frame
                                                    phi is the angle measured from the positive part of x_global to the positive part of x_local.
        """
        self.position = pose[0:2]  #Numpy array, [x,y] Position of the person with respect to the global frame
        phi = pose[2]
        self.phi = np.deg2rad(phi)  # Float,  phi is the angle measured from the positive part of x_global to the positive part of x_local.
        self.alpha = np.deg2rad(phi + 90)  #The angle of the local_y axis with respect to the positive side of global_x axis. (Following ROS)


class Interaction():
    def __init__(self, person1_pose, person2_pose):
        """
        Initialize the interaction formed by two persons

            @param person1_pose:  Python List, containing [x,y,phi] of the first person
            @param person2_pose:  Python List, containing [x,y,phi] of the second person
            Example:
                 person1_pose ==> np.asarray([0.0, 2.0, 50.0])
                 person2_pose ==> np.asarray([2.0, 0.0, 90.0])
            @return True if there exist an interaction, False otherwise
        """
        # Members --------------------------------------------------------------
        print "P1= ", person1_pose
        print "P2= ", person2_pose

        self.person1 = Person(np.asarray(person1_pose))
        self.person2 = Person(np.asarray(person2_pose))
        self.__max_dist__ = 3  #Maximum distance between two persons to be considered as interacting

        #Initialize formation characteristics
        self.__init_vertix__()
        self.__init_center__()
        self.__init_formation_type__()
        self.__init_meeting_points__()



    def __init_formation_type__(self):
        """ Classifies the formation as vis-a-vis, v-form, c-form, L-form etc.
            sets the self.Type member, indicating the type of formation
        """
        # Compute unit vectors ----------------------------------------------
        uy1 = np.asarray([np.cos(self.person1.alpha), np.sin(self.person1.alpha)])  #unit vector of the local_y axis of this person
        uy2 = np.asarray([np.cos(self.person2.alpha), np.sin(self.person2.alpha)])  #unit vector of the local_y axis of this person
        # Compute the angle of the formation cos(theta) = a.b/(|a||b|) ---------
        self.gamma = np.arccos(np.dot(uy1, uy2))  #[rad] the angle between the visual axis of the two persons.
        gamma_deg = np.rad2deg(self.gamma)  #[deg] the angle between the visual axis of the two persons.
        print "self.gamma(deg):", gamma_deg
        # Compute the distance between the persons
        dist = np.linalg.norm(self.person1.position - self.person2.position).sum()  #distance between the two persons
        print "dist:", dist
        eps = 10  #degrees

        if dist <= self.__max_dist__:

            if self.DistVi[0] >= 0 and self.DistVi[1] >= 0:
                if gamma_deg >= 0  and gamma_deg < 55 - eps:
                    self.type = FormationType.I_FORM
                if gamma_deg >= 55 - eps and gamma_deg <= 90 - eps:
                    self.type = FormationType.C_FORM
                if gamma_deg > 90 - eps and gamma_deg <= 90 + eps:
                    self.type = FormationType.L_FORM
                if gamma_deg > 90 + eps and gamma_deg <= 180 - eps:
                    self.type = FormationType.V_FORM
                if gamma_deg > 180 - eps and gamma_deg <= 180:
                    self.type = FormationType.VIS_VIS
            elif self.DistVi[0] == -1 and self.DistVi[1] == -1:  #Check if it is an I formation
                if gamma_deg == 0.0:
                    self.type = FormationType.I_FORM
            else:
                self.type = FormationType.NONE
        else:
            self.type = FormationType.NONE
        print "type=", FormationType.text[self.type]

    def __init_center__(self):
        """ Computes the center of the interaction.

        Self.Vi should be  computed before calling this function.
        @return:  True if the center was computed properly, False otherwise.

        """
        # Compute the middle point: The point in the middle of the line joining the two persons --------------------------------------
        self.middle_point = np.divide(self.person1.position + self.person2.position, np.asarray([2.0, 2.0]))  #  The point in the middle of the line joining the two persons
                                                                                                             #  numpy array; (xm,ym); where, xm = (x1+x2)/2
        print "middle:", self.middle_point
        if self.Vi != None:
            #self.C = (self.Vi + self.middle_point) / 2  #The center of the interaction
            self.C = self.middle_point  #The center of the interaction
            print "Center:", self.C
            return True
        else:
            return False


    def __init_meeting_points__(self):
        """ Computes the meeting points of a given formation.

        """
        self.meeting_points_list = []  #Python list: The list of meeting points [[x1,y1],[x2,y2],....], The coordinates are in the global refference frame
        #Only compute meeting points if the two persons are actually interacting
        # Compute unit vector from H1 to C
        HC = np.asarray([self.C[0] - self.person1.position[0], self.C[1] - self.person1.position[1]])  #vector joining H1 and C
        HC_ortho = np.asarray([-HC[1], HC[0]])  #Orthogonal vector to HC
        ux1 = np.asarray([np.cos(self.person1.phi), np.sin(self.person1.phi)])  #unit vector of the local_y axis of this person
        print "ux1: ", ux1
        CV = np.asarray([self.Vi[0] - self.C[0], self.Vi[1] - self.C[1]])
        uCV = CV / np.linalg.norm(CV)
        # Compute the distance between the persons
        dist = np.linalg.norm(self.person1.position - self.person2.position).sum()  #distance between the two persons
        if self.type!=FormationType.NONE:
            if self.type == FormationType.L_FORM or self.type == FormationType.C_FORM:
                MP = self.C + 0.5 * dist * uCV
                self.meeting_points_list.append(MP)
                print "Meeting points", self.meeting_points_list
            elif self.type == FormationType.V_FORM:
                #The meeting points for a V_FORM or a VIS_VIS formation will be near the center
                MP = self.C + 0.5 * dist * uCV
                self.meeting_points_list.append(MP)
                print "Meeting points", self.meeting_points_list
            elif self.type == FormationType.I_FORM:
                #The meeting points for a V_FORM or a VIS_VIS formation will be near the center
                MP = self.C + 0.5 * dist * ux1
                self.meeting_points_list.append(MP)
                print "Meeting points", self.meeting_points_list
            elif self.type == FormationType.VIS_VIS:
                MP = self.C + HC_ortho
                self.meeting_points_list.append(MP)
                MP = self.C - HC_ortho
                self.meeting_points_list.append(MP)
                print "Meeting points", self.meeting_points_list
            else:
                print "Bad Formation type, error in __init_meeting_points__"

        else:
            print "There is no interaction so no meeting points can be computed."


    def __init_vertix__(self):
        """ Computes the point where the two visual axis of self.person2 and self.person2 intersect.
            It initializes the self.Vi and self.DistVi members
            @return: True if the intersection exist or 'False' if there is no intersection.
        """
        self.Vi = np.asarray([0.0, 0.0])  #The coordinates of the intersection between the visual axis.
        self.DistVi = np.asarray([-1.0, -1.0])  # DistVi(d1,d2), where d1 and d2 are the distances from person_n to the intersection.
                                                # dn will be positive if the intersection is in front of pn or negative if it is behind.
                                                # we will consider that there is no intersection in this formation if self.DistVi = (-1,-1).
        cos_phi = np.cos(self.person1.phi)
        sin_phi = np.sin(self.person1.phi)
        A = np.asarray([[cos_phi, -np.cos(self.person2.phi)],
                        [sin_phi, -np.sin(self.person2.phi)]])
        B = np.asarray([[self.person2.position[0] - self.person1.position[0]], [self.person2.position[1] - self.person1.position[1]]])
        try:
            """
            """
            self.DistVi = np.linalg.solve(A, B)
        except Exception as error:
            print "There is no intersection when trying to find self.DistVi for self.person1:", self.person1.position, "and self.person2:", self.person2.position
            return False
        Vix = self.person1.position[0] + self.DistVi[0] * cos_phi  # x1+dist1*cos(phi1)
        Viy = self.person1.position[1] + self.DistVi[0] * sin_phi  # y1+dist1*sin(phi1)
        self.Vi = np.transpose(np.asarray([Vix, Viy]))[0]  #The coordinates of the intersection between the visual axis.
        print "Vi=", self.Vi
        return True

#===============================================================================
# If this code is executed as main we will do this
#===============================================================================
if __name__ == "__main__":
    p1 = [0.0, 2.0, 50.0]
    p2 = [2.0, 0.0, 90.0]
    p3 = [2.0, 0.0, -90.0]
    p4 = [0.0, 0.0, 115.0]
    p5 = [-2.0, 0.0, 65.0]
    p6 = [0.0, -2.0, 0.0]
    p7 = [2.0, -2.0, 180.0]
    p8 = [4.0, 0.0, 90.0]
    print "***interaction p1, p2 ***"
    interaction1 = Interaction(p1, p2)
    print "***interaction2 p1, p3***"
    interaction2 = Interaction(p1, p3)
    print "***interaction3 p5, p4***"
    interaction3 = Interaction(p5, p4)
    print "***interaction4 p3, p6***"
    interaction3 = Interaction(p3, p6)
    print "***interaction5 p6, p7***"
    interaction3 = Interaction(p6, p7)
    print "***interaction6 p8, p2***"
    interaction3 = Interaction(p8, p2)






