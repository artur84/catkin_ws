"""To be used in eclipse pydev first.
Go to:   Window->Preferences->Pydev Interpreter ->PYTHONPATH
 Add new folder with value GLOBAL_DIR/ProBtLinux32/lib 
 
 
IN run configurations: Run->Run Configurations->Environment
add name:LD_LIBRARY_PATH
value:GLOBAL_DIR/ProBtLinux32/lib 

(GLOBAL_DIR is the global route where the Probt folder is located e.g /home/artur)
"""


from pypl import *
from math import *

def radiantodegree(x):
    r= 180.0*x/pi
    if r > 270 :
        r= r-360
    return r


Coord_X_goal=[100,0,0]
Coord_Y_goal=[0,100,-100]

Goal = plSymbol('Goal',plIntegerType(0,2))# The index of each goal whose coordinates are stored in Coord_X_goal
X = plSymbol('X',plIntegerType(0,100))#Pos of the goals with respect to the robot???
Y =  plSymbol('Y',plIntegerType(0,100))#Pos of the goals with respect to the robot???
Command = plSymbol('Command',plIntegerType(-(90+45),(90+45)))


P_Goal = plProbTable(Goal,[0.3,0.3,0.4])
P_X = plUniform(X)
P_Y = plUniform(Y)

def f_command(Output_,Input_):
    Output_[0]= radiantodegree(atan2(Coord_Y_goal[Input_[Goal].to_int()]-Input_[Y].to_float(),\
                                     Coord_X_goal[Input_[Goal].to_int()])-Input_[X].to_float())


JointDistributionList=plComputableObjectList()
JointDistributionList.push_back(P_Goal)
JointDistributionList.push_back(P_Y)
JointDistributionList.push_back(P_X)
#JointDistributionList.push_back(plCndNormal(Command,X^Y^Goal,Mean , Std_deviation))
JointDistributionList.push_back(plCndNormal(Command,X^Y^Goal,plPythonExternalFunction(X^Y^Goal,f_command), 30))

model=plJointDistribution(JointDistributionList)

question= model.ask(Goal,Command^X^Y)

sensor_reading_values=plValues(Command^X^Y)


sensor_reading_values[Command]=45
sensor_reading_values[X]=0
sensor_reading_values[Y]=0

result=question.instantiate(sensor_reading_values) #It just instantiate but doesn't make any operation


print result.tabulate() #Here is where
print result.compile().best()

