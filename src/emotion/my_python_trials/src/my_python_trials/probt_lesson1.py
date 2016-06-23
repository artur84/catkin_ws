"""To be used in eclipse pydev first.
Go to:   Window->Preferences->Pydev Interpreter ->PYTHONPATH
 Add new folder with value GLOBAL_DIR/ProBtLinux32/lib 
 
 
IN run configurations: Run->Run Configurations->Environment
add name:LD_LIBRARY_PATH
value:GLOBAL_DIR/ProBtLinux32/lib 

(GLOBAL_DIR is the global route where the Probt folder is located e.g /home/artur)
"""


# import the pypl module
# import all 
from pypl import *

# transparent N1

#Type
irange = plIntegerType ( 0,1) 

#Symbols 
C= plSymbol  ("C", irange)
T= plSymbol ("T", irange)


PC = plProbTable(C,[0.99, 0.01]) #Probabilidad apriori de tener cancer
PT = plDistributionTable(T, C) #Probabilidad de cada resultado del test dado que tenemos cancer
PT.push(plProbTable(T, [0.8, 0.2]), 0) #PT.push(Table, value of evidence variable)
PT.push(plProbTable(T, [0.1, 0.9]), 1) #PT.push(Table, value of evidence variable)

PTC = plJointDistribution(PC*PT)
question = PTC.ask(C,T) #C sabiendo T

print question

result = question.instantiate(1)
print result

compiled_result = result.compile()


print compiled_result