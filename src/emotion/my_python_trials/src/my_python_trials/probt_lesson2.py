"""To be used in eclipse pydev first.
Go to:   Window->Preferences->Pydev Interpreter ->PYTHONPATH
 Add new folder with value GLOBAL_DIR/ProBtLinux32/lib 
 
 
IN run configurations: Run->Run Configurations->Environment
add name:LD_LIBRARY_PATH
value:GLOBAL_DIR/ProBtLinux32/lib 

(GLOBAL_DIR is the global route where the Probt folder is located e.g /home/artur)
"""
from pypl import *

################### EJEMPLO HECHO EN CLASE ######33
GrayClouds = plSymbol('GrayClouds', plIntegerType(0,1))
Sprinkler = plSymbol('Sprinkler', plIntegerType(0,1))
WetGrass = plSymbol('WetGrass', plIntegerType(0,1))
Rain = plSymbol('Rain', plIntegerType(0,1))

PGC = plProbTable(GrayClouds, [0.5, 0.5])

PSgivenGC = plDistributionTable(Sprinkler, GrayClouds)
PSgivenGC.push(plProbTable(Sprinkler, [0.5,0.5]),0)
PSgivenGC.push(plProbTable(Sprinkler, [0.9,0.1]),1)

PRgivenGC = plDistributionTable(Rain, GrayClouds)
PRgivenGC.push(plProbTable(Rain, [0.8,0.2]),0)
PRgivenGC.push(plProbTable(Rain, [0.35,0.65]),1)

PWGgivenSR = plDistributionTable(WetGrass, Sprinkler^Rain)
V=plValues(Rain^Sprinkler)
#For the table 
V[Rain]=0
V[Sprinkler] = 0
PWGgivenSR.push(plProbTable(WetGrass, [1.0,0.0]),V)

V[Rain]=0
V[Sprinkler] = 1
PWGgivenSR.push(plProbTable(WetGrass, [0.1,0.9]),V)

V[Rain]=1
V[Sprinkler] = 0
PWGgivenSR.push(plProbTable(WetGrass, [0.1,0.9]),V)

V[Rain]=1
V[Sprinkler] = 1
PWGgivenSR.push(plProbTable(WetGrass, [0.01,0.99]),V)

#Joint distribution
model = plJointDistribution(PGC*PSgivenGC*PRgivenGC*PWGgivenSR)
#problem definition 
question = model.ask(GrayClouds, WetGrass) 

V2=plValues(WetGrass)
VC=plValues(GrayClouds)
VC[GrayClouds]=0
V2[WetGrass] = 1

result =  question.instantiate(V2)

compiled_result = result.compile()

print result.compute(VC)

print compiled_result





#-------------------- Diagnostic Medical 
#type
Binary = plIntegerType(0,1)

#Variables
C=plSymbol('C',Binary)
T=plSymbol('T',Binary)

#distributions
PC=plProbTable(C,[0.99, 0.01])

#conditional distribution 
PT_k_C = plDistributionTable(T,C)

#distribution on T knowing C=0
V= plValues(C^T)

PT_k_0 = plProbTable(T,[0.8,0.2])

#partial definition of the conditional distribution 
V[C]=0
PT_k_C.push(PT_k_0,V)

#to complete the definition of the condition distribution
#shortcut 
PT_k_C.push(plProbTable(T,[0.1,0.9]),1)

#model or joint distribution
model = plJointDistribution(PC*PT_k_C)

#inference 
question = model.ask(C,T)

#use
V[T]=1
result = question.instantiate(V)

#display probability table 
print result.compile()

#------------------ Dirac sum of two dices 

D1=plSymbol("D1",plIntegerType(1,6))
D2=plSymbol("D2",plIntegerType(1,6))
S=plSymbol("S",plIntegerType(2,12))
values = plValues(D1^D2)

def my_plus(O_,I_) : 
    O_[S]=I_[D1]+I_[D2]

funcmodel=plPythonExternalFunction(S,D1^D2,my_plus)
PS_k_D1D2= plFunctionalDirac(S,D1^D2,funcmodel)

PD1 = plUniform(D1)
PD2 = plUniform(D2)

model = plJointDistribution(PD1*PD2*PS_k_D1D2)

question =model.ask(D1)

print question.compile()


#-------------------- Dirac diagnostic 
A=plSymbol("A",Binary)
B=plSymbol("B",Binary)
C=plSymbol("C",Binary)
D=plSymbol("D",Binary)
E=plSymbol("E",Binary)
F=plSymbol("F",Binary)

def my_or(O_,I_):
    O_[D]=I_[A] or I_[B]

def my_and_one(O_,I_):
    O_[E]=I_[C] and I_[B]

def my_and_two(O_,I_):
    O_[F]=I_[D] and I_[E]

fAB=plPythonExternalFunction(D,A^B,my_or)
PD_k_AB= plFunctionalDirac(D,A^B,fAB)

fCB=plPythonExternalFunction(E,C^B,my_and_one)
PE_k_CB= plFunctionalDirac(E,C^B,fCB)

fDE=plPythonExternalFunction(F,D^E,my_and_two)
PF_k_DE= plFunctionalDirac(F,D^E,fDE)

model = plJointDistribution(plUniform(A)* 
                            plUniform(B) * \
                            plUniform(C) * \
                            PD_k_AB * PE_k_CB *  PF_k_DE)

question = model.ask(A^B^C,F)

result = question.instantiate(0).compile()

print result

# --------------------- very simple learning 
HC=plLearnHistogram(C)

HC.add_point(0)
HC.add_point(0)
HC.add_point(1)

PC=HC.get_distribution()

HC.add_point(1)
PC=HC.get_distribution()


#---------------------- EM learning generating data

C = plSymbol('C',plIntegerType(0,1))
W = plSymbol('W',plRealType(0,100))
#define a ML learner for a binomial law
#define intial guess : P(Lambda|pi_0)
pC = plBinomial(C,0.55)
pWkC  = plDistributionTable(W,C)
pWkC.push(plNormal(W,45.0,10.0),0)
pWkC.push(plNormal(W,65.0,10.0),1)

experimental_data = open('C:/Users/mazer/Documents/Inria/cours/gender_weight.csv','w')
experimental_data.write('W\n')

values=plValues(C^W)
#..........

for i in range(1000):
    #................
    experimental_data.write('{0}\n'.format(values[W]))

experimental_data.close()

#---------------------- EM learning from data 
pC_learner = plLearnHistogram(C)
pW_learner = plCndLearn1dNormal(W,C)
#define intial guess
pC_init = plBinomial(C,0.7)
pWkC_init = plDistributionTable(W,C)
pWkC_init.push(plNormal(W,20.0,40.0),0)
pWkC_init.push(plNormal(W,70.0,40.0),1)
#define the learner 
learner=plEMLearner(pC_init*pWkC_init,
                    [pC_learner,pW_learner])
#define the data source 
file = 'C:/Users/mazer/Documents/Inria/cours/gender_weight.csv'
data = plCSVDataDescriptor(file,W)
#perform learning stop with a threshold 
learner.run(data,10e-9)
#get the prameters
print learner.get_distribution(0)
print learner.get_distribution(1)

#frozen distribution 

C_learner = plLearnFrozenDistribution (C)











