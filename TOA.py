import matplotlib.pyplot as plt
from sympy import *
import numpy as np
import random
import turtle
import math
#c=299552816

def plot_cicle(centers,rads,MT,BT_x,BT_y):
    fig=plt.figure()
    ax = fig.add_subplot(111)
    for center,rad in zip(centers,rads):
        cir=plt.Circle((center[0],center[1]),radius=rad,color='y',fill=False)
        ax.add_patch(cir)

    ax.plot(BT_x,BT_y,'ro')
    ax.plot(MT[0],MT[1],'ro')
    plt.axis('scaled')
    plt.axis('equal')
    plt.show()

def CRLB(MT,BT,sigma):
    a,b,c=0,0,0
    for bt in BT:
        deno=sigma**2*((MT[0]-bt[0])**2+(MT[1]-bt[1])**2)
        a=a+(MT[0]-bt[0])**2/deno
        b=b+((MT[0]-bt[0])*(MT[1]-bt[1]))/deno
        c=c+(MT[1]-bt[1])**2/deno
    try:
        Fisher=np.array([[a,b],[b,c]])
        CLB=np.linalg.inv   (Fisher)
        print(CLB)
    except:
        print("矩阵不可逆")
    return CLB

class AML:

    def __init__(self,bt,mt,bt_x,bt_y,Distance,Dis_):
        self.BT=bt
        self.MT=mt
        self.BT_x=bt_x
        self.BT_y=bt_y
        self.Distance=Distance
        self.dis_=Dis_
        '''
        self.BT=[]
        for i in range(number):
            self.BT.append([random.random() * 100, random.random() * 100])
        self.MT=[random.random()*100,random.random()*100]
        self.BT_x=[x[0] for x in self.BT]
        self.BT_y = [x[1] for x in self.BT]
        print("MT:",self.MT)
        self.dis_, self.Distance, self.MPE = [], [], []
        # true distance
        for x, y in self.BT:
            self.Distance.append(((self.MT[0] - x) ** 2 + (self.MT[1] - y) ** 2) ** 0.5)
        # noise
        for d in self.Distance:
            self.MPE.append(math.log(d) * random.gauss(ave, sigma))
        # estimate distance
        for d, n in zip(self.Distance, self.MPE):
            self.dis_.append(d + n)
        '''

    def RSR(self,dis, pos, BT):
        sum = 0
        for d, p, bt in zip(dis, pos, BT):
            elem = (d - ((pos[0] - bt[0]) ** 2 + (pos[1] - bt[1]) ** 2) ** 0.5) ** 2
            sum = sum + elem
        return sum

    def cal(self,tag, paras, cords, c_p=0):
        sum, g_i = 0, []
        if tag == 0:
            for para, cord in zip(paras, cords):
                g_i.append((c_p - cord) / (2 * para ** 2))
            return g_i

        if tag == 1:
            for para, cord in zip(paras, cords):
                sum = sum + para * cord
            return sum

    def cal1(self,s, P, X, Y, D):
        sum = 0
        for p, x, y, d in zip(P, X, Y, D):
            sum = sum + p * (s + x ** 2 + y ** 2 - d ** 2)
        return sum

    def cal2(self,s, P, X, Y, D):
        sum = 0
        if isinstance(s,complex):
            s=s.real
        else:
            s=abs(s)
        for p, x, y, d in zip(P, X, Y, D):
            sum = sum + p * (s + x ** 2 + y ** 2 - d ** 2)
        return sum

    def init_value(self):
        a,b=2*(self.BT_x[1]-self.BT_x[0]),2*(self.BT_y[1]-self.BT_y[0])
        c=(self.dis_[0]**2-self.dis_[1]**2)-((self.BT_x[0]**2-self.BT_x[1]**2)+(self.BT_y[0]**2-self.BT_y[1]**2))
        d,e=2*(self.BT_x[2]-self.BT_x[0]),2*(self.BT_y[2]-self.BT_y[0])
        f=(self.dis_[0] ** 2 - self.dis_[2] ** 2) - ((self.BT_x[0]**2 - self.BT_x[2]**2) + (self.BT_y[0]**2 - self.BT_y[2]**2))
        y = (f - d / a * c) / (e - d / a * b)
        x=(c-b*y)/a
        #plot_cicle(self.BT, self.dis_, self.MT, self.BT_x, self.BT_y)
        return x,y

    def estimate_one(self,x_ini,y_ini):
        self.g_i,self.h_i=self.cal(0,self.dis_,self.BT_x,c_p=x_ini),self.cal(0,self.dis_,self.BT_y,c_p=y_ini)
        a,b,c,d=self.cal(1,self.g_i,self.BT_x),self.cal(1,self.g_i,self.BT_y),self.cal(1,self.h_i,self.BT_x),self.cal(1,self.h_i,self.BT_y)
        A=2*np.array([[a,b],[c,d]])
        s=Symbol('s')
        e=self.cal1(s,self.g_i,self.BT_x,self.BT_y,self.dis_)
        f = self.cal1(s, self.h_i, self.BT_x, self.BT_y, self.dis_)
        B=np.array([e,f]).T
        A_=np.linalg.inv((A.T).dot(A)).dot(A.T)
        s_=solve((A_[0][0]*e+A_[0][1]*f)**2+(A_[1][0]*e+A_[1][1]*f)**2-s,s)
        #print(s_)
        e_1=self.cal2(s_[0],self.g_i,self.BT_x,self.BT_y,self.dis_)
        f_1=self.cal2(s_[0],self.h_i,self.BT_x,self.BT_y,self.dis_)
        B_1= np.array([e_1, f_1]).T

        e_2 = self.cal2(s_[1], self.g_i, self.BT_x, self.BT_y, self.dis_)
        f_2 = self.cal2(s_[1], self.h_i, self.BT_x, self.BT_y, self.dis_)
        B_2 = np.array([e_2, f_2]).T
        try:
            position1=np.linalg.inv(A).dot(B_1)
            position2 = np.linalg.inv(A).dot(B_2)
            t1=self.RSR(self.dis_,position1,self.BT)
            t2=self.RSR(self.dis_,position2,self.BT)
            return position1 if t1<t2 else position2
        except:
            print("矩阵不可逆")

    def estimate(self):
        x_ini,y_ini=self.init_value()
        #print(x_ini,y_ini)
        pos_,J_v = [],[]
        location = [x_ini, y_ini]
        for i in range(5):
            sum=0
            for d,cord in zip(self.dis_,self.BT):
                sum=sum+(d-((location[0]-cord[0])**2+(location[1]-cord[1])**2)**0.5)**2
            J_v.append(sum)
            location = self.estimate_one(float(location[0]), float(location[1]))
            pos_.append(list(location))
        id = J_v.index(min(J_v))
        print("estimate_location:",pos_[id])

def main():
    aml=AML(5,0,2)
    CRLB(aml.MT,aml.BT,2)
    #aml.estimate()

if __name__=="__main__":
    main()