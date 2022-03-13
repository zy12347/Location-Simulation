import matplotlib.pyplot as plt
from sympy import *
import cvxpy as cp
import numpy as np
import cvxopt
import random
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

class Program:

    def __init__(self,mt,bt_x,bt_y,Distance,Dis_):
        self.MT=mt
        self.BT_x=bt_x
        self.BT_y=bt_y
        self.Distance=Distance
        self.dis_=Dis_
#-------------------------------------------for CRLB

    def CRLB(self):
        a, b, c,sigma= 0, 0, 0,1
        for bt_x, bt_y in zip(self.BT_x, self.BT_y):
            deno = sigma ** 2 * ((self.MT[0] - bt_x) ** 2 + (self.MT[1] - bt_y) ** 2)
            a = a + (self.MT[0] - bt_x) ** 2 / deno
            b = b + ((self.MT[0] - bt_x) * (self.MT[1] - bt_y)) / deno
            c = c + (self.MT[1] - bt_y) ** 2 / deno
        try:
            Fisher = np.array([[a, b], [b, c]])
            crlb = 1 / (Fisher)
            print(crlb)
        except:
            print("矩阵不可逆")
        return crlb
#----------------------------------------------For ML algorithm

    def RSR(self,dis, pos, BT_x,BT_y):
        sum = 0
        for d, p, bt_x,bt_y in zip(dis, pos, BT_x,BT_y):
            elem = (d - ((pos[0] - bt_x) ** 2 + (pos[1] - bt_y) ** 2) ** 0.5) ** 2
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
        # deal with BT in a line
        if self.BT_y[0]==self.BT_y[1] and self.BT_y[0]==self.BT_y[2]:
            x=((self.dis_[1]**2-self.dis_[0]**2)-(self.BT_x[1]**2-self.BT_x[0]**2))/(2*(self.BT_x[0]-self.BT_x[1]))
            y=(self.dis_[0]**2-(x-self.BT_x[0])**2)**0.5+self.BT_y[0]

        elif self.BT_x[0]==self.BT_x[1] and self.BT_x[0]==self.BT_x[2]:
            y = ((self.dis_[1] ** 2 - self.dis_[0] ** 2) - (self.BT_y[1] ** 2 - self.BT_y[0] ** 2)) / (2 * (self.BT_y[0] - self.BT_y[1]))
            x = (self.dis_[0] ** 2 - (y - self.BT_y[0]) ** 2) ** 0.5 + self.BT_x[0]

        else:
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
            t1=self.RSR(self.dis_,position1,self.BT_x,self.BT_y)
            t2=self.RSR(self.dis_,position2,self.BT_x,self.BT_y)
            return position1 if t1<t2 else position2
        except:
            print("矩阵不可逆")

    def estimate(self):
        x_ini,y_ini=self.init_value()
        pos_,J_v = [],[]
        location = [x_ini, y_ini]
        for i in range(5):
            sum=0
            for d,cord_x,cord_y in zip(self.dis_,self.BT_x,self.BT_y):
                sum=sum+(d-((location[0]-cord_x)**2+(location[1]-cord_y)**2)**0.5)**2
            J_v.append(sum)
            location = self.estimate_one(float(location[0]), float(location[1]))
            pos_.append(list(location))
        id = J_v.index(min(J_v))
        #print("estimate_location:",pos_[id])
        pos_[id]=[round(pos_[id][0],1),round(pos_[id][1],1)]
        return pos_[id]

#-------------------------------------------------------------------------for LS

    def LLS_1_E(self):
        A,p=[],[]
        for bt_x,bt_y,dis in zip(self.BT_x[1:],self.BT_y[1:],self.dis_[1:]):
            A.append([bt_x-self.BT_x[0],bt_y-self.BT_y[0]])
            p.append([self.dis_[0]**2-dis**2-((self.BT_x[0]**2+self.BT_y[0]**2)-(bt_x**2+bt_y**2))])
        A3=np.array(A)
        p3=np.array(p)
        pos=0.5*(np.linalg.inv((A3.T).dot(A3))).dot(A3.T).dot(p3)
        position=[round(pos[0][0],1),round(pos[1][0],1)]
        return position

    def LLS_AVE_E(self):
        A,p=[],[]
        d_ave=sum(list(map(lambda d: d**2,self.dis_)))/len(self.dis_)
        k_ave=sum(list(map(lambda x,y: x**2+y**2,self.BT_x,self.BT_y)))/len(self.BT_x)
        x_ave=sum(self.BT_x)/len(self.BT_x)
        y_ave = sum(self.BT_y) / len(self.BT_y)
        for bt_x,bt_y,dis in zip(self.BT_x,self.BT_y,self.dis_):
            A.append([bt_x-x_ave,bt_y-y_ave])
            p.append([d_ave-dis**2+(bt_x**2+bt_y**2)-k_ave])
        A3=np.array(A)
        p3=np.array(p)
        pos=0.5*(np.linalg.inv((A3.T).dot(A3))).dot(A3.T).dot(p3)
        position=[round(pos[0][0],1),round(pos[1][0],1)]
        return position

    def LLS_RS_E(self):
        A,p=[],[]
        i_d=self.dis_.index(min(self.dis_))
        d_min,x_min,y_min=self.dis_[i_d],self.BT_x[i_d],self.BT_y[i_d]
        BT_X=self.BT_x[0:i_d]+self.BT_x[i_d+1:]
        BT_Y = self.BT_y[0:i_d] + self.BT_y[i_d + 1:]
        dis_=self.dis_[0:i_d]+self.dis_[i_d+1:]
        for bt_x,bt_y,dis in zip(BT_X,BT_Y,dis_):
            A.append([bt_x-x_min,bt_y-y_min])
            p.append([d_min**2-dis**2-((x_min**2+y_min**2)-(bt_x**2+bt_y**2))])
        A3=np.array(A)
        p3=np.array(p)
        pos=0.5*(np.linalg.inv((A3.T).dot(A3))).dot(A3.T).dot(p3)
        position=[round(pos[0][0],1),round(pos[1][0],1)]
        return position

    def R_LS_E(self):
        num=len(self.dis_)
        X, G = cp.Variable((3, 3),PSD=true), cp.Variable((num + 1, num + 1),PSD=true)# PSD=true means semidefinite
        expr=0
        constraints = [G[num, num] == 1, X[2, 2] == 1]
        for i in range(num):
            C_i=[[1,0,-self.BT_x[i]],[0,1,-self.BT_y[i]],[-self.BT_x[i],-self.BT_y[i],(self.BT_x[i]**2+self.BT_y[i]**2)]]
            expr=expr+G[i,i]-2*self.dis_[i]*G[num,i]+self.dis_[i]**2
            constraints += [G[i, i] == cp.trace(C_i@X)]
        obj=cp.Minimize(expr)
        prob=cp.Problem(obj,constraints)
        prob.solve()
        #print("status",prob.status)
        #print("optimal value",prob.value)
        #print("optimal var",X.value,G.value)
        position = [round(X.value[0,2],1), round(X.value[1,2], 1)]
        return position

#-----------------------------------------------------NewTon-Gaussian

    def Grad(self,f,x1,x2,x_i,y_i):
        f1=diff(f,x1)
        f2=diff(f,x2)
        grad=np.array([[f1.subs([(x1,x_i),(x2,y_i)])],[f2.subs([(x1,x_i),(x2,y_i)])]])
        return grad

    def Hess(self,f,x1,x2,x_i,y_i):
        f1=diff(f,x1)
        f2=diff(f,x2)
        f11,f12,f21,f22=diff(f1,x1),diff(f1,x2),diff(f2,x1),diff(f2,x2)
        hess=np.array([[f11.subs([(x1,x_i),(x2,y_i)]),f12.subs([(x1,x_i),(x2,y_i)])],
                       [f21.subs([(x1,x_i),(x2,y_i)]),f22.subs([(x1,x_i),(x2,y_i)])]])
        hess = np.array(hess, dtype='float')
        return hess

    def NewTon_Gauss(self):
        err,count=0.1,0
        X=self.init_value()
        x_k,x_k1=np.array([[X[0]],[X[1]]]),np.array([[0],[0]])
        x_p,y_p=symbols('x,y')
        target_fun=sum(list(map(lambda d,bt_x,bt_y:(d-((x_p-bt_x)**2+(y_p-bt_y)**2)**0.5)**2,self.dis_,self.BT_x,self.BT_y)))
        pos=[]
        while True:
            #print(x_k)
            pos.append([round(x_k[0][0],1),round(x_k[1][0],1)])
            x_err=x_k-x_k1
            if x_err[0][0]**2+x_err[1][0]**2<=err**2:
                break
            grad = self.Grad(target_fun, x_p, y_p, x_k[0][0], x_k[1][0])
            hess = self.Hess(target_fun, x_p, y_p, x_k[0][0], x_k[1][0])
            if grad[0][0]**2+grad[1][0]**2<err*2:
                break
            x_k1=x_k
            x_k=x_k-(np.linalg.inv(hess)).dot(grad)
            count=count+1
            if count>10:
                break
        #print(pos)
        #print(len(pos))
        return pos

#------------------------------------------------------------------------画圆函数

    def plot_cicle(centers, rads, MT, BT_x, BT_y):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for center, rad in zip(centers, rads):
            cir = plt.Circle((center[0], center[1]), radius=rad, color='y', fill=False)
            ax.add_patch(cir)

        ax.plot(BT_x, BT_y, 'ro')
        ax.plot(MT[0], MT[1], 'ro')
        plt.axis('scaled')
        plt.axis('equal')
        plt.show()

def main():
    #print('hello LOS')
    BT_x, BT_y, Dis, MPE, dis_ = [], [], [], [], []
    ave,sigma=0,1
    MT=[400,300]
    for i in range(4):
        x = random.random() * 800
        BT_x.append(x)
        y = random.random() * 500
        BT_y.append(y)
        d = ((MT[0] - BT_x[i]) ** 2 + (MT[1] - BT_y[i]) ** 2) ** 0.5
        Dis.append(d)
        n = math.log(d) * random.gauss(ave,sigma)
        MPE.append(n)
        dis_.append(d + n)
    sdr=Program(MT,BT_x,BT_y,Dis,dis_)
    sdr.NewTon_Gauss()
    #sdr.test()
if __name__=="__main__":
    main()