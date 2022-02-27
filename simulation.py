import tkinter.messagebox as messagebox
from tkinter import *
import tkinter
import random
import math
import TOA

class simulation:
    def __init__(self,master=None):
        self.root=master
        self.root.geometry('1000x800')
        self.root.title('SIMULATION')
        self.para={}
        self.Home()

    def getBT(self,e1,e2):
        #try:
        for i in range(len(e1)):
            self.para['BT_x'+str(i)]=int(e1[i].get())
            self.para['BT_y'+str(i)]=int(e2[i].get())
        print(self.para)
        self.Home()
        #except:
        #    messagebox.showwarning("warning!please input digit!")

    def getMT(self,e1,e2,e3):
        #try:
        self.para['MT_x']=int(e1.get())
        self.para['MT_y']=int(e2.get())
        self.para['Number']=int(e3.get())
        print(self.para)
        self.input()
        #except:
        #    messagebox.showwarning("warning!please input digit!")

    def Home(self):
        menu = Menu(self.root)
        self.root.config(menu=menu)

        filemenu = Menu(menu)
        menu.add_cascade(label='TOA', menu=filemenu)
        filemenu.add_command(label='Home', command=self.Home)

        self.frame1 = Canvas(self.root)
        self.frame2 = Frame(self.root)
        self.frame3 = Frame(self.root)

        self.frame1.config(bg='white', height=500, width=800)
        Label(self.frame1, text='frame1').place(in_=self.frame1, anchor=NW)
        self.frame1.place(x=0, y=0)
        E1_x, E1_y = [], []
        oval=self.frame1.create_oval(100, 10, 410, 200, outline="red", fill="#adf123")

        self.frame2.config(bg='#334353', height=500, width=200)
        Label(self.frame2, text='frame2').place(in_=self.frame2, anchor=NW)
        self.frame2.place(x=800, y=0)
        Number=Entry(self.frame2)
        Number.place(in_=self.frame2,anchor=NW,x=25,y=40,width=50)
        MT_x = Entry(self.frame2)
        MT_x.place(in_=self.frame2,anchor=NW,x=25,y=80,width=50)
        MT_y = Entry(self.frame2)
        MT_y.place(in_=self.frame2, anchor=NW, x=80, y=80, width=50)
        Button(self.frame2,text='confirm',command=lambda: self.getMT(MT_x,MT_y,Number)).place(in_=self.frame2,anchor=NW,x=80,y=120)
        #Button(self.frame2, text='AML', command=lambda: self.AML()).place(in_=self.frame2, anchor=NW, x=50, y=100,width=100)
        #Button(self.frame2, text='AML', command=lambda: self.AML()).place(in_=self.frame2, anchor=NW, x=50, y=200,width=100)

        self.frame3.config(bg='#EEE8AA', height=300, width=1000)
        Label(self.frame3, text='frame3').place(in_=self.frame3, anchor=NW)
        self.frame3.place(x=0, y=500)

    def input(self):
        number=int(self.para['Number'])
        self.frame2_1 = Frame(self.root)

        self.frame2_1.config(bg='white', height=800, width=1000)
        Label(self.frame2_1, text='frame1').place(in_=self.frame2_1, anchor=NW)
        self.frame2_1.place(x=0, y=0)
        E1_x,E1_y=[],[]
        skip = int(400 / number)
        for i in range(number):
            e_t1 = Entry(self.frame2_1, bg='#f5ce42')
            e_t1.place(in_=self.frame2_1, anchor=NW, x=400, y=160 + i * skip, width=50)
            e_t2 = Entry(self.frame2_1, bg='#f5ce42')
            e_t2.place(in_=self.frame2_1, anchor=NW, x=500, y=160 + i * skip, width=50)
            E1_x.append(e_t1)
            E1_y.append(e_t2)
        Button(self.frame2_1, bg='#f5ce42', text='confirm', command=lambda: self.getBT(E1_x, E1_y)).place(in_=self.frame2_1, anchor=NW, x=600, y=400,width=100)

    def AML(self):
        aml=TOA.AML()
        aml.estimate()

def GenData(number,ave,sigma):
    BT = []
    for i in range(number):
        BT.append([random.random() * 100, random.random() * 100])
    MT = [random.random() * 100, random.random() * 100]
    BT_x = [x[0] for x in BT]
    BT_y = [x[1] for x in BT]
    print("MT:", MT)
    dis_, Distance, MPE = [], [], []
    # true distance
    for x, y in BT:
        Distance.append(((MT[0] - x) ** 2 + (MT[1] - y) ** 2) ** 0.5)
    # noise
    for d in Distance:
        MPE.append(math.log(d) * random.gauss(ave, sigma))
    # estimate distance
    for d, n in zip(Distance, MPE):
        dis_.append(d + n)

    return BT,MT,BT_x,BT_y,Distance,dis_

def main():
    root=Tk()
    simulation(root)
    #bt, mt, bt_x, bt_y, Distance, Dis_=GenData(5,0,1)
    #aml=TOA.AML(bt,mt,bt_x,bt_y,Distance,Dis_)
    #aml.estimate()
    mainloop()

if __name__=="__main__":
    main()
