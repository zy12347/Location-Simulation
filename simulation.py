import tkinter.messagebox as messagebox
from tkinter import *
import threading
import tkinter
import random
import time
import math
import PIL
import LOS


class simulation:

    def __init__(self, master=None):
        self.root = master
        self.root.geometry('1000x800')
        self.root.title('SIMULATION')
        self.para = {}
        self.para['MT_x'],self.para['MT_y'],self.para['MT']=500, 400,[500,400]#init value for convenience
        self.para['BT_x'], self.para['BT_y'] = [300, 700, 300, 700], [300, 300, 500,500]
        self.para['ave'],self.para['sigma'],self.para['Number']=0,2,4
        self.para['map']='map.png'
        self.Home()

    def getBT(self, e1, e2) :
        BT_x, BT_y, Dis, MPE, dis_ = [], [], [], [], []
        try:
            for i in range(len(e1)):
                x = float(e1[i].get())
                y = float(e2[i].get())
                BT_x.append(x)
                BT_y.append(y)
                d = ((self.para['MT_x'] - x) ** 2 + (self.para['MT_y'] - y) ** 2) ** 0.5
                Dis.append(d)
                n = math.log(d) * random.gauss(self.para['ave'], self.para['sigma'])
                MPE.append(n)
                dis_.append(d + n)

            self.para['BT_x'] = BT_x
            self.para['BT_y'] = BT_y
            self.para['Dis'] = Dis
            self.para['MPE'] = MPE
            self.para['dis_'] = dis_

            #print(self.para)
            self.Home()
            #self.PlotStation()
        except:
            messagebox.showwarning("warning!")

    def getMT(self, e1, e2, e3):
        try:
            self.para['MT_x'] = float(e1.get())
            self.para['MT_y'] = float(e2.get())
            self.para['MT'] = [float(e1.get()), float(e2.get())]
            self.para['Number'] = int(e3.get())
            #print(self.para)
            self.input()
        except:
            messagebox.showwarning("warning!")

    def nois(self,e1,e2):
        try:
            self.para['ave'] = float(e1.get())
            self.para['sigma'] = float(e2.get())
            #print(self.para)
        except:
            messagebox.showwarning("warning!")

    def MT_pos(self,e1,e2):
        try:
            self.para['MT_x'] = float(e1.get())
            self.para['MT_y'] = float(e2.get())
            quote = 'True MT position is :  ' + '[' + str(self.para['MT_x']) + ',' + str(self.para['MT_y']) + ']'
            Label(self.frame3, text=quote, bg='#EEE8AA', justify='left').place(in_=self.frame3, anchor=NW, x=300, y=30,
                                                                               width=300)

            #print(self.para)
        except:
            messagebox.showwarning("warning!")

    def GetNum(self,e1):
        try:
            self.para['Number'] = int(e1.get())
            #print(self.para)
        except:
            messagebox.showwarning("warning!")

    def drawbase(self):
        self.frame1.img_bg = PIL.ImageTk.PhotoImage(PIL.Image.open(self.para['map']).resize((1000, 800)))
        mt_png = self.frame1.create_image((0, 0), anchor='nw')
        self.frame1.itemconfig(mt_png, image=self.frame1.img_bg)

        self.frame1.img_mt = PIL.ImageTk.PhotoImage(PIL.Image.open('MT.png').resize((30, 30)))
        mt_png = self.frame1.create_image((self.para['MT_x'], self.para['MT_y']), anchor='nw')
        self.frame1.itemconfig(mt_png, image=self.frame1.img_mt)  # 内存保护机制，自动销毁图片内存，需要将图片绑定到canavas本身
        self.frame1.img_bt = PIL.ImageTk.PhotoImage(PIL.Image.open('BT.png').resize((30, 30)))
        for x, y in zip(self.para['BT_x'], self.para['BT_y']):
            bt_png = self.frame1.create_image((x, y), anchor='nw')
            self.frame1.itemconfig(bt_png, image=self.frame1.img_bt)

    def Circle(self):
        self.frame1.delete(tkinter.ALL)
        BT_x, BT_y, Dis, MPE, dis_ = [], [], [], [], []
        for i in range(self.para['Number']):
            x = self.para['MT_x'] + math.sin(2 * math.pi / self.para['Number'] * i) * min(self.para['MT_x'],
                                                                                          900 - self.para[
                                                                                              'MT_y']) * 0.75
            BT_x.append(x)
            y = self.para['MT_y'] + math.cos(2 * math.pi / self.para['Number'] * i) * min(self.para['MT_y'],
                                                                                          700 - self.para[
                                                                                              'MT_y']) * 0.75
            BT_y.append(y)
            d = ((self.para['MT_x'] - x) ** 2 + (self.para['MT_y'] - y) ** 2) ** 0.5
            Dis.append(d)
            n = math.log(d) * random.gauss(self.para['ave'], self.para['sigma'])
            MPE.append(n)
            dis_.append(d + n)

        self.para['BT_x'] = BT_x
        self.para['BT_y'] = BT_y
        self.para['Dis'] = Dis
        self.para['MPE'] = MPE
        self.para['dis_'] = dis_
        self.drawbase()

    def Randd(self):
        self.frame1.delete(tkinter.ALL)
        BT_x, BT_y, Dis, MPE, dis_ = [], [], [], [], []
        for i in range(self.para['Number']):
            x = random.random()*900
            BT_x.append(x)
            y = random.random()*700
            BT_y.append(y)
            d = ((self.para['MT_x'] - x) ** 2 + (self.para['MT_y'] - y) ** 2) ** 0.5
            Dis.append(d)
            n = math.log(d) * random.gauss(self.para['ave'], self.para['sigma'])
            MPE.append(n)
            dis_.append(d + n)
        self.para['BT_x'] = BT_x
        self.para['BT_y'] = BT_y
        self.para['Dis'] = Dis
        self.para['MPE'] = MPE
        self.para['dis_'] = dis_
        self.drawbase()

    def Set_XJTU(self):
        self.para['map']='map1.png'
        self.Home()

    def Set_Cartoon(self):
        self.para['map']='map.png'
        self.Home()

    def Set_World(self):
        self.para['map']='world.png'
        self.Home()

    def Home(self):
        menu = Menu(self.root)
        self.root.config(menu=menu)

        scenario = Menu(menu)
        menu.add_cascade(label='Scenario', menu=scenario)
        scenario.add_command(label='Home', command=self.Home)

        draw=Menu(menu)
        menu.add_cascade(label='Draw', menu=draw)
        draw.add_command(label='Circle', command=self.Circle)
        draw.add_command(label='Random', command=self.Randd)
        draw.add_command(label='Input', command=self.input)

        method=Menu(menu)

        ml=Menu(method)
        ml.add_command(label='AML',command=self.AML)
        ml.add_command(label='Newton_Gauss', command=self.NewTon_Gauss)

        ls = Menu(method)
        ls.add_command(label='LLS_1',command=self.LLS_1)
        ls.add_command(label='LLS_AVE', command=self.LLS_AVER)
        ls.add_command(label='LLS_AVE', command=self.LLS_RS)

        sdr = Menu(method)
        sdr.add_command(label='R_LS', command=self.R_LS)

        menu.add_cascade(label='Method', menu=method)
        method.add_cascade(label='ML',menu=ml)
        method.add_cascade(label='LS',menu=ls)
        method.add_cascade(label='SDR',menu=sdr)

        para=Menu(menu)
        menu.add_cascade(label='Para', menu=para)
        para.add_command(label='set_para',command=self.Set_Para)

        map = Menu(menu)
        menu.add_cascade(label='Map', menu=map)
        map.add_command(label='XJTU', command=self.Set_XJTU)
        map.add_command(label='Cartoon', command=self.Set_Cartoon)
        map.add_command(label='World', command=self.Set_World)


        self.frame1 = Canvas(self.root)

        self.frame1.config(bg='white', height=800, width=1000)
        Label(self.frame1, text='frame1').place(in_=self.frame1, anchor=NW)
        self.frame1.place(x=0, y=0)
        self.frame1.img_bg = PIL.ImageTk.PhotoImage(PIL.Image.open(self.para['map']).resize((1000, 800)))
        mt_png = self.frame1.create_image((0, 0), anchor='nw')
        self.frame1.itemconfig(mt_png, image=self.frame1.img_bg)


    def Set_Para(self):
        self.para_frame = Frame(self.root)
        self.para_frame.config(bg='#e8c974', height=200, width=200)
        Label(self.para_frame, text='para').place(in_=self.para_frame, anchor=NW)
        self.para_frame.place(x=400, y=200)
        Number = Entry(self.para_frame)
        Number.place(in_=self.para_frame, anchor=NW, x=20, y=40, width=85)
        Button(self.para_frame, text='BT Num', command=lambda: self.GetNum(Number)).place(in_=self.para_frame, anchor=NW, x=120,y=40,width=50)
        Ave = Entry(self.para_frame)
        Ave.place(in_=self.para_frame, anchor=NW, x=20, y=80, width=40)
        Sigma = Entry(self.para_frame)
        Sigma.place(in_=self.para_frame, anchor=NW, x=65, y=80, width=40)
        Button(self.para_frame, text='Gauss', command=lambda: self.nois(Ave, Sigma)).place(in_=self.para_frame, anchor=NW,x=120,y=80, width=50)
        MT_x = Entry(self.para_frame)
        MT_x.place(in_=self.para_frame, anchor=NW, x=20, y=120, width=40)
        MT_y = Entry(self.para_frame)
        MT_y.place(in_=self.para_frame, anchor=NW, x=65, y=120, width=40)
        Button(self.para_frame, text='MT pos', command=lambda: self.MT_pos(MT_x, MT_y)).place(in_=self.para_frame, anchor=NW,x=120, y=120, width=50)
        Button(self.para_frame, bg='#f5ce42', text='Home', command=self.Home).place(in_=self.para_frame, anchor=NW, x=60, y=160, width=80)

    def input(self):
        number = int(self.para['Number'])
        self.frame2_1 = Frame(self.root)

        self.frame2_1.config(bg='white', height=800, width=1000)
        Label(self.frame2_1, text='frame1').place(in_=self.frame2_1, anchor=NW)
        self.frame2_1.place(x=0, y=0)
        E1_x, E1_y = [], []
        skip = int(400 / number)
        for i in range(number):
            e_t1 = Entry(self.frame2_1, bg='#f5ce42')
            e_t1.place(in_=self.frame2_1, anchor=NW, x=400, y=160 + i * skip, width=50)
            e_t2 = Entry(self.frame2_1, bg='#f5ce42')
            e_t2.place(in_=self.frame2_1, anchor=NW, x=500, y=160 + i * skip, width=50)
            E1_x.append(e_t1)
            E1_y.append(e_t2)
        Button(self.frame2_1, bg='#f5ce42', text='Confirm', command=lambda: self.getBT(E1_x, E1_y)).place(
            in_=self.frame2_1, anchor=NW, x=600, y=400, width=100)
        Button(self.frame2_1, bg='#f5ce42', text='Home', command=self.Home).place(
            in_=self.frame2_1, anchor=NW, x=600, y=300, width=100)

    def PlotStation(self):
        self.frame1.create_rectangle(self.para['MT_x'] - 5, self.para['MT_y'] - 5, self.para['MT_x'] + 5,
                                     self.para['MT_y'] + 5, fill='#a83e32')
        for x, y in zip(self.para['BT_x'], self.para['BT_y']):
            self.frame1.create_rectangle(x - 5, y - 5, x + 5, y + 5, fill='#71a832')

    def draw_espos(self,es_pos):
        self.frame1.img_es_aml = PIL.ImageTk.PhotoImage(PIL.Image.open('estimate.png').resize((30, 30)))
        mt_png = self.frame1.create_image((es_pos[0], es_pos[1]), anchor='nw')
        self.frame1.itemconfig(mt_png, image=self.frame1.img_es_aml)
        self.frame1.create_text((es_pos[0] + 10, es_pos[1] - 10), anchor='nw',
                                text='[' + str(es_pos[0]) + ',' + str(es_pos[1]) + ']', fill='#f20242')

    def AML(self):
        self.frame1.delete(tkinter.ALL)
        self.drawbase()
        aml = LOS.Program(self.para['MT'], self.para['BT_x'], self.para['BT_y'], self.para['Dis'], self.para['dis_'])
        es_pos = aml.estimate()
        self.draw_espos(es_pos)

    def LLS_1(self):
        self.frame1.delete(tkinter.ALL)
        self.drawbase()
        lls1 = LOS.Program(self.para['MT'], self.para['BT_x'], self.para['BT_y'], self.para['Dis'], self.para['dis_'])
        es_pos = lls1.LLS_1_E()
        self.draw_espos(es_pos)

    def LLS_AVER(self):
        self.frame1.delete(tkinter.ALL)
        self.drawbase()
        lls_aver = LOS.Program(self.para['MT'], self.para['BT_x'], self.para['BT_y'], self.para['Dis'], self.para['dis_'])
        es_pos = lls_aver.LLS_AVE_E()
        self.draw_espos(es_pos)

    def LLS_RS(self):
        self.frame1.delete(tkinter.ALL)
        self.drawbase()
        lls_rs = LOS.Program(self.para['MT'], self.para['BT_x'], self.para['BT_y'], self.para['Dis'], self.para['dis_'])
        es_pos = lls_rs.LLS_RS_E()
        self.draw_espos(es_pos)

    def R_LS(self):
        self.frame1.delete(tkinter.ALL)
        self.drawbase()
        s_ls = LOS.Program(self.para['MT'], self.para['BT_x'], self.para['BT_y'], self.para['Dis'], self.para['dis_'])
        es_pos = s_ls.R_LS_E()
        self.draw_espos(es_pos)

    def NewTon_Gauss(self):
        self.frame1.delete(tkinter.ALL)
        self.drawbase()
        N_G = LOS.Program(self.para['MT'], self.para['BT_x'], self.para['BT_y'], self.para['Dis'], self.para['dis_'])
        es_pos = N_G.NewTon_Gauss()
        length=len(es_pos)
        self.frame1.img_es_gn = PIL.ImageTk.PhotoImage(PIL.Image.open('estimate.png').resize((30, 30)))
        for i in range(length-1):
            mt_png = self.frame1.create_image((es_pos[i][0], es_pos[i][1]), anchor='nw')
            self.frame1.itemconfig(mt_png, image=self.frame1.img_es_gn)
            time.sleep(1)
            self.frame1.delete(mt_png)
        mt_png = self.frame1.create_image((es_pos[length-1][0], es_pos[length-1][1]), anchor='nw')
        self.frame1.itemconfig(mt_png, image=self.frame1.img_es_gn)
        self.frame1.create_text((es_pos[length-1][0] + 10, es_pos[length-1][1] - 10), anchor='nw',
                                text='[' + str(es_pos[length-1][0]) + ',' + str(es_pos[length-1][1]) + ']', fill='#f20242')

    def newton_gaussian(self):
        T=threading.Thread(target=self.NewTon_Gauss)
        T.start()

def main():
    root = Tk()
    s=simulation(root)
    mainloop()


if __name__ == "__main__":
    main()
