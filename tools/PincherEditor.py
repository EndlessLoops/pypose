#!/usr/bin/env python
'''
@author: Phil Williammee
Description: This is a  PyPose tool for phantomx Pincher arm 5dof 
robotic arm using kinematic positioning, matplotlib simulation, wx Frame
@version: PhantomX.0.5, 6-5-2013
Python 2.7.3-2(32 bit)
wx 2.8 msw -unicode
matplotlib 1.2.1

'''

import sys, time, os, wx, project
# The recommended way to use wx with mpl is with the WXAgg backend. 
import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigCanvas, \
    NavigationToolbar2WxAgg as NavigationToolbar
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from driver import *
from ax12 import *
from ToolPane import *

class PincherEditor(ToolPane):
    """ phantomx robotic pincher arm 5dof simulator """
    """ editor for the capture and creation of poses. """
    BT_DELTA_T = wx.NewId()
    BT_RELAX = wx.NewId()
    BT_RELAX_ID = wx.NewId()
    BT_CAPTURE = wx.NewId()
    BT_SET = wx.NewId()
    BT_POSE_ADD = wx.NewId()
    BT_POSE_ADV = wx.NewId()
    BT_POSE_REM = wx.NewId()
    BT_POSE_RENAME = wx.NewId()
    ID_POSE_BOX = wx.NewId()
    BT_LIVE = wx.NewId()
    
    def __init__(self, parent, port=None):  
        ToolPane.__init__(self, parent, port)
        '''project variables'''
        self.curpose = "" #initial name of pose
        self.saveReq = False
    
        self.go_live = False
        
        #@todo this she be held in project, check to make sure project variables = 5
        #phantom x only uses 5 dynamixels
        self.parent.project.count = 5
        self.dynamixels = [0]*5#holds the values of the dynamixels (0-1024)
        self.oldmixels = [0]*self.parent.project.count
                
        #should handle this in parent 
        self.parent.Move((0,0))#open window in upper left corner 

        self.figure = Figure() #create a matplot figure        
        self.canvas = FigCanvas(self, -1, self.figure)
        # You must set up the canvas before creating the 3D axes
        self.axes = Axes3D(self.figure)
        self.axes.azim = 150 #initial azimuth view is looking at home position in degrees
        
        # slider settings
        self.slider_SP = [150,60,80,-45,512]#starting position a0,w,z,ga,g
        self.slider_minL = [0,-20,-50,-90,0]
        self.slider_maxL = [300,280,280,90,1023]
        self.label_list = ["Rotation a0:  ","Extension w:  ","Height z:        ",
                           "Gripper angle ","Gripper:         "]  
        
        self.datagen = Robot_Cord(self.slider_SP)#data gen object handles positioning of servo,(x,y,z)...ect
        self.draw_plot()#draw the plot at the top of the frame
        
        #handy timer is there a app timer?
        self.t1 = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTest1Timer)
        
        self.box = wx.BoxSizer(wx.VERTICAL) #a box to add sliders too
        self.box.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        
        sizer = wx.GridBagSizer(5,5)# main sizer edit,list,buttons
        temp = wx.StaticBox(self, wx.NewId(), 'edit Picher arm')
        temp.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        
        editBox = wx.StaticBoxSizer(temp,orient=wx.VERTICAL| wx.GROW) 
        poseEditSizer = wx.GridBagSizer(0,0)# edit pose sizer box
        poseEditSizer.AddGrowableCol(0)#allow sliders to expand in column 1
        
        # a box of slider panels
        self.sliders = list() # the editors in the window 
        for i in range(self.parent.project.count):
            temp = wx.Panel(self,-1)
            hbox = wx.BoxSizer(wx.HORIZONTAL)
            temp.text = wx.StaticText(temp, i, self.label_list[i])#name is i
            temp.position = wx.Slider(temp, i, self.slider_SP[i], self.slider_minL[i], 
                                      self.slider_maxL[i], wx.DefaultPosition, (325, -1), 
                                      wx.SL_HORIZONTAL | wx.SL_LABELS|wx.EXPAND|wx.GROW)
            #put the panel in the sizer
            hbox.Add(temp.text,flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL|
                     wx.ALIGN_RIGHT )
            hbox.Add(temp.position,flag=wx.EXPAND|wx.ALL|wx.ALIGN_LEFT|wx.GROW)
            temp.SetSizerAndFit(hbox)
            poseEditSizer.Add(temp, (i,0),span=(1,1),flag=wx.EXPAND|wx.ALL|
                            wx.ALIGN_LEFT|wx.GROW, border=2) 
            temp.Disable()  # servo editors start out disabled, enabled when a pose is selected
            self.sliders.append(temp)#build a list of servo panels
        
        self.servos = self.datagen.sliders_to_dynamixels(self.sliders)
        
        # grid it
        editBox.Add(poseEditSizer)
        sizer.Add(editBox, (0,0), wx.GBSpan(1,1), wx.EXPAND|wx.GROW)
        
        # list of poses
        self.posebox = wx.ListBox(self, self.ID_POSE_BOX, choices=self.parent.project.poses.keys())
        sizer.Add(self.posebox, pos=(0,1), span=(1,1),flag=wx.EXPAND|wx.ALL)
        
        # and buttons
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.Button(self, self.BT_POSE_ADD, 'add'))
        hbox.Add(wx.Button(self, self.BT_POSE_REM, 'remove'))   
        hbox.Add(wx.Button(self, self.BT_POSE_RENAME, 'rename')) 
        sizer.Add(hbox,(1,1),wx.GBSpan(1,1),wx.ALIGN_CENTER)  

        # toolbar
        toolbar = wx.Panel(self, -1)
        toolbarsizer = wx.BoxSizer(wx.HORIZONTAL) 
        self.toggle_live = wx.ToggleButton(toolbar, self.BT_LIVE, "live")   
        toolbarsizer.Add(self.toggle_live,1)    
        
        #  delta-T for interpolation
        self.deltaTButton = wx.Button(toolbar, self.BT_DELTA_T, 'delta-T')   
        self.deltaTButton.Disable()     
        toolbarsizer.Add(self.deltaTButton,1)
        self.deltaT = 2000
        if port != None and port.hasInterpolation == True:        
            self.deltaTButton.Enable()
            
        #add buttons to the sizer
        toolbarsizer.Add(wx.Button(toolbar, self.BT_RELAX, 'relax'),1)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_CAPTURE, 'capture'),1)         
        toolbarsizer.Add(wx.Button(toolbar, self.BT_SET, 'set'),1)                
        toolbar.SetSizer(toolbarsizer)
        sizer.Add(toolbar, (1,0), wx.GBSpan(1,1), wx.ALIGN_CENTER)
        
        #bind slider
        self.Bind(wx.EVT_SLIDER, self.updatePose)
        
        self.Bind(wx.EVT_CHECKBOX, self.relaxServo)
        
        wx.EVT_BUTTON(self, self.BT_RELAX, self.parent.doRelax)    
        wx.EVT_BUTTON(self, self.BT_CAPTURE, self.capturePose) 
        wx.EVT_BUTTON(self, self.BT_SET, self.setPose)   
        wx.EVT_BUTTON(self, self.BT_POSE_ADD, self.addPose)   
        wx.EVT_BUTTON(self, self.BT_POSE_REM, self.remPose)   
        wx.EVT_BUTTON(self, self.BT_POSE_RENAME, self.renamePose)
        wx.EVT_BUTTON(self, self.BT_DELTA_T, self.doDeltaT)   
        wx.EVT_LISTBOX(self, self.ID_POSE_BOX, self.doPose)

        # key accelerators
        aTable = wx.AcceleratorTable([(wx.ACCEL_CTRL,  ord('S'), self.BT_CAPTURE),  # capture Servos
                                      (wx.ACCEL_CTRL, ord('R'), self.BT_RELAX),     # Relax
                                      (wx.ACCEL_CTRL, ord('A'), self.BT_POSE_ADV),  # Advance
                                     ])
        self.SetAcceleratorTable(aTable)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.live_bt, id=self.BT_LIVE)
        self.Bind(wx.EVT_MENU,self.capturePose,id=self.BT_CAPTURE)
        self.Bind(wx.EVT_MENU,self.parent.doRelax,id=self.BT_RELAX)
        self.Bind(wx.EVT_MENU,self.advancePose,id=self.BT_POSE_ADV)

        self.box.Add(sizer, 0, wx.ALL|wx.EXPAND|wx.GROW)#add the sizer to the box      
        self.SetSizerAndFit(self.box)#add the box to the panel
        self.box.Fit(self.parent)#expand to fit Frame
        
        #self.delay = False
    '''class functions and event handlers     ****    end of initialization     ******  '''          
    #draw the matplot simulation and a lot more        
    def draw_plot(self):
        my_coord_list = self.datagen.get_pos()     
        if self.datagen.l12 < (self.datagen.l[1]+self.datagen.l[2]):#check boundaries  
            self.dynamixels, self.error = self.datagen.get_dynamixel()
            if self.error == []:
                ''' draw the plot '''
                self.axes.cla() #clear the old axes
                self.set_ax()   #reset all of the axes limits
                self.axes.set_axisbelow(True) #send grid lines to the background  
                xs = (my_coord_list[0])
                ys = (my_coord_list[1])
                zs = (my_coord_list[2])
                self.plot_data= self.axes.plot(xs, ys, zs, 'o-', markersize=20, 
                                               markerfacecolor="orange", linewidth = 8, color="blue")
                self.canvas.draw()
                tool_point = np.array([xs[3],ys[3],zs[3]])
                self.parent.sb.SetStatusText("Servo positions: " + str(self.dynamixels) +
                                              "      Tool Point: " + str(np.round(tool_point, 2)))
            else:
                self.parent.sb.SetStatusText("Servo safety limits reached")
                
        else:
            self.parent.sb.SetStatusText("Target Position Can NOT Be Reached")            
        
    
    def set_ax(self):#ax panel set up
        self.axes.set_xlim3d(-200, 200)
        self.axes.set_ylim3d(-200, 200)
        self.axes.set_zlim3d(-5, 200)
        self.axes.set_xlabel("X axis")
        self.axes.set_ylabel('Y axis')
        self.axes.set_zlabel('Z axis')
        for j in self.axes.get_xticklabels() + self.axes.get_yticklabels(): #hide ticks
            j.set_visible(False)
        self.axes.set_axisbelow(True) #send grid lines to the background
   
    def updatePose(self, e=None):#slider event
        """ Save updates to a pose, do live update if neeeded. """
        id = e.GetId()
        if self.curpose != "": 
            #get the value of the slider that changed this should send position
            self.datagen.sliders_val[id]=e.GetInt()
            #@todo: return the coordinates to plot why store data in coordinate system 
            self.draw_plot()
            self.datagen.dynamixels_to_angles(self.dynamixels)
            
            if self.error == [] and self.datagen.l12 < (self.datagen.l[1]+self.datagen.l[2]):
                #save all of the new dynamixel positions not just one
                for servo in range(self.parent.project.count):
                    #save dynamixel values to the project
                    #print (str(self.curpose) + " saving " + str(self.dynamixels[servo ]) + "to position " + str(servo))
                    self.parent.project.poses[self.curpose][servo] = self.dynamixels[servo ]

                self.parent.project.save = True
                                   
    def OnTest1Timer(self, evt):
        #set up the speeds for live mode
        speeds = [a - b for a, b in zip(self.dynamixels, self.oldmixels)]
        #limit speed to 100
        speeds = [x if (-100< x <100) else 100 for x in speeds]
        if self.dynamixels != self.oldmixels and self.port != None:               
            #load all the servo values
            for i, pos in enumerate(self.dynamixels):
                self.port.setReg(i+1, P_GOAL_SPEED_L, [int(abs(speeds[i]))%256,int(abs(speeds[i]))>>8]) 
                self.port.setReg(i+1, P_GOAL_POSITION_L, [pos%256,pos>>8])
        self.oldmixels = self.dynamixels
                
    def live_bt(self, e=None):
        
        if self.go_live == False:
            self.go_live = True
            self.toggle_live.SetLabel("live ON")
            self.t1.Start(500)
        else:
            self.go_live = False
            self.toggle_live.SetLabel("live OFF")
            #put speeds back to 0
            for i in range(len(self.dynamixels)):
                self.port.setReg(i+1, P_GOAL_SPEED_L, [0%256,0>>8])
            self.t1.Stop()
            
    
    def relaxServo(self, e=None):
        """ Relax or enable a servo. """
        servo = e.GetId() + 1
        checked = e.IsChecked()
        if checked: 
            self.port.setReg(servo, P_TORQUE_ENABLE, [1])
        else:
            self.port.setReg(servo, P_TORQUE_ENABLE, [0])
            
    def setPose(self, e=None):
        #self.deltaT = 2000 # TODO: calculate this or add a button
        NUM_SERVOS = 5 
        """ Write a pose out to the robot. """
        if self.port != None:  
            print "Setting pose..."
            if self.port.hasInterpolation == True:  # lets do this smoothly!
                # set pose size -- IMPORTANT!
                self.port.execute(253, 7, [NUM_SERVOS])
                # load the pose to arbotix
                #self.port.execute(253, 8, [0] + project.extract(self.dynamixels))#these two are equivelent
                self.port.execute(253, 8, [0] + project.extract(self.parent.project.poses[self.curpose]))                 
                self.port.execute(253, 9, [0, self.deltaT%256,self.deltaT>>8,255,0,0])              
                self.port.execute(253, 10, list())
            else:
                self.parent.sb.SetStatusText("Please Select a Pose") 
        else:
            self.parent.sb.SetStatusText("No Port Open") 

    def capturePose(self, e=None):  
        """ Downloads the current pose from the robot to the GUI. """
        if self.port != None: 
            if self.curpose != "":   
                #print "Capturing pose..."
                errors = "could not read servos: "
                errCount = 0.0
                dlg = wx.ProgressDialog("capturing pose","this may take a few seconds, please wait...",self.parent.project.count + 1)
                dlg.Update(1)
                for servo in range(self.parent.project.count):
                    pos = self.port.getReg(servo+1,P_PRESENT_POSITION_L, 2)
                    if pos != -1 and len(pos) > 1:
                        #record the value to the servo 
                        self.servos[servo] = pos[0] + (pos[1]<<8)
                    else: 
                        errors = errors + str(servo+1) + ", "
                        errCount = errCount + 1.0
                    #save the captured values to the project
                    if self.curpose != "":                
                        self.parent.project.poses[self.curpose][servo] = self.servos[servo] 
                    val = servo+2
                    dlg.Update(val) 
                
                if errors != "could not read servos: ":
                    self.parent.sb.SetStatusText(errors[0:-2],0)   
                    # if we are failing a lot, raise the timeout
                    if errCount/self.parent.project.count > 0.1 and self.parent.port.ser.timeout < 10:
                        self.parent.port.ser.timeout = self.parent.port.ser.timeout * 2.0   
                        print "Raised timeout threshold to ", self.parent.port.ser.timeout
                else:
                    self.parent.sb.SetStatusText("captured pose!",0)
                    
                dlg.Destroy()
                self.parent.project.save = True
                
                #convert from dynamixel to slider values
                slider_val = self.datagen.dynamixels_to_sliders(self.servos)
                
                #update the sliders
                for servo in range(self.parent.project.count):
                    self.sliders[servo].position.SetValue(slider_val[servo])  
                
                #draw the new pose:
                self.datagen.sliders_val = slider_val
                self.draw_plot() 
            
            else:
                self.parent.sb.SetBackgroundColour('RED')
                self.parent.sb.SetStatusText("Please Select a Pose",0) 
                self.parent.timer.Start(20)       
        else:
            self.parent.sb.SetBackgroundColour('RED')
            self.parent.sb.SetStatusText("No Port Open",0) 
            self.parent.timer.Start(20)
        
    def advancePose(self, e=None):
        """ Create a new pose, with a default name. """
        if self.parent.project.name != "":
            i = 0
            while True:
                if "pose"+str(i) in self.parent.project.poses.keys():
                    i = i + 1
                else:
                    break
            # have name, create pose
            self.parent.project.poses["pose"+str(i)] = project.pose("",self.parent.project.count)
            self.posebox.Append("pose"+str(i))
            # select pose
            self.loadPose("pose"+str(i))
            self.posebox.SetSelection(self.posebox.FindString("pose"+str(i)))
    
    #click item in the posebox event   
     
    def loadPose(self, posename):
        if self.curpose == "":   # if we haven't yet, enable servo editors
            for slider_panel in self.sliders:
                slider_panel.Enable()  #enable the sliders      
        self.curpose = posename  
        #load the saved dynamixel values from project           
        for servo in range(self.parent.project.count):
            self.servos[servo] = self.parent.project.poses[self.curpose][servo]
        #convert them to slider values
        slider_val = self.datagen.dynamixels_to_sliders(self.servos)
        #update the sliders
        for servo in range(self.parent.project.count):
            self.sliders[servo].position.SetValue(slider_val[servo])          
        self.parent.sb.SetStatusText('now editing pose: ' + self.curpose,0)
        self.parent.project.save = True
        #draw the new pose:
        self.datagen.sliders_val = slider_val
        self.draw_plot()
            
    def doPose(self, e=None):
        """ Load a pose into the slider boxes. """
        if e.IsSelection():
            self.loadPose(str(e.GetString()))
            
    def addPose(self, e=None):
        """ Add a new pose to the listbox"""
        print ("adding a new pose")
        if self.parent.project.name != "":
            dlg = wx.TextEntryDialog(self,'Pose Name:', 'New Pose Settings')
            dlg.SetValue("")
            if dlg.ShowModal() == wx.ID_OK:
                self.posebox.Append(dlg.GetValue()) 
                self.parent.project.poses[dlg.GetValue()] = project.pose("",self.parent.project.count)
                dlg.Destroy()
            self.parent.project.save = True
        else:
            dlg = wx.MessageDialog(self, 'Please create a new robot first.', 'Error', wx.OK|wx.ICON_EXCLAMATION)
            dlg.ShowModal()
            dlg.Destroy()
               
    def renamePose(self, e=None):
        print ("renaming a pose")
        """ Rename a pose. """
        if self.curpose != "":
            dlg = wx.TextEntryDialog(self,'Name of pose:', 'Rename Pose')
            dlg.SetValue(self.curpose)
            if dlg.ShowModal() == wx.ID_OK:
                # rename in project data
                newName = dlg.GetValue()
                self.parent.project.poses[newName] = self.parent.project.poses[self.curpose]
                del self.parent.project.poses[self.curpose] 
                v = self.posebox.FindString(self.curpose)
                self.posebox.Delete(v)
                self.posebox.Insert(newName,v)
                self.posebox.SetSelection(v)
                self.curpose = newName
                self.parent.project.save = True

    def remPose(self, e=None):
        print ("removing a pose")
        """ Remove a pose. """
        if self.curpose != "":
            dlg = wx.MessageDialog(self, 'Are you sure you want to delete this pose?', 'Confirm', wx.OK|wx.CANCEL|wx.ICON_EXCLAMATION)
            if dlg.ShowModal() == wx.ID_OK:
                v = self.posebox.FindString(self.curpose)
                del self.parent.project.poses[self.curpose]
                self.posebox.Delete(v)
                self.curpose = ""
                dlg.Destroy()
                for servo in self.sliders:   # disable editors if we have no pose selected
                    servo.Disable()
            self.parent.sb.SetStatusText("please create or select a pose to edit...",0)
            self.parent.project.save = True   
    
    def doDeltaT(self, e=None):
        """ Adjust delta-T variable """
        dlg = wx.TextEntryDialog(self,'Enter time in mS:', 'Adjust Interpolation time')
        dlg.SetValue(str(self.deltaT))
        if dlg.ShowModal() == wx.ID_OK:
            print "Adjusting delta-T:" + str(dlg.GetValue())
            self.deltaT = int(dlg.GetValue())
            dlg.Destroy()

    def portUpdated(self):
        """ Adjust delta-T button """
        if self.port != None and self.port.hasInterpolation == True:        
            self.deltaTButton.Enable()
        else:
            self.deltaTButton.Disable()
            
'''
Created on May 30, 2013
@author: Phil Williammee
@todo: add a error check
description this class converts between sliders to dynamixals
and dynamixels to sliders
'''
class Robot_Cord(object):
    def __init__(self, init_slider=list([0]*5)):
        SEGMENTS = int(4) #number of phantomX segments
        SLIDER_HOME = init_slider
        SERVO_HOME = list()
      
        #initial settings of sliders, could pass these?
        self.tw = SLIDER_HOME[1] # w axis position depth
        self.tz = SLIDER_HOME[2]# z axis starting position height
        self.gripper_angle = SLIDER_HOME[3] #preselected gripper angles
        self.gripper = SLIDER_HOME[4]
  
        #variables used to calculate data
        self.l12 = 0.0 # hypotenuse belween a1 & a3
        self.a12 = 0.0 #inscribed angle between hypotenuse, w 
        self.l = np.array([0, 105, 105, 100])# actual measurements of segment length in mm
        self.w = np.array([0]*SEGMENTS,dtype=float) #horizontal coordinate
        self.z = np.array([0]*SEGMENTS,dtype=float) #vertical coordinate
        self.x = np.array([0]*SEGMENTS,dtype=float) #x axis components 
        self.y = np.array([0]*SEGMENTS,dtype=float) #y axis components
        self.a = np.array([SLIDER_HOME[0]]*SEGMENTS,dtype=float) #angle for the link, reference is previous link
        
        #used to load slider raw data directly into the function
        self.sliders_val = [self.a[0], self.tw, self.tz, self.gripper_angle, self.gripper]
    
    '''these functions convert slider variables to dynamixel and xyz coord'''   
    def sliders_to_var(self):#loads the variables needed to calculate pos.
        #print self.sliders_val
        self.a[0] = np.deg2rad(self.sliders_val[0])
        self.tw = self.sliders_val[1]
        self.tz = self.sliders_val[2]
        self.gripper_angle = self.sliders_val[3]
        self.gripper = self.sliders_val[4]
        
    def calc_p2(self):#calculates position 2
        self.w[3] = self.tw
        self.z[3] = self.tz
        self.w[2] = self.tw-np.cos(np.radians(self.gripper_angle))*self.l[3]
        self.z[2] = self.tz-np.sin(np.radians(self.gripper_angle))*self.l[3]
        self.l12 = np.sqrt(np.square(self.w[2])+np.square(self.z[2])) 
           
    def calc_p1(self):#calculate position 1
        self.a12 = np.arctan2(self.z[2],self.w[2])#return the appropriate quadrant  
        self.a[1] = np.arccos((np.square(self.l[1])+np.square(self.l12)-np.square(self.l[2])) 
                              /(2*self.l[1]*self.l12))+self.a12
        self.w[1] = np.cos(self.a[1])*self.l[1]
        self.z[1] = np.sin(self.a[1])*self.l[1]
    
    def calc_angles(self): #calculate all of the motor angles see diagram
        self.a[2] = np.arctan((self.z[2]-self.z[1])/(self.w[2]-self.w[1]))-self.a[1]
        self.a[3] = np.deg2rad(self.gripper_angle)-self.a[1]-self.a[2]  

    def calc_x_y(self):#calc x_y of servoscoordinates 
        for i in range(4):#fixed number of segments
            self.x[i] = self.w[i]*np.cos(self.a[0])
            self.y[i] = self.w[i]*np.sin(self.a[0])
             
    def calc_positions(self):#method to calculate data
        self.sliders_to_var()
        self.calc_p2() 
        self.calc_p1() 
        self.calc_x_y()
        self.calc_angles() 
        
    #calculate data should return this        
    def get_pos(self):
        self.calc_positions()#make sure data is current
        #convert arrays to lists for drawing the line
        xs = np.array(self.x).tolist()
        ys = np.array(self.y).tolist()
        zs = np.array(self.z).tolist()
        return (xs, ys, zs)
        
    def get_dynamixel(self):#returns a list of dynamixals positions and error
        try: 
            dynamixals = [0]*5 #fixed number of motors
            dynamixals[0]= int(round((self.a[0]*195.5696),0))
            dynamixals[1] = int(round(((self.a[1]-np.deg2rad(240))*-195.5696),0))#+60 degree
            dynamixals[2] = int(round(((self.a[2]-np.deg2rad(150))*-195.5696),0))#-150 degrees
            dynamixals[3] = int(round(((self.a[3]-np.deg2rad(150))*-195.5696),0))#-150 degrees
            dynamixals[4] = self.gripper
            #todo add a function to check for error, collision...ect
            if dynamixals[3]>860:
                return dynamixals, [865]
            error = [i for i in dynamixals if i>1024 or i<0] #safety limits
            return dynamixals, error
        
        except:
            print "**** ERROR CALCULATING DYNAMIXALS IS IT NULL! ****"
            error = [1111]
            dynamixals = []
            return dynamixals, error
        
    def sliders_to_dynamixels(self, sliders):
        for i in range(5):
            self.sliders_val[i] = sliders[i].position.GetValue()
    
        self.calc_positions()
        dynamixels, error = self.get_dynamixel()
        return dynamixels
      
    '''these functions start with dynamixel coor end up with slider'''  
    #calculating positions given servo values
    def dynamixels_to_angles(self, dynamixels):#converts dynamixels to angles
        angles = [0]*5 #angles are in radians
        angles[0] = dynamixels[0]*(np.deg2rad(300)/1024)
        angles[1] = (dynamixels[1]*-(np.deg2rad(300)/1024))+np.deg2rad(240)
        angles[2] = (dynamixels[2]*-(np.deg2rad(300)/1024))+np.deg2rad(150)
        angles[3] = (dynamixels[3]*-(np.deg2rad(300)/1024))+np.deg2rad(150)
        angles[4] = dynamixels[4]#this is gripper open,close not an angle
        return angles
        
    def angles_to_sliders(self, angles):    
        gripper_angle = angles[1]+angles[2]+angles[3]
        #w1 = self.l[1]*np.cos(angles[1])
        #z1 = self.l[1]*np.sin(angles[1])
        #law of cosines to find third side l12
        l12 = np.sqrt((self.l[1]*self.l[1])+(self.l[2]*self.l[2])
                      -(2*self.l[1]*self.l[2]*np.cos(np.pi-angles[2])))
        #law of cosines to find second angle sigma
        sigma = np.arccos(((self.l[1]*self.l[1])+(l12*l12)
                           -(self.l[2]*self.l[2])) / (2*self.l[1]*l12))
        a12 = angles[1]-sigma
        w2 = l12*np.cos(a12)
        z2 = l12*np.sin(a12)
        wt = (self.l[3]*np.cos(gripper_angle))+w2
        zt = (self.l[3]*np.sin(gripper_angle))+z2
        sliders = [np.rad2deg(angles[0]), wt, zt, np.rad2deg(gripper_angle ), angles[4] ]
        int_sliders = [int(round(slide,0)) for slide in sliders]
        return int_sliders
    
    def dynamixels_to_sliders(self, dynamixels):
        angles = self.dynamixels_to_angles(dynamixels)
        return self.angles_to_sliders(angles)#return slider values
        
        
NAME = "pincher editor"
STATUS = "opened pincher editor..."
