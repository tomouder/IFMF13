# -*- coding: utf-8 -*-
"""
Created on Mon Mar 29 20:54:43 2021

@author: matth
"""
import PySimpleGUI as sg
import serial
import subprocess

def main():
    #Identify the theme to use
    sg.theme("SystemDefaultForReal")

    # Define the layout for the GUI
    layout = [
        [sg.Frame('Global Coordinate Position (X and Y Based on Actuator Direction)',[[ 
         sg.Text("Global X-Coordinate: ###### [mm]", size=(32, 2), 
                 justification='right', key='xpos', font= ('hellvetica',11)), 
         sg.Text("Global Y-Coordinate: ###### [mm]",font = ("hellvetica",11), 
                 size=(32, 2), justification = 'left',key='ypos')]])],
        
        [sg.Frame('Local Coordinate Position (From Relative Datum)',[[
         sg.Text('Local X-Coordinate: ###### [mm]',font= ('hellvetica',11), size=(32,2), 
                 justification = "right",key='localxpos'),
         sg.Text('Local Y-Coordinate: ###### [mm]', size=(32,2), font= ('hellvetica',11),
                 justification = "left",key='localypos')]])],
        
        [sg.Text("Please Zero Actuators", font=('hellvetica',15), 
                 size=(55,1), background_color='#F7F3EC', justification='center',key='stat',relief=sg.RELIEF_SUNKEN)],
        
        [sg.Button("Zero Actuators")],
        [sg.Button("Enter Coordinate Control Mode", disabled=True,key='toggle'
                   )],
        
        [sg.Button('Zero Local X', key='xzer',disabled=False),
         sg.Button('Zero Local Y', key='yzer',disabled=False,)],

        [sg.Text('Desired X-Coordinate Position Change [mm]',visible=False,key='xchange'),
         sg.InputText(visible=False,key='xin')],
        
        [sg.Text('Desired Y-Coordinate Position Change [mm]',visible=False,key='ychange'),
         sg.InputText(visible=False,key='yin')]

    ]

    # Open the Amscope software fullscreen for image view and processing if
    # desired
    cmd = "C:\\Program Files\\AmScope\\Amscope\\x64\\amscope.exe"
    subprocess.Popen(cmd, stdout=subprocess.PIPE, creationflags=0x08000000)


    # Create the window and show it without the plot
    window = sg.Window("Inverted Fluorescence Microscope Control", layout,
                       location=(768,0), size=(768,432), resizable=True).Finalize()
    # Initialize the state to zero
    state = 0
    
    #Initiate finite state machine while loop. Does not break until the window
    #is closed.

    while True:
        #State 0 runs once and never again.
        if state == 0:
            #Initialize Modetoggle flag to True
            InJoy = True
            xzeroed = False
            yzeroed = False
            readcoord = False
            #Open the Serial port to communicate with the arduino                  
            ser = serial.Serial(port='COM3', baudrate=9600)
            #Read what the arduino code says into the status bar
            window['stat'].update('Zero Actuators to Begin')
            state = 1   #Transition to state 1 on the next pass through main

        elif state == 1: #Communication
            event, values = window.read(timeout=20) #Decide if event occurred.
            
            #if the window is closed tell the arduino to transiion back into 
            #its init state
            if event == sg.WIN_CLOSED:              
                ser.write(b'INIT')  #Trigger to tell program arduino to reset
                break               #Break the mainloop
                
            elif event == 'Zero Actuators': #Zero button was pushed
                state = 2
            
            elif event == 'Enter Coordinate Control Mode':
                state = 3
            elif event == 'xzer':
                xdatum = xcoord
                window['localxpos'].update('Local X-Coordinate: 0.000 [mm]')
                xzeroed = True
            elif event == 'yzer':
                ydatum = ycoord
                window['localypos'].update('Local Y-Coordinate 0.000 [mm]')
                yzeroed = True
                
                
            if readcoord == True:
                if ser.inWaiting() > 0:
                    reading = ser.readline().decode('ascii')
                    reading = reading.rstrip()
                    if reading == 'Itsx':
                        xcoord = (ser.readline().decode('ascii'))
                        xcoord = (float(xcoord.rstrip())/1000)
                        xcoord = "{:.3f}".format(xcoord)
                        window['xpos'].update(f'Global X-Coordinate: {xcoord} [mm]')
                        if xzeroed == True:
                            Localx = float(xcoord)-float(xdatum)
                            Localx = "{:.3f}".format(Localx)  
                        else:
                            Localx = xcoord
                        window['localxpos'].update(f'Local X-Coordinate: {Localx} [mm]')
                        
                    elif reading == 'Itsy':
                        ycoord = (ser.readline().decode('ascii')) 
                        ycoord = (float(ycoord.rstrip())/1000)
                        ycoord = "{:.3f}".format(ycoord)
                        window['ypos'].update(f'Global Y-Coordinate: {ycoord} [mm]')
                        if yzeroed == True:
                            Localy = float(ycoord)-float(ydatum)
                            Localy = "{:.3f}".format(Localy)
                        else:
                            Localy = ycoord
                        window['localypos'].update(f'Local Y-Coordinate: {Localy} [mm]')

            elif ser.inWaiting() > 0:
                reading = ser.readline().decode('ascii')
                reading = reading.rstrip()
                if reading == "ready":
                    readcoord = True
                    window['stat'].update('In Joystick Control Mode')
                    window['toggle'].update(disabled=False)
                    window['xzer'].update(disabled=False)
                    window['yzer'].update(disabled=False)
                else:
                    ser.reset_input_buffer()
            

        elif state == 2: # Zero Actuator State
            #Disable the Enter Coordinate Control Mode until zeroing 
            #process is completed
            window["toggle"].update(disabled=True)
            #write a zero to the actuator to tell it to transition states
            ser.write(b'wakeup')  
            window["Zero Actuators"].update(disabled=True)
            window['stat'].update('Zeroing Encoders...')
            state = 1
            
        elif state == 3: #Mode Toggle State
            ser.write(b'Toggle')            
            if InJoy == True:

                window["toggle"].update('Enter Joystick Control Mode')
                window["stat"].update('In Coordinate Control Mode')
                InJoy = False
                window['xchange'].update(visible=True)
                window['xin'].update(visible=True)
                window['ychange'].update(visible=True)
                window['yin'].update(visible=True)
            else:
                window["toggle"].update('Enter Coordinate Control Mode')
                window["stat"].update('In Joystick Control Mode')
                window['xchange'].update(visible=False)
                window['xin'].update(visible=False)
                window['ychange'].update(visible=False)
                window['yin'].update(visible=False)
                InJoy = True         
            state = 1
                    
        elif state == 4: #Error Code State
            pass
        
    window.close()
    ser.close()
main()