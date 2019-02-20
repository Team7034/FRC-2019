from launchpad_py import LaunchpadBase
from threading import Thread, current_thread
from networktables import NetworkTables
import time
import logging

class LaunchpadMini(LaunchpadBase): #launchpad mini class, extends the launchpad base
    '''
    #showing time
    i = 0
    while True:
        print("dsTime:", sd.getNumber("dsTime", "N/A"))
        self.net_table.putNumber("robotTime", i)
        time.sleep(1) #pauses the loop for 1 second
        i += 1
    '''
    #initialize launchpad and raise error if not connected
    def __init__( self ): #constructor
        super().__init__() #running the constructor for launchpad
        if not super().Open():
            raise RuntimeError("Could not connect to the Launchpad.")

        self.Reset() #all functions need "self", and all variables
        self.quit_list = [False, False, 0] #when you hold down button 1 and 8 for 5 seconds it will quit
        self.button_state = [] #saves the values of the last button pressed



        self.automatic = False #alyssa is driving
        self.grid_status = 0 
        '''
        are u resetting (when you're not on a set path by pathfinder so you hit where 
        you are on the launchpad map) (1) or is it in driving mode (0)
        '''
        self.high_gear = True #should basically always be in high gear






        #set up basic leds
        self.LedCtrlXY(8,1,3,3) #button A
        self.LedCtrlXY(8,2,0,0) #button B

        #network tables init
        logging.basicConfig(level=logging.DEBUG) #to see messages from networktables
        NetworkTables.initialize(server="10.70.34.2") #connects to 7034 server
        
        '''
        #code to wait until connected
        cond = threading.Condition() 
        notified = [False]

        def connectionListener(connected, info):
            print(info, '; Connected=%s' % connected)
            with cond:
                notified[0] = True
                cond.notify()

        NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

        with cond:
            print("Waiting")
            if not notified[0]:
                cond.wait()
        print("Connected!")'''

        #creates an instance of the network table
        #remember to use self
        self.net_table = NetworkTables.getTable("SmartDashboard")
        ''' lets get rid of sub tables for now
        #creating a subtable in the network table
        self.sub_table = self.net_table.getSubTable("miniTable")
        '''
        #creates variables in the subtable for automatic, grid status, and high gear
        self.net_table.putBoolean("automatic", False)
        self.net_table.putNumber("gridStat", 0)
        self.net_table.putBoolean("highGear", True)


    #-------------------------------------------------------------------------------------
    #-- reset the Launchpad
    #-- Turns off all LEDs
    #-------------------------------------------------------------------------------------
    def Reset( self ):
        self.midi.RawWrite( 176, 0, 0 )


    #-------------------------------------------------------------------------------------
    #-- Returns a Launchpad compatible "color code byte"
    #-- NOTE: In here, number is 0..7 (left..right)
    #-------------------------------------------------------------------------------------
    def LedGetColor( self, red, green ):
        led = 0
        
        red = min( int(red), 3 ) # make int and limit to <=3
        red = max( red, 0 )      # no negative numbers

        green = min( int(green), 3 ) # make int and limit to <=3
        green = max( green, 0 )      # no negative numbers

        led |= red
        led |= green << 4 
        
        return led

        
    #-------------------------------------------------------------------------------------
    #-- Controls a grid LED by its raw <number>; with <green/red> brightness: 0..3
    #-- For LED numbers, see grid description on top of class.
    #-------------------------------------------------------------------------------------
    def LedCtrlRaw( self, number, red, green ):

        if number > 199:
            if number < 208:
                # 200-207
                self.LedCtrlAutomap( number - 200, red, green )
        else:
            if number < 0 or number > 120:
                return
            # 0-120
            led = self.LedGetColor( red, green )
            self.midi.RawWrite( 144, number, led )


    #-------------------------------------------------------------------------------------
    #-- Controls a grid LED by its coordinates <x> and <y>  with <green/red> brightness 0..3
    #-------------------------------------------------------------------------------------
    def LedCtrlXY( self, x, y, red, green ):

        if x < 0 or x > 8 or y < 0 or y > 8:
            return

        if y == 0:
            self.LedCtrlAutomap( x, red, green )
        
        else:
            self.LedCtrlRaw( ( (y-1) << 4) | x, red, green )

    #-------------------------------------------------------------------------------------
    #-- Controls an automap LED <number>; with <green/red> brightness: 0..3
    #-- NOTE: In here, number is 0..7 (left..right)
    #-------------------------------------------------------------------------------------
    def LedCtrlAutomap( self, number, red, green ):

        if number < 0 or number > 7:
            return

        # TODO: limit red/green
        led = self.LedGetColor( red, green )
        
        self.midi.RawWrite( 176, 104 + number, led )


    #-------------------------------------------------------------------------------------
    #-- all LEDs on
    #-- <colorcode> is here for backwards compatibility with the newer "Mk2" and "Pro"
    #-- classes. If it's "0", all LEDs are turned off. In all other cases turned on,
    #-- like the function name implies :-/
    #-------------------------------------------------------------------------------------
    def LedAllOn( self, colorcode = None ):
        if colorcode == 0:
            self.Reset()
        else:
            self.midi.RawWrite( 176, 0, 127 )


                    
    #-------------------------------------------------------------------------------------
    #-- Returns True if a button event was received.
    #-------------------------------------------------------------------------------------
    def ButtonChanged( self ):
        return self.midi.ReadCheck()

        
    #-------------------------------------------------------------------------------------
    #-- Returns the raw value of the last button change as a list:
    #-- [ <button>, <True/False> ]
    #-------------------------------------------------------------------------------------
    def ButtonStateRaw( self ):
        if self.midi.ReadCheck():
            a = self.midi.ReadRaw()
            return [ a[0][0][1] if a[0][0][0] == 144 else a[0][0][1] + 96, True if a[0][0][2] > 0 else False ]
        else:
            return []


    #-------------------------------------------------------------------------------------
    #-- Returns an x/y value of the last button change as a list:
    #-- [ <x>, <y>, <True/False> ]
    #-------------------------------------------------------------------------------------
    def ButtonStateXY( self ):
        if self.midi.ReadCheck():
            a = self.midi.ReadRaw()

            if a[0][0][0] == 144:
                x = a[0][0][1] & 0x0f
                y = ( a[0][0][1] & 0xf0 ) >> 4
                
                self.button_state = [ x, y+1, True if a[0][0][2] > 0 else False ]
                return self.button_state
                
            elif a[0][0][0] == 176:
                self.button_state = [ a[0][0][1] - 104, 0, True if a[0][0][2] > 0 else False ]
                return self.button_state
        
        self.button_state = []
        return self.button_state

    #general frc LED functions ect
    #button = [x,y,on/off]

    def check_buttons(self, button):
        #check if auto has been toggled
        #if y equals 0 and its pressed and basically if its one of those top circle buttons [2,7]
        if button[1] == 0 and button[2] and button[0] >= 1 and button[0] <= 6:
            self.automatic = not self.automatic
            #send self.automatic to the robot
            self.net_table.putBoolean("automatic", self.automatic)
        
        #check if position needs to be reset

        if button[0] == 1 and button[1] == 2 and button[2] and self.grid_status == 0:
            self.grid_status = 1 #1 means position is reset
            self.net_table.putNumber("gridStat", self.grid_status)
        
        #check if position has been reset
        #in central four buttons on launchpad
        if button[0] >= 2 and button[0] <= 5 and button[1] >= 3 and button[1] <= 6 and button[2] and self.grid_status == 1:
            self.reset_grid()
            self.grid_status = 0 #0 means driving mode, we're on track, not resetting mode
            self.net_table.putNumber("gridStat", self.grid_status)
            x, y = self.convert_coords(button[0], button[1])
            self.net_table.putNumber("xPos", x)
            self.net_table.putNumber("yPos", y)
            #send coordinates of grid? to tell robot its goal location um that's a casey problem

        if button[0] >= 2 and button[0] <= 5 and button[1] >= 3 and button[1] <= 6 and button[2] and self.grid_status == 0:
            x, y = self.convert_coords(button[0],button[1])
            self.net_table.putNumber("xPos", x)
            self.net_table.putNumber("yPos", y)

        #check if shifting
        if button[0] == 8 and button[1] == 1 and button[2] and not self.high_gear:
            self.high_gear = True
            self.LedCtrlXY(8,1,3,3)
            self.LedCtrlXY(8,2,0,0)
            self.net_table.putBoolean("highGear", self.high_gear)
        elif button[0] == 8 and button[1] == 2 and button[2] and self.high_gear:
            self.high_gear = False
            self.LedCtrlXY(8,1,0,0)
            self.LedCtrlXY(8,2,3,3)
            self.net_table.putBoolean("highGear", self.high_gear)

    def check_quit( self ):
        if self.button_state != []:
            if self.button_state[0] == 0 and self.button_state[1] == 0:
                self.quit_list[0] = self.button_state[2]
            elif self.button_state[0] == 7 and self.button_state[1] == 0:
                self.quit_list[1] = self.button_state[2]

        if self.quit_list[0] and self.quit_list[1]:
            if self.quit_list[2] == 0:
                self.quit_list[2] = time.mktime(time.gmtime())
                self.LedCtrlXY(0,0,3,3)
                self.LedCtrlXY(1,0,0,0)
                self.LedCtrlXY(2,0,0,0)
                self.LedCtrlXY(3,0,0,0)
                self.LedCtrlXY(4,0,0,0)
                self.LedCtrlXY(5,0,0,0)
                self.LedCtrlXY(6,0,0,0)
                self.LedCtrlXY(7,0,3,3)
            elif self.quit_list[2]+1 == time.mktime(time.gmtime()):
                self.LedCtrlXY(1,0,3,3)
                self.LedCtrlXY(6,0,3,3)
            elif self.quit_list[2]+2 == time.mktime(time.gmtime()):
                self.LedCtrlXY(2,0,3,3)
                self.LedCtrlXY(5,0,3,3)
            elif self.quit_list[2]+3 == time.mktime(time.gmtime()):
                self.LedCtrlXY(3,0,3,3)
                self.LedCtrlXY(4,0,3,3)
            elif self.quit_list[2]+4 == time.mktime(time.gmtime()):
                return True
            else: 
                pass
    
        else:
            self.quit_list[2] = 0
            return False

    def update_drive_status(self):
        if self.quit_list[2] == 0:
            if self.automatic:
                self.LedCtrlXY(0,0,0,3)
                self.LedCtrlXY(1,0,0,3)
                self.LedCtrlXY(2,0,0,3)
                self.LedCtrlXY(3,0,0,3)
                self.LedCtrlXY(4,0,0,3)
                self.LedCtrlXY(5,0,0,3)
                self.LedCtrlXY(6,0,0,3)
                self.LedCtrlXY(7,0,0,3)
            else:
                self.LedCtrlXY(0,0,3,0)
                self.LedCtrlXY(1,0,3,0)
                self.LedCtrlXY(2,0,3,0)
                self.LedCtrlXY(3,0,3,0)
                self.LedCtrlXY(4,0,3,0)
                self.LedCtrlXY(5,0,3,0)
                self.LedCtrlXY(6,0,3,0)
                self.LedCtrlXY(7,0,3,0)

    def check_center(self, button):
        if button[0] >=2 and button[0] <= 5 and button[1] >= 3 and button[1] <= 6:
            if button[2]:
                self.LedCtrlXY(button[0], button[1], 3, 0)
            else:
                self.LedCtrlXY(button[0], button[1], 0, 0)

    def reset_grid(self):
        self.LedCtrlXY(2,3,0,0)
        self.LedCtrlXY(3,3,0,0)
        self.LedCtrlXY(4,3,0,0)
        self.LedCtrlXY(5,3,0,0)
        self.LedCtrlXY(2,4,0,0)
        self.LedCtrlXY(3,4,0,0)
        self.LedCtrlXY(4,4,0,0)
        self.LedCtrlXY(5,4,0,0)
        self.LedCtrlXY(2,5,0,0)
        self.LedCtrlXY(3,5,0,0)
        self.LedCtrlXY(4,5,0,0)
        self.LedCtrlXY(5,5,0,0)
        self.LedCtrlXY(2,6,0,0)
        self.LedCtrlXY(3,6,0,0)
        self.LedCtrlXY(4,6,0,0)
        self.LedCtrlXY(5,6,0,0)

    def update_grid_status(self):
        #ready for buttons to be pressed
        if self.grid_status == 0:
            self.LedCtrlXY(2,6,0,3)
            self.LedCtrlXY(5,6,0,3)
            self.LedCtrlXY(1,2,3,0)
        elif self.grid_status == 1:
            self.LedCtrlXY(2,3,3,0)
            self.LedCtrlXY(3,3,3,0)
            self.LedCtrlXY(4,3,3,0)
            self.LedCtrlXY(5,3,3,0)
            self.LedCtrlXY(2,4,3,0)
            self.LedCtrlXY(3,4,3,0)
            self.LedCtrlXY(4,4,3,0)
            self.LedCtrlXY(5,4,3,0)
            self.LedCtrlXY(2,5,3,0)
            self.LedCtrlXY(3,5,3,0)
            self.LedCtrlXY(4,5,3,0)
            self.LedCtrlXY(5,5,3,0)
            self.LedCtrlXY(2,6,3,0)
            self.LedCtrlXY(3,6,3,0)
            self.LedCtrlXY(4,6,3,0)
            self.LedCtrlXY(5,6,3,0)

    def convert_coords(self, device_x, device_y) : #wait how do i use device_x and device_y
        if device_y == 3:
            y = 3
        if device_y == 4:
            y = 2
        if device_y == 5:
            y = 1
        if device_y == 6:
            y = 0
        x = device_x - 2
        return (x, y)