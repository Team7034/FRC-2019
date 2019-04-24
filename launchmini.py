from launchpad_py import LaunchpadBase
from threading import Thread, current_thread
from networktables import NetworkTables
import time
import logging


#launchpad mini class, extends the launchpad base
class LaunchpadMini(LaunchpadBase): 
    
    #initialize launchpad and raise error if not connected
    def __init__( self ): #constructor
        super().__init__() #running the constructor for launchpad
        if not super().Open():
            raise RuntimeError("Could not connect to the Launchpad.")

        self.Reset() 
        self.quit_list = [False, False, 0] #when you hold down button 1 and 8 for 5 seconds it will quit
        self.button_state = [] #saves the values of the last button pressed

        self.grid_status = 0

        self.isCleared = False

        '''~~~~~~~~~~~~~~~grid_status~~~~~~~~~~~~~~~~~~~~~~
        0: the main launchpad driving grid
        1: resetting, middle 4x4 is red, if you hit a button within then it changes to self.grid_status = 0
        2: the entire thing is green, alyssa can manually drive
        3: the entire thing is red, alyssa cannot manually drive

        '''
        
        #button = [x,y,on/off]

        #network tables initializion:
        #to see messages from networktables
        logging.basicConfig(level=logging.DEBUG) 
        #connects to 7034 server
        NetworkTables.initialize(server="10.70.34.2") 

        #creates an instance of the network table
        self.net_table = NetworkTables.getTable("SmartDashboard")
        #creating a subtable in the network table
        self.pos_table = self.net_table.getSubTable("miniTable")

    #reset the launchpad & turn off all LEDs
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
        
    def check_buttons(self, button):    
        self.button_state = button
        #check if position needs to be reset

        if button[0] == 1 and button[1] == 2 and button[2] and self.grid_status == 0:
            self.grid_status = 1 #1 means position is reset
            self.net_table.putNumber("gridStat", self.grid_status) 
        
        #check if position has been reset
        #in central four buttons on launchpad
        if button[0] >= 2 and button[0] <= 5 and button[1] >= 3 and button[1] <= 6 and button[2] and self.grid_status == 1:
            self.reset_grid()
            self.grid_status = 0 #0 means alyssa's driving, pathfinder is ready to go
            self.net_table.putNumber("gridStat", self.grid_status)
            coords = self.convert_coords(button[0], button[1])
            #self.net_table.putNumberArray("position", coords)
            self.net_table.putNumber("x_robot", coords[0])
            self.net_table.putNumber("y_robot", coords[1]) 

        elif button[0] >= 2 and button[0] <= 5 and button[1] >= 3 and button[1] <= 6 and button[2] and self.grid_status == 0:
            coords = self.convert_coords(button[0],button[1])
            #self.net_table.putNumberArray("position", coords)
            self.net_table.putNumber("x_target", coords[0])
            self.net_table.putNumber("y_target", coords[1])



    def update_drive_status(self):
        if self.quit_list[2] == 0:
            if self.grid_status == 0:
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


        #if (self.net_table.getNumber("gridStat", 0) == 0) and (self.grid_status != 0):
        if self.net_table.getNumber("gridStat", 0) == 0:
            self.grid_status = 0
        if self.net_table.getNumber("gridStat", 0) == 1:
            self.grid_status = 1
        if self.net_table.getNumber("gridStat", 0) == 3:
            self.grid_status = 3
        if self.net_table.getNumber("gridStat", 0) == 4:
            self.grid_status = 4
        


        #self.net_table.putNumber("gridStat", self.grid_status)


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

    def show_drive_method(self):
        if self.grid_status == 3: #all green!
            self.clear_board()
            for x in range(0, 9):
                for y in range(0, 9):
                    self.LedCtrlXY(x,y,0,3)
        elif self.grid_status == 4: #all red!
            self.clear_board()
            for x in range(0, 9):
                for y in range(0, 9):
                    self.LedCtrlXY(x,y,3,0)

    def lightup_target(self):
        if self.grid_status == 0:
            print("path in progress")
            self.LedCtrlXY(button[0], button[1], 3, 0)


    def drive_mode(self):
        if button[0] == 8 and button[1] == 1:
            self.grid_status == 1
            self.net_table.putNumber("gridStat", self.grid_status)

    def clear_board(self):
        for x in range(0, 9):
            for y in range(0, 9):
                self.LedCtrlXY(x,y,0,0)
