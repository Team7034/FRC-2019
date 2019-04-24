import launchmini
import time

if __name__ == "__main__": #just python's psvm
    try: 
        lp = launchmini.LaunchpadMini()
    except RuntimeError:
        raise
    

    while True:
        lp.update_grid_status() #central 4x4 & lights
        lp.update_drive_status()
        lp.show_drive_method()
        

        if lp.ButtonChanged():
            #whenever a button is pressed, get info, update LEDS
            button = lp.ButtonStateXY()
            lp.check_buttons(button)
            lp.check_center(button)

        if lp.check_quit(): #do we wanna quit
            break

    lp.Reset() 
    lp.Close()

    