package frc.robot;

public class LineFollower {

    ColorSensor sensor;

    public LineFollower(ColorSensor sense)
    {
        sensor = sense;
    }
    
    public String getOutput()
    {

        sensor.read();

        //double[] leftRight = new double[2];

        if (sensor.grayscale > 220)
        {
          return "Straight";
        }
        else if (sensor.grayscale < 220)
        {
            return "Left";
        }
        else 
        {
            return "Stop";
        }

        //return "Stop";
    }

}