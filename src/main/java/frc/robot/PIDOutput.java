package frc.robot;
class PIDOutput
{
    public double k_prop;
    public double k_int;
    public double k_diff;

    public PIDOutput ()
    {
        k_prop = 0.22;
        k_int = 0.1;
        k_diff = 0.1;
    }

    public double output(double in, double target)
{
    double err = in - target;
    double out = 0;

    out+=(k_prop * err); //p term
    out+=(k_int * 4);
    



    out = 0;


    return out;
}
}