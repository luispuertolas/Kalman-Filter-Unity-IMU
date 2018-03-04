using UnityEngine;
using System.IO.Ports;
using System;
using System.IO;


public class KalmanDemo : MonoBehaviour
{
    KalmanFilter filter;
    private SerialPort port = new SerialPort("COM3", 9600);
    private String CurrentReading = "ABCD";
    private float[] IndividualValues = { 0, 0, 0, 0, 0, 0 };
    private String[] SubStrings;
    private float x=0, y=0, z=0, vx=0, t2=0, x1, y1, z1, time, vel, acc;

    //Initialize the filter
    void Start()
    {
        port.ReadTimeout = 2000;
        port.Close();
        if (!port.IsOpen)
            port.Open();
        filter = new KalmanFilter(new float[,]
         {{0f}, //XPos
			    {0f}, //YPos
			    {0f}, //ZPos
			    {0f}, //XVel
			    {0f}, //YVel
			    {0f}, //ZVel
			    {0f}, //XAccel
			    {0f}, //YAccel
			    {0f}, //ZAccel
         },

         //Measurement Matrix
         //Tells the filter which of the above we're actually sensing
         //Turns out you can just straight up comment out the lines that 
         //you want the filter to solve for
         new float[,]
          {{1f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //XPos
		   //{0f, 1f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //YPos - Not Measured!
		   //{0f, 0f, 1f, 0f, 0f, 0f, 0f, 0f, 0f}, //ZPos - Not Measured!
		   //{0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 0f}, //XVel - Not Measured!
		   //{0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f}, //YVel - Not Measured!
		   //{0f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f}, //ZVel - Not Measured!
		   {0f, 0f, 0f, 0f, 0f, 0f, 1f, 0f, 0f}, //XAccel
           //{0f, 0f, 0f, 0f, 0f, 0f, 0f, 1f, 0f}, //YAccel - Not Measured!
           //{0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 1f} //ZAccel - Not Measured!
          },

         //Process Noise; how much each value will deviate from the predicted value
         //Quick gaussian distribution intuition
         //If the ProcessNoise is 0.1 then you're saying that
         //68% of the time, the actual system will deviate less than sqrt(0.1) *from the predicted value*
         //and 95% of the time, less than 2*sqrt(0.1) etc.
         //It's a measure of how good your StateTransitionMatrix is
         //Ours sucks since it doesn't factor in that a meddling sine wave is moving the tracker around
         //So our numbers need to be kind of high
         new float[,]
          {{0.31471f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //XPos
			    //{0f, 0.562f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //YPos
			    //{0f, 0f, 0.560f, 0f, 0f, 0f, 0f, 0f, 0f}, //ZPos
			    {0f, 0f, 0f, 0.000169f, 0f, 0f, 0f, 0f, 0f}, //XVel
			    //{0f, 0f, 0f, 0f, 0.000025f, 0f, 0f, 0f, 0f}, //YVel
			    //{0f, 0f, 0f, 0f, 0f, 0.000025f, 0f, 0f, 0f}, //ZVel
			    {0f, 0f, 0f, 0f, 0f, 0f, 0.00129f, 0f, 0f}, //Constant XAccel
			    //{0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //Constant YAccel
			    //{0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f} //Constant ZAccel
          },

         //Same as above, except now it's the measurement deviating from actual position (not the predicted position)
         new float[,]
          {{-0.530047805f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //XPos
			    //{0f, 0.000249f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //YPos
			    //{0f, 0f, 60f, 0.000199f, 0f, 0f, 0f, 0f, 0f}, //ZPos
			    {0f, 0f, 0f, -0.00000315516f, 0f, 0f, 0f, 0f, 0f}, //XVel
			    //{0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //YVel
			    //{0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //ZVel
			    {0f, 0f, 0f, 0f, 0f, 0f, -1.000626221f, 0f, 0f}, //XAccel
			    //{0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, //YAccel
			    //{0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f} //ZAccel
          },

         //Initial Error measurement; 0f = slow initial convergence, 1f = fast initial convergence
         1f
     );
  }
    
    void Update() {
        CurrentReading = port.ReadLine();
        SubStrings = CurrentReading.Split(',');
        print(CurrentReading);
        x1 = Convert.ToSingle(SubStrings[0]);
        y1 = Convert.ToSingle(SubStrings[1]);
        z1 = Convert.ToSingle(SubStrings[2]);
        time = Convert.ToSingle(SubStrings[4]);
        if(x >=0 && x1>=0)
        {
            vel = x1 - x;
        }
        if (x <= 0 && x1 <= 0)
        {
            vel = x1 + x;
        }
        //Dummy Data : Generate a very noisy position estimate
        filter.UpdateState(new float[,]
        {{x1}, //Noisy Position
	    {0},
        {0},
        {(vel)/(time-t2)},
        {0},
        {0},
        {(vel-vx)/(time-t2)}, //Flawless acceleration
	    {0},
        {0}});
        filter.PredictState(Time.deltaTime);
        transform.localEulerAngles = new Vector3(0, filter.StateMatrix[0, 0], 0);
        x = x1;
        y = y1;
        z = z1;
        vx = vel;
        t2 = time;
    }
}
