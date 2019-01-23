package org.team2168.PID.trajectory;

import java.awt.Color;

import org.team2168.PID.pathplanner.FalconLinePlot;

public class OneDimensionalRotation {

	
	OneDimensionalMotionProfiling wheels;
	OneDimensionalMotionProfiling angle;
	
	double vMax = 300.0;
	double aMax = 2000.0;
	double jMax =15000.0;
	
	double[] time;
	double[] pos;
	double[] vel;		
	double[] acc;
	double[] jerk;
	public double[] heading;
	
	public OneDimensionalRotation(double distance, double v_max, double accel_max, double j_max)
	{
		

		
	}
	
	public OneDimensionalRotation(double start, double distance, double v_max, double accel_max, double j_max)
	{
		boolean backwards=false;

		double circumference;
		if(distance>start) //rotate clockwise
		{
			circumference = (Math.PI*3*(distance-start)/180.0)/12.0;
			angle = new OneDimensionalMotionProfiling( start,  distance,  v_max,  accel_max,  j_max);
		}
		else
		{
			backwards=true;
			circumference = (Math.PI*3*(start-distance)/180.0)/12.0;
			angle = new OneDimensionalMotionProfiling( distance,  start,  v_max,  accel_max,  j_max);
			//this.backwards;
		}

		
		angle.S_curves();
		System.out.println(circumference);
		wheels = new OneDimensionalMotionProfiling(circumference, 8.0, 8.0, 100.0);
		wheels.S_curves();
		
		
		this.time = wheels.time;
		this.pos = wheels.pos;
		this.vel = wheels.vel;
		this.acc = wheels.vel;
		this.jerk = wheels.jerk;
		
		
		int counterAngle = angle.time.length-1;
		int counterWheels = wheels.time.length-1;
		
		heading  = new double[wheels.getTimeArray().length];
		
		//if angle array larger
		if(counterAngle >= counterWheels)
		{
			
			
			for(int i=heading.length-1; i>=0; i--)
			{
				heading[i] = angle.pos[counterAngle];
				counterAngle--;
			}
			
			System.out.println("This");
		}	
		else //wheels is greater than angle
		{
			for(int i=counterAngle; i>=0; i-- )
			{
				heading[counterWheels] = angle.pos[i];
				counterWheels--;
			}
			
			for(int i=counterWheels; i>=0; i--)
				heading[i]=angle.pos[0];;
			
			System.out.println("that");
		}
		
		
		   //we want to drive the path backwards
		   // swap the left and right wheels, and negate the velocitys, also correct
		   //heading to be 180 from current position
		   if(backwards) //invert heading
		   {

		   	   
		   	   double[] temp = new double[this.heading.length];
		   	   int counter = this.heading.length-1;
		   	   for (int i=0; i<temp.length; i++)
		   	   {
		   		   temp[i] = this.heading[counter];
		   		   counter--;
		   	   }
		   	   
		   	   heading = temp;
		   }
	 	   

	}
	
	
	
	
	
	public OneDimensionalRotation()
	{
		
		
	}
	
	
	public static void main(String[] args)
	{
		double vMax = 2500.0;
		double aMax = 3000.0;
		double jMax =30000.0;
		System.out.println("Hello");
		 OneDimensionalRotation rot = new OneDimensionalRotation(40, -10, 2500, 3000, 30000);
		 
		 
		 FalconLinePlot fig3 = new FalconLinePlot(rot.time, rot.pos ,Color.black);
			fig3.yGridOn();
			fig3.xGridOn();
			fig3.setYLabel("Position (inches)");
			fig3.setXLabel("time (seconds)");
			fig3.setTitle("Top Down View of FRC Field (30ft x 27ft) \n shows global position of robot path, along with left and right wheel trajectories");
			fig3.setSize(600,400);
			
			FalconLinePlot fig5 = new FalconLinePlot(new double[][]{{0.0,0.0}});
			fig5.yGridOn();
			fig5.xGridOn();
			fig5.setYLabel("Accel (ft/sec/sec)");
			fig5.setXLabel("time (seconds)");
			fig5.setTitle("Accel Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
			fig5.addData(rot.time,rot.acc, Color.black);
			
			FalconLinePlot fig6 = new FalconLinePlot(new double[][]{{0.0,0.0}});
			fig6.yGridOn();
			fig6.xGridOn();
			fig6.setYLabel("Accel (ft/sec/sec)");
			fig6.setXLabel("time (seconds)");
			fig6.setTitle("Jerk Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
			fig6.addData(rot.time,rot.jerk, Color.black);
		 
			FalconLinePlot fig7 = new FalconLinePlot(new double[][]{{0.0,0.0}});
			fig7.yGridOn();
			fig7.xGridOn();
			fig7.setYLabel("Heading");
			fig7.setXLabel("time (seconds)");
			fig7.setTitle("Gyro Angle");
			fig7.addData(rot.time,rot.heading, Color.black);
		
	}
	
}
