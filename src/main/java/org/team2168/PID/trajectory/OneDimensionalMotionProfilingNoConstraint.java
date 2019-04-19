package org.team2168.PID.trajectory;

import java.awt.Color;

import org.team2168.PID.pathplanner.FalconLinePlot;

public class OneDimensionalMotionProfilingNoConstraint {

	
	
	double delta = 0.0;
	double q0 = 0.0;
	double q1 = 0.3;
	double v0 = 0.0;
	double v1 =  0.0;
	double a0 = 0.0;
	double a1 = 0.0;
	double t0 = 0.0;
	double t1 = 0.0;

	double T=t1-t0;
	double h=q1-q0;

	//Quintic Matrix
	double A0 = q0;
	double A1 = v0;
	double A2 = 1/2 * a0;
	double A3 = 1/(2*Math.pow(T,3)) * (20*h - (8*v1 + 12*v0)*T - (3*a0 - a1)*Math.pow(T,2));
	double A4 = 1/(2*Math.pow(T,4)) * (-30*h +(14*v1 + 16*v0)*T + (3*a0 - 2*a1)*Math.pow(T,2));
	double A5 = 1/(2*Math.pow(T,5)) * (12*h - 6*(v1+v0)*T + (a1-a0)*Math.pow(T,2));

	// vector array thing
	double spacing = 50.0; //1/50 hz
	
	public double[] time;
	public double[] pos;
	public double[] vel;		
	public double[] acc;
	public double[] jerk;

	public static void main(String[] args){
		
		OneDimensionalMotionProfilingNoConstraint oneDirection= new OneDimensionalMotionProfilingNoConstraint(0,22,5);
		
		FalconLinePlot fig3 = new FalconLinePlot(oneDirection.time, oneDirection.pos ,Color.black);
		fig3.yGridOn();
		fig3.xGridOn();
		fig3.setYLabel("Position (inches)");
		fig3.setXLabel("time (seconds)");
		fig3.setTitle("Top Down View of FRC Field (30ft x 27ft) \n shows global position of robot path, along with left and right wheel trajectories");
		fig3.setSize(600,400);

		//Velocity
		FalconLinePlot fig4 = new FalconLinePlot(new double[][]{{0.0,0.0}});
		fig4.yGridOn();
		fig4.xGridOn();
		fig4.setYLabel("Velocity (ft/sec)");
		fig4.setXLabel("time (seconds)");
		fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig4.addData(oneDirection.time,oneDirection.vel, Color.magenta);
		
		FalconLinePlot fig5 = new FalconLinePlot(new double[][]{{0.0,0.0}});
		fig5.yGridOn();
		fig5.xGridOn();
		fig5.setYLabel("Accel (ft/sec/sec)");
		fig5.setXLabel("time (seconds)");
		fig5.setTitle("Accel Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig5.addData(oneDirection.time,oneDirection.acc, Color.black);
		
		FalconLinePlot fig6 = new FalconLinePlot(new double[][]{{0.0,0.0}});
		fig6.yGridOn();
		fig6.xGridOn();
		fig6.setYLabel("Accel (ft/sec/sec)");
		fig6.setXLabel("time (seconds)");
		fig6.setTitle("Jerk Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig6.addData(oneDirection.time,oneDirection.jerk, Color.black);
	
		
	}
	
	
	public OneDimensionalMotionProfilingNoConstraint(double distance, double time)
	{
		this.q1 = distance;
		this.t1 = time;
		S_curves();
	}

	public OneDimensionalMotionProfilingNoConstraint(double start, double end, double time)
	{
		this.q0 = start;
		this.q1 = end;
		this.t1 = time;
		S_curves();
	}

	
	public void S_curves()
	{

		T=t1-t0;
		h=q1-q0;

		A0 = q0;
		A1 = v0;
		A2 = 1/2 * a0;
		A3 = 1/(2*Math.pow(T,3)) * (20*h - (8*v1 + 12*v0)*T - (3*a0 - a1)*Math.pow(T,2));
		A4 = 1/(2*Math.pow(T,4)) * (-30*h +(14*v1 + 16*v0)*T + (3*a0 - 2*a1)*Math.pow(T,2));
		A5 = 1/(2*Math.pow(T,5)) * (12*h - 6*(v1+v0)*T + (a1-a0)*Math.pow(T,2));
			
		System.out.println("Time:" + T);
		System.out.println("Real Spacing: " + T*spacing);
		System.out.println("Calc Spacing: " + (int)Math.ceil(((T)*spacing)));


		 time = new double[(int)Math.ceil(((T)*spacing))+1]; 
		 pos = new double[(int)Math.ceil(((T)*spacing))+1];
		 vel = new double[(int)Math.ceil(((T)*spacing))+1];		
		 acc = new double[(int)Math.ceil(((T)*spacing))+1];
		 jerk = new double[(int)Math.ceil(((T)*spacing))+1];

		//Linspace time = new Linspace(t0, spacing, t1); nd avoding looping on time again for pos,vel,accel,jerk
		for(int i=0; i<time.length-1; i++)
		{
			time[i]=i*1.0/spacing + t0;
			pos[i] = positionProfile(A5,A4,A3,A2,A1,A0,time[i]);
			vel[i] = velocityProfile(A5,A4,A3,A2,A1,time[i]);
			acc[i] = accelerationProfile(A5,A4,A3,A2,time[i]);
			jerk[i] = jerkProfile( A5,A4,A3,time[i]);
		}
		//take care of last value, and avoding looping on time again for pos,vel,accel,jerk
		time[time.length-1] = T;
		pos[time.length-1] = positionProfile(A5,A4,A3,A2,A1,A0,time[time.length-1]);
		vel[time.length-1] = velocityProfile(A5,A4,A3,A2,A1,time[time.length-1]);
		acc[time.length-1] = accelerationProfile(A5,A4,A3,A2,time[time.length-1]);
		jerk[time.length-1] = jerkProfile( A5,A4,A3,time[time.length-1]);
		//System.out.println(time[time.length-1]);
				//Linspace time = new Linspace(t0, spacing, t1);
		for(int i=0; i<time.length; i++)
		{

		}
	}
		

double positionProfile(double ax,double bx,double cx,double dx, double ex,double fx, double t)
{
    return ax * t * t * t * t * t + bx * t * t * t * t + cx * t * t * t + dx * t * t + ex * t + fx;
} 

double velocityProfile(double ax,double bx,double cx,double dx, double ex, double t)
{
    return 5 * ax * t * t * t * t + 4 * bx * t * t * t + 3 * cx * t * t + 2 * dx * t + ex;
} 

double accelerationProfile(double ax,double bx,double cx,double dx, double t)
{
    return 20 * ax * t * t * t + 12 * bx * t * t + 6 * cx * t + 2 * dx;
} 

double jerkProfile(double ax,double bx,double cx, double t)
{
    return 60 * ax * t * t + 24 * bx * t + 6 * cx;
} 
	


public double[] getTimeArray()
{
	return time;
}

public double[] getPosArray()
{
	return pos;
}

public double[] getVelArray()
{
	return vel;
}

public double[] getAccelArray()
{
	return acc;
}

public double[] getJerkArray()
{
	return jerk;
}



}