package org.team2168.PID.trajectory;

import java.awt.Color;

import org.team2168.PID.pathplanner.FalconLinePlot;

public class OneDimensionalMotionProfiling {

	
	
	double delta = 0.0;
	double q0 = 0.0;
	double q1 = 0.3;
	double v0 = 0.0;
	double v1 =  0.0;
	double a0 = 0.0;
	double a1 = 0.0;
	double t0 = 0.0;
	
	double error = q1-q0;
	
//	double vMax = 2500.0;
//	double aMax = 3000.0;
//	double jMax =30000.0;
	
	double vMax = 220.0;
	double aMax = 220.0;
	double jMax = 1000.0;
	
	double vMin = -vMax;
	double aMin = -aMax;
	double jMin = -jMax;
	
   

	
	double Tj1 = 0;
	double Ta = 0;
	double Tj2 = 0;
	double Td = 0;
	double Tv = 0;
	
	// vector array thing
	double spacing = 50.0; //1/50 hz
	
	public double[] time;
	public double[] pos;
	public double[] vel;		
	public double[] acc;
	public double[] jerk;

	public static void main(String[] args){
		
		OneDimensionalMotionProfiling oneDirection= new OneDimensionalMotionProfiling(0,50,110,110,1000);
		
		
//		for(int i=0; i<oneDirection.getVelArray().length; i++)
//			System.out.println(oneDirection.getPosArray()[i]);
		
		FalconLinePlot fig3 = new FalconLinePlot(oneDirection.time, oneDirection.pos ,Color.black);
		fig3.yGridOn();
		fig3.xGridOn();
		fig3.setYLabel("Position (inches)");
		fig3.setXLabel("time (seconds)");
		fig3.setTitle("Top Down View of FRC Field (30ft x 27ft) \n shows global position of robot path, along with left and right wheel trajectories");
		fig3.setSize(600,400);
		
//force graph to show 1/2 field dimensions of 24.8ft x 27 feet
double fieldWidth = 27.0;
//fig3.setXTic(0, 54, 1);
//fig3.setYTic(0, fieldWidth, 1);


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
	
	
	public OneDimensionalMotionProfiling(double distance)
	{
		this.q1 = distance;
		
		S_curves();
	}
	public OneDimensionalMotionProfiling(double distance, double v0)
	{
		this.q1 = distance;
		this.v0 = v0;
		S_curves();
	}
	
	public OneDimensionalMotionProfiling(double distance, double v0, double aMax)
	{
		this.q1 = distance;
		this.v0 = v0;
		this.aMax = a0;
		S_curves();
	}
	public OneDimensionalMotionProfiling(double distance, double v_max, double accel_max, double j_max)
	{
		this.q1 = distance;
		this.vMax = v_max;
		this.aMax = accel_max;
		this.jMax = j_max;
		S_curves();
		
	}
	
	public OneDimensionalMotionProfiling(double start, double distance, double v_max, double accel_max, double j_max)
	{
		if(start > distance)
		{
		this.q0 = distance;
		this.q1 = start;
		}
		else
		{
		this.q0 = start;
		this.q1 = distance;
		}
		
		double error = q1-q0;
		
		if(error<10)
		{
			this.vMax = v_max;
			this.aMax = accel_max;
			this.jMax = j_max;
		}
		else
		{
			this.vMax = v_max;
			this.aMax = accel_max;
			this.jMax = j_max;
		}
		S_curves();
		
		if(start > distance)
		{
			this.invert();
		}
	}
	 
	
	public void S_curves()
	{
		if (((vMax- v0) * jMax ) < Math.pow(aMax, 2)){
			Tj1 = Math.sqrt((vMax - v0)/jMax);
			Ta = 2*Tj1;
			System.out.println("A");
		}
		else 
		{
			Tj1 = aMax/jMax;
			Ta = Tj1 + ((vMax - v0)/aMax);
			System.out.println("B");
		}	
		
		if (((vMax- v1) * jMax ) < Math.pow(aMax, 2)){
			Tj2 = Math.sqrt((vMax - v1)/jMax);
			Td = 2 * Tj2;
			System.out.println("C");
		}
		else {
			Tj2 = aMax/jMax;
			Td = Tj2 + ((vMax - v1) / aMax);
			System.out.println("D");
		}	
		
		
		// Tj1 = 1/6
		// Ta = 4/5+1/6 = 29/30
		//Tj2 = 1/6 
		// Td =	4/5 + 1/6 = 29/30	
		// Tv = 5/4 - (29/60)*(1/4) - 29/60 = 155/240 = 31/48
		// t1 = 681/240 = 1.22
		
		//duration of constant velocity
		Tv = (q1-q0)/vMax - (Ta/2) * (1+v0/vMax) - (Td/2)*(1+(v1/vMax));
		
		if (Tv <= 0) 
		{
			Tj1 = aMax / jMax;
			Tj2 = aMax/ jMax;
			Tv = 0;
			
			delta = Math.pow(aMax, 4) / Math.pow(jMax, 2) + 2 * (Math.pow(v0, 2) + Math.pow(v1, 2)) + aMax * (4 * (q1 - q0)-2*(aMax/jMax) * (v0+v1));
		
			Ta = (Math.pow(aMax, 2) / jMax - 2 * v0 + Math.sqrt(delta)) / (2 * aMax);
			Td = (Math.pow(aMax, 2) / jMax - 2 * v1 + Math.sqrt(delta)) / (2 * aMax);
		}
		
		double t1 = Ta + Tv + Td;
		double	T = t1 - t0;
			
		System.out.println("Time:" + T);
		System.out.println("Real Spacing: " + T*spacing);
		System.out.println("Calc Spacing: " + (int)Math.ceil(((T)*spacing)));


		 time = new double[(int)Math.ceil(((T)*spacing))+1]; 
		 pos = new double[(int)Math.ceil(((T)*spacing))+1];
		 vel = new double[(int)Math.ceil(((T)*spacing))+1];		
		 acc = new double[(int)Math.ceil(((T)*spacing))+1];
		 jerk = new double[(int)Math.ceil(((T)*spacing))+1];
		
		//Linspace time = new Linspace(t0, spacing, t1);
		 for(int i=0; i<time.length-1; i++)
			{
				time[i]=i*1.0/spacing + t0;
				//System.out.println(time[i]);
			}
			//take care of last value
			time[time.length-1] = T;
			//System.out.println(time[time.length-1]);
		

		//Compute actual min/max a and vc
		double aLimA = jMax*Tj1;
		double aLimD = -jMax*Tj2;
		double vLim = v0 + (Ta-Tj1)*aLimA;
		
								
		
		// Calculation of trajectory for q1 > q2
		// ??
		for(int i=0;i<time.length;i++)
		{
			//System.out.println(time[i]);
			 if( time[i] <= Tj1) {//t<1/6
				 pos[i]=q0 + v0*(time[i]) + jMax*Math.pow((time[i]),3)/6;
				 //System.out.println("time[i] " +  time[i] + " pos[i] " +  pos[i]);
				 
			     vel[i]= v0 + jMax*Math.pow((time[i] + t0), 2)/2;
			     acc[i] = jMax*(time[i] + t0);
			     jerk[i] = jMax;}
			 
			    else if ((time[i] > Tj1) && (time[i] <= Ta - Tj1)) //1/6<t<4/5
			    {
			     pos[i] = q0 + v0*(time[i]) + (aLimA/6.0)*(3.0*Math.pow((time[i]), 2) - 3.0*Tj1*(time[i]) + Math.pow(Tj1,2)   );
			     //pos =1.228
			     vel[i] = v0 + aLimA*((time[i] + t0) - (Tj1/2));
			     acc[i] = jMax*Tj1;
			     jerk[i] = 0; }
			    else if ((time[i] + t0)>  Ta - Tj1 && (time[i] + t0) <= Ta) //4/5<t<29/30
			    {
			    pos[i] = q0 + (vLim + v0)*Ta/2 - vLim*(Ta-(time[i] + t0)) - jMin*(Math.pow((Ta-(time[i] + t0)),3)/6);
			    //pos = 1.290
			    vel[i] = vLim + jMin*(Math.pow((Ta-(time[i] + t0)),2)/2);
			    acc[i] = -jMin*(Ta-(time[i] + t0));
			    jerk[i] = jMin;
			    }
			    else if ((time[i] + t0) > Ta && (time[i] + t0) <= Ta + Tv) //29/30<t< 261/240
			    {
			    	pos[i] = q0 + (vLim + v0)*(Ta/2)+vLim*((time[i] + t0)-Ta);
			        vel[i] = vLim;
			        acc[i] = 0;
			        jerk[i] = 0;
			    }
			    else if ((time[i] + t0) > T-Td && (time[i] + t0) <= T-Td+Tj2)   // 9 1/30 < t < 9 1/5
			    {
			        pos[i] = q1 - (vLim +v1)*(Td/2) + vLim*((time[i] + t0)-T+Td)-jMax*(Math.pow(((time[i] + t0)-T+Td),3)/6);
			        vel[i] = vLim - jMax*(Math.pow(((time[i] + t0)-T+Td),2)/2);
			        acc[i] = -jMax*((time[i] + t0)-T+Td);
			        jerk[i] = jMin;
			    }
			    else if ((time[i] + t0) > T-Td+Tj2 && (time[i] + t0) <= T-Tj2) // 9 1/5 < t < 9 5/6
			    {
			        pos[i] = q1 - (vLim+v1)*(Td/2)+vLim*((time[i] + t0)-T+Td) + (aLimD/6)
			        		*(3*Math.pow(((time[i] + t0)-T+Td),2) - 3*Tj2*((time[i] + t0)-T+Td) + Math.pow(Tj2,2));
			        vel[i] = vLim+aLimD*((time[i] + t0)-T+Td-Tj2/2);
			        acc[i] = aLimD;
			        jerk[i] = 0;
			    }
			    else if ((time[i] + t0) > T-Tj2 && (time[i] + t0) <=T)  // 9 5/6 < t < 10
			    {
			    	pos[i] = q1-v1*(T-(time[i] + t0))-jMax*(Math.pow((T-(time[i] + t0)),3)/6);
			        vel[i] = v1+jMax*(Math.pow((T-(time[i] + t0)),2)/2);
			        acc[i] = -jMax*(T-(time[i] + t0));
			        jerk[i] = jMax;  
			    }
		}
		//Lets create a bank image
	
	}
		
	

public void invert()
{
		int i = pos.length-1;

		  //Invert arrays
		  double[] temp_p = new double[pos.length];
		  double[] temp_v = new double[pos.length];
		  double[] temp_a = new double[pos.length];
		  double[] temp_j = new double[pos.length];

		//inverting all arrays
		for(int j=i; j>=0; j--)
		{
			temp_p[j]= pos[i-j];
			temp_v[j]= -vel[i-j];
			temp_a[j]= acc[i-j];
			temp_j[j]= -jerk[i-j];
		}

		this.pos = temp_p;
		this.vel =temp_v;
		this.acc =temp_a;
		this.jerk =temp_j;
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