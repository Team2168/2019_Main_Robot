package org.team2168.PID.trajectory;
import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.TimeZone;
import java.util.TimerTask;

import org.team2168.RobotMap;
import org.team2168.PID.pathplanner.FalconLinePlot;



/**
 * The purpose of this project is to generate functions which provide smooth 
 * paths between global waypoints. The approach this project takes is to use
 * quintic (5th order splines) hermite splines to create a continue path
 * between governing waypoints. 
 * 
 * The objective is to have a solution which interpolates the control points
 * provided by the user, and to also have C2 continuity (continuous 1st and 2nd
 * order derivatives).
 * 
 * Since this project is to be used for the mobile navigation of a differential
 * drive mobile ground robot. This algorithm also provides the position, velocity,
 * acceleration, and jerk motion profiles, for the left and right wheels, while
 * trying to maintain max velocity, max acceleration, and max jerk constraints.
 * 
 * @author Kevin Harrilal
 * Reference https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf
 * https://www.siggraph.org/education/materials/HyperGraph/modeling/splines/hermite.htm
 * http://paulbourke.net/miscellaneous/interpolation/
 * https://www.cs.utexas.edu/~fussell/courses/cs384g/lectures/lecture16-Interpolating_curves.pdf
 * https://www.youtube.com/watch?v=dxvmafuP9Wk
 * http://www.cs.cmu.edu/afs/cs/academic/class/15462-s10/www/lec-slides/lec06.pdf
 * 
 * 
 */



public class QuinticTrajectory
{
	//Path Variables
	public double[][] origPath;
	private Spline[] splines = null;
	
	Trajectory traj;
	TrajectoryGenerator.Config config;
	Trajectory.Pair leftRightTraj; 
	
	public double[] leftVel;
	public double[] rightVel;
	public double[] leftPos;
	public double[] rightPos;
	public double[] time;
	public double[][] leftPath;
	public double[][] rightPath;
	public double[][] rightVelocity;
	public double[][] leftVelocity;
	public double[][] rightAccel;
	public double[][] leftAccel;
	public double[][] rightJerk;
	public double[][] leftJerk;
	public double[] heading;

	public double[] rightAcc;
	public double[] leftAcc;

	private static PrintWriter log;

	//private static String directory = "/home/lvuser/Path/";
	private static String directory = "Path/";
		
	public boolean reverse = false;

	double totalSplineLength = 0;
	double wheelSpacing = 26.0; //inches

	public static void main(String[] args)
	{
		
		System.out.println("Hello World");
		System.out.flush();
		
		
		double[][] waypointPath = new double[][]{
//			{0.0, 0.0, 0},
//			{6.0, 0.0, 0},
//			{10.0, 4.0, Math.PI/2-0.001}, //works with 19.3 on practice bot
//		//	{9.0, 20.5, Math.PI/2}
			
			{0.0, 0.0, 0},
			{30.0, 0.0, 0},
			{84.0, 54.0, Math.PI/2-0.001},
			{138.0, 108.0, 0},
			{168.0, 108.0, 0}
			
			// {48.0, 0.0, 0},
			// {96.0, 0.0, 0},
			// {120.0, 30.0, Math.PI/6},
			
			// {204.0, 21.0, -Math.PI/6},
			// {263.0, 32.0, Math.PI/3-0.001},
			// {252.0, 90.0, 5*Math.PI/6-0.002}
			
		};
		
		
		
		double[][] waypointPath2 = new double[][]{
			{0.0, 26.5, 0},
			{10.0, 28.5, Math.PI/2-0.001} //works with 19.3 on practice bot
	};
		
	double[][] waypointPath3 = new double[][]{
		
		
		{2, 25.5, 0},
		{16, 25.5, 0},
		{19, 21.5, -Math.PI/4+0.0001},

	};		
		QuinticTrajectory quinticPath= new QuinticTrajectory("path1.txt", waypointPath, false);
		quinticPath.calculate();
		//quinticPath.plotPath();
		//System.out.println(quinticPath.traj.toStringEuclidean());



		
		
		//Lets create a bank image

//		FalconLinePlot fig3 = new FalconLinePlot(waypointPath, null, Color.black);
//		fig3.yGridOn();
//		fig3.xGridOn();
//		fig3.setYLabel("Y (feet)");
//		fig3.setXLabel("X (feet)");
//		fig3.setTitle("Quintic Path (Robot Complete Path)");
//		//fig3.setTitle("Top Down View of FRC Field (30ft x 27ft) \n shows global position of robot path, along with left and right wheel trajectories");
//
//
////		//force graph to show 1/2 field dimensions of 24.8ft x 27 feet
//		double fieldWidth = 32;
//		fig3.setXTic(0, 40, 1);
//		fig3.setYTic(0, fieldWidth, 1);
//		fig3.addData(quinticPath.rightPath, Color.magenta);
//		fig3.addData(quinticPath.leftPath, Color.blue);
//
//		//fi[g3.addData(quinticPath2.leftPath, Color.blue);
//		//fig3.addData(quinticPath2.rightPath, Color.magenta);
//		//fig3.addData(waypointPath2, null, Color.black);
// 
//		fig3.addData(new double[][]{{4.667, 3}}, Color.black);

		FalconLinePlot fig3 = new FalconLinePlot(waypointPath, null, Color.black);
		fig3.yGridOn();
		fig3.xGridOn();
		fig3.setYLabel("Y (feet)");
		fig3.setXLabel("X (feet)");
		fig3.setTitle("Quintic Path (Robot Complete Path)");
		//fig3.setTitle("Top Down View of FRC Field (30ft x 27ft) \n shows global position of robot path, along with left and right wheel trajectories");


//		//force graph to show 1/2 field dimensions of 24.8ft x 27 feet
//		double fieldWidth = 32;
		double fieldWidth = 100;
//		fig3.setXTic(0, 30, 1);
		fig3.setXTic(0, 150, 1);
		fig3.setYTic(0, fieldWidth, 1);
		fig3.addData(quinticPath.rightPath, Color.magenta);
		fig3.addData(quinticPath.leftPath, Color.blue);
		

	//	fig3.addData(quinticPath2.leftPath, Color.blue);
	//	fig3.addData(quinticPath2.rightPath, Color.magenta);
	//	fig3.addData(waypointPath2, null, Color.black);
 
//		fig3.addData(quinticPath2.leftPath, Color.blue);
//		fig3.addData(quinticPath2.rightPath, Color.magenta);
//		fig3.addData(waypointPath3, null, Color.black);
		
		fig3.addData(new double[][]{{4.667, 3}}, Color.black);
		
		fig3.setXTic(0.0, 348.0, 12.0);
		fig3.setYTic(0.0, 348.0, 12.0);
		
		
		//outline field perimeter
		double[][] edge = {{336,12},{12,12},{12,336},{336,336}};
		fig3.addData(edge, Color.black);
		
		edge = new double[][] {{324,146},{221,146},{221,202},{324,202}};
		fig3.addData(edge, Color.black);
		
		edge = new double[][] {{12,110},{60,110},{60,238},{12,238}};
		fig3.addData(edge, Color.black);
		
		edge = new double[][] {{60,110},{60,100},{108,100},{108,248},{60,248},{60,238}};
		fig3.addData(edge, Color.black);
		
		edge = new double[][] {{12,150},{60,150}};
		fig3.addData(edge, Color.black);
		
		edge = new double[][] {{12,198},{60,198}};
		fig3.addData(edge, Color.black);
		
		
//		double[][] edge = {{(1),(16+27/2.0-2-4.81/12)},{(1),(16-27/2.0+2+4.81/12)}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{1,16+27/2.0-2-4.81/12},{1+2+10.9/12,16+27/2.0}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{1,16-27/2.0+2+4.81/12},{1+2+10.9/12, 16-27/2.0}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{1+2+10.9/12,16+27/2.0},{29,16+27/2.0}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{1+2+10.9/12, 16-27/2.0},{29,16-27/2.0}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{28,16+27/2.0}, {28, 16-27/2.0}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{12+7.6/12, 16+27/2.0-7-1.5/12}, {16+3.2/12, 16+27/2.0-7-1.5/12}, {16+3.2/12, 16-27/2.0+7+1.5/12}, {12+7.6/12, 16-27/2.0+7+1.5/12}, {12+7.6/12, 16+27/2.0-7-1.5/12}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{12+7.6/12,16+27/2.0-11-7.75/12}, {12+7.6/12-3.5, 16+27/2.0-11-7.75/12}, {12+7.6/12-3.5, 16-27/2.0+11+7.75/12}, {12+7.6/12, 16-27/2.0+11+7.75/12}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{12+7.6/12-3.5, 16+1.875},{12+7.6/12-3.5, 16-1.875}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{28,16+28/2.0-5-11.6/12}, {28-1-11.7/12, 16+28/2.0-5-11.6/12}, {28-1-11.7/12,  16+28/2.0-5-11.6/12-2-1.4/12}, 
//			{28-1-11.7/12-3-2.9/12, 16+28/2.0-5-11.6/12-2-1.4/12}, {28-1-11.7/12-3-2.9/12, 16-28/2.0+5+11.6/12+2+1.4/12}, {28-1-11.7/12, 16-28/2.0+5+11.6/12+2+1.4/12},
//			{28-1-11.7/12, 16-28/2.0+5+11.6/12}, {28, 16-28/2.0+5+11.6/12}};
//		fig3.addData(edge, Color.black);
//		
//		edge = new double[][] {{1,16+27/2.0-8-6.25/12}, {1+2+11.7/12, 16+27/2.0-8-6.25/12}, {1+2+11.7/12, 16+27/2.0-8-4-6.25/12}, {1, 16+27/2.0-8-4-6.25/12}};
//		fig3.addData(edge, Color.black);
		
		
		
		
		//Velocity
//
		FalconLinePlot fig33 = new FalconLinePlot(new double[][]{{0.0,0.0}});
		fig33.yGridOn();
		fig33.xGridOn();
		fig33.setYLabel("Position (ft/time)");
		fig33.setXLabel("time (seconds)");
		fig33.setTitle("Pos Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig33.addData(quinticPath.time,quinticPath.leftPos, Color.magenta);
		fig33.addData(quinticPath.time,quinticPath.rightPos, Color.blue);
		
		FalconLinePlot fig44 = new FalconLinePlot(new double[][]{{0.0,0.0}});
		fig44.yGridOn();
		fig44.xGridOn();
		fig44.setYLabel("heading (degt/time)");
		fig44.setXLabel("time (seconds)");
		fig44.setTitle("Pos Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig44.addData(quinticPath.time,quinticPath.heading, Color.green);
		
		
		
				FalconLinePlot fig4 = new FalconLinePlot(new double[][]{{0.0,0.0}});
				fig4.yGridOn();
				fig4.xGridOn();
				fig4.setYLabel("Velocity (ft/sec)");
				fig4.setXLabel("time (seconds)");
				fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
				fig4.addData(quinticPath.rightVelocity, Color.magenta);
				fig4.addData(quinticPath.leftVelocity, Color.cyan);


				FalconLinePlot fig5 = new FalconLinePlot(new double[][]{{0.0,0.0}});
				fig5.yGridOn();
				fig5.xGridOn();
				fig5.setYLabel("accel (ft^2/sec)");
				fig5.setXLabel("time (seconds)");
				fig5.setTitle("Acceleration Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
				fig5.addData(quinticPath.rightAccel, Color.magenta);
				fig5.addData(quinticPath.leftAccel, Color.cyan);
				

		
	}
	
	
	
	private QuinticTrajectory(double[][] path)
	{
		
		this.origPath = doubleArrayCopy(path);
		
		config = new TrajectoryGenerator.Config();
	    config.dt = .02;
//	    config.max_acc = 8.0*12;
//	    config.max_jerk = 30.0*12;
//	    config.max_vel = 10.0*12;
	    
	    config.max_vel = 3.0*12;
	    config.max_acc = 3.0*12;
	    config.max_jerk = 30.0*12;
	    
	    
	}
	
	public QuinticTrajectory(String filename, double[][] path, boolean reverse)
	{
		this(filename, path);

		if(reverse)
			this.invert();
	}

	public QuinticTrajectory(String filename, double[][] path)
	{
		this(path);

		//checkfile also calls calculate
		checkFileExist(filename);


		
	}

	public void plotPath()
	{
		FalconLinePlot fig4 = new FalconLinePlot(new double[]{0.0});
		fig4.yGridOn();
		fig4.xGridOn();
		fig4.setYLabel("Position (inch)");
		fig4.setXLabel("time (seconds)");
		fig4.setTitle("Position Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig4.addData(this.rightPos, Color.magenta);
		fig4.addData(this.leftPos, Color.cyan);

		FalconLinePlot fig5 = new FalconLinePlot(new double[]{0.0});
		fig5.yGridOn();
		fig5.xGridOn();
		fig5.setYLabel("Velocity (ft/sec)");
		fig5.setXLabel("time (seconds)");
		fig5.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig5.addData(this.rightVel, Color.magenta);
		fig5.addData(this.leftVel, Color.cyan);

	}
	
	private void makeFile(String Filename) {
		try {
			File file = new File(directory); ///home/lvuser/Paths
			if (!file.exists()) {
				if (file.mkdir()) {
					System.out.println("Path directory is created!");
				} else {
					System.out.println("Failed to create path directory!");
				}
			}
			log = new PrintWriter(directory + Filename);
			log.println(this.traj.getNumSegments());
			for(int i = 0; i<this.traj.getNumSegments(); i++)
				log.println(this.leftPos[i] +","+this.rightPos[i] +","+ this.leftVel[i] +"," + this.rightVel[i] +"," + this.heading[i]);
			log.flush();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
	
	
	private void checkFileExist(String filename) 
	{
		String line = "";
		
		try 
		{
				File file = new File(directory + filename); ///home/lvuser/Paths
				if (!file.exists()) 
				{
					calculate();
					makeFile(filename);
					System.out.println("Printed File! " + directory+filename);
				}
				else
				{
					boolean isFirstLine = true;
					int filecounter = 0;
					FileReader fr = new FileReader(directory + filename);
					String[] values;	
					BufferedReader br = new BufferedReader(fr);
					
					while((line = br.readLine()) != null)
					{
						//System.out.println(line);
						values = line.split(",");
					
					if(isFirstLine) 
					{
						this.leftPos = new double[Integer.parseInt(values[0])];
						this.rightPos = new double[Integer.parseInt(values[0])];
						this.leftVel = new double[Integer.parseInt(values[0])];
						this.rightVel = new double[Integer.parseInt(values[0])];
						this.heading = new double[Integer.parseInt(values[0])];
						isFirstLine = false;
					}
					else 
					{
						this.leftPos[filecounter] = Double.parseDouble(values[0]) ;
						this.rightPos[filecounter] =  Double.parseDouble(values[1]);
						this.leftVel[filecounter] = Double.parseDouble(values[2]) ;
						this.rightVel[filecounter] =  Double.parseDouble(values[3]);
						this.heading[filecounter] = Double.parseDouble(values[4]);
						
						filecounter++;
					}	
				}
					
					br.close();
					System.out.println("Trajectory Read from file " +directory+filename);
//					System.out.println(this.leftVel.length);
//					for(int i=0; i<this.leftVel.length; i++)
//					{
//						System.out.println(this.leftVel[i] +"," + this.rightVel[i] +"," + this.getHeadingDeg()[i]);
//						System.out.flush();
//					}
				
				}
			}		
			catch (FileNotFoundException e) 
			{
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			catch (IOException e)
			{
				e.printStackTrace();
				
			}
	}
	

	public QuinticTrajectory(double[][] path, double maxVel, double maxAccel)
	{
		
		this(path);
	    config.max_vel = maxVel;
	    config.max_vel = maxAccel;
	}

	
	public void calculate()
	{
		long start = System.nanoTime();
		//Calculate Total Arc Length Using Distance
		quinticSplines(origPath);
		
		//calculate total distance
		for (int i = 0; i < splines.length; ++i)
			this.totalSplineLength += splines[i].arc_length_;
		
		

		// Generate a smooth trajectory over the total distance.
	    this.traj = TrajectoryGenerator.generate(config,
	            TrajectoryGenerator.SCurvesStrategy, 0.0, this.origPath[0][2],
	            this.totalSplineLength, 0.0, this.origPath[0][2]);
	   

	    long end = System.nanoTime();
//	    System.out.println("Final time is " + (end - start)/1e9 + " seconds");
//	    
//	    System.out.println("Total Length = " + this.totalSplineLength);
//		System.out.println("Traj num segments=" + this.traj.getNumSegments() );
//	    
	    
	    fixHeadings();
	   
	    leftRightTraj = makeLeftAndRightTrajectories(traj, wheelSpacing);
	    
	    
	    
//	    System.out.println("Left = " + leftRightTraj.left.getNumSegments());
	    
	    copyWheelPaths();
	    
//	    print(this.leftPath);
//	    print(this.leftVelocity);
	    
		
	}
	
//	public static void print(double[] path)
//	{
//		System.out.println("X: \t Y:");
//
//		for(double u: path)
//			System.out.println(u);
//	}



	/**
	 * Prints Cartesian Coordinates to the System Output as Column Vectors in the Form X	Y
	 * @param path
	 */
//	public static void print(double[][] path)
//	{
//		System.out.println("X: \t Y:");
//
//		for(double[] u: path)
//			System.out.println(u[0]+ "\t" +u[1]);
//	}

	/**
	 * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array
	 * 
	 * BigO: Order N x M
	 * @param arr
	 * @return
	 */
	public static double[][] doubleArrayCopy(double[][] arr)
	{

		//size first dimension of array
		double[][] temp = new double[arr.length][arr[0].length];

		for(int i=0; i<arr.length; i++)
		{
			//Resize second dimension of array
			temp[i] = new double[arr[i].length];

			//Copy Contents
			for(int j=0; j<arr[i].length; j++)
				temp[i][j] = arr[i][j];
		}

		return temp;

	}
	
	  public Spline[] quinticSplines(double[][] path) {
	    if (path.length < 2) {
	      return null;
	    }

	    // Compute the total length of the path by creating splines for each pair
	    // of waypoints.
	    this.splines = new Spline[path.length - 1];
	    double[] spline_lengths = new double[splines.length];
	    
	    double total_distance = 0;
	    
	    for (int i = 0; i < splines.length; ++i) 
	    {
	      splines[i] = new Spline();
	      if (!Spline.reticulateSplines(path[i][0],path[i][1],path[i][2],
	    		  path[i+1][0],path[i+1][1],path[i+1][2], splines[i])) {
	        return null;
	      }
	      spline_lengths[i] = splines[i].calculateLength();
	      total_distance += spline_lengths[i];
	    }
	    
	    return splines;
	

}
	  private void fixHeadings()
	  {
	  // Assign headings based on the splines.
	    int cur_spline = 0;
	    double cur_spline_start_pos = 0;
	    double length_of_splines_finished = 0;
	    for (int i = 0; i < traj.getNumSegments(); ++i) {
	      double cur_pos = traj.getSegment(i).pos;

	      boolean found_spline = false;
	      while (!found_spline) {
	        double cur_pos_relative = cur_pos - cur_spline_start_pos;
	        if (cur_pos_relative <= this.splines[cur_spline].arc_length_) {
	          double percentage = splines[cur_spline].getPercentageForDistance(
	                  cur_pos_relative);
	          traj.getSegment(i).heading = splines[cur_spline].angleAt(percentage);
	          double[] coords = splines[cur_spline].getXandY(percentage);
	          traj.getSegment(i).x = coords[0];
	          traj.getSegment(i).y = coords[1];
	          found_spline = true;
	        } else if (cur_spline < splines.length - 1) {
	          length_of_splines_finished += this.splines[cur_spline].arc_length_;
	          cur_spline_start_pos = length_of_splines_finished;
	          ++cur_spline;
	        } else {
	          traj.getSegment(i).heading = splines[splines.length - 1].angleAt(1.0);
	          double[] coords = splines[splines.length - 1].getXandY(1.0);
	          traj.getSegment(i).x = coords[0];
	          traj.getSegment(i).y = coords[1];
	          found_spline = true;
	        }
	      }
	    }

	  
}
	  
	  /**
	   * Generate left and right wheel trajectories from a reference.
	   *
	   * @param input The reference trajectory.
	   * @param wheelbase_width The center-to-center distance between the left and
	   * right sides.
	   * @return [0] is left, [1] is right
	   */
	  static Trajectory.Pair makeLeftAndRightTrajectories(Trajectory input,
	          double wheelbase_width) {
	    Trajectory[] output = new Trajectory[2];
	    output[0] = input.copy();
	    output[1] = input.copy();
	    Trajectory left = output[0];
	    Trajectory right = output[1];

	    for (int i = 0; i < input.getNumSegments(); ++i) {
	      Trajectory.Segment current = input.getSegment(i);
	      double cos_angle = Math.cos(current.heading);
	      double sin_angle = Math.sin(current.heading);

	      Trajectory.Segment s_left = left.getSegment(i);
	      s_left.x = current.x - wheelbase_width / 2 * sin_angle;
	      s_left.y = current.y + wheelbase_width / 2 * cos_angle;
	      if (i > 0) {
	        // Get distance between current and last segment
	        double dist = Math.sqrt((s_left.x - left.getSegment(i - 1).x)
	                * (s_left.x - left.getSegment(i - 1).x)
	                + (s_left.y - left.getSegment(i - 1).y)
	                * (s_left.y - left.getSegment(i - 1).y));
	        s_left.pos = left.getSegment(i - 1).pos + dist;
	        s_left.vel = dist / s_left.dt;
	        s_left.acc = (s_left.vel - left.getSegment(i - 1).vel) / s_left.dt;
	        s_left.jerk = (s_left.acc - left.getSegment(i - 1).acc) / s_left.dt;
	      }

	      Trajectory.Segment s_right = right.getSegment(i);
	      s_right.x = current.x + wheelbase_width / 2 * sin_angle;
	      s_right.y = current.y - wheelbase_width / 2 * cos_angle;
	      if (i > 0) {
	        // Get distance between current and last segment
	        double dist = Math.sqrt((s_right.x - right.getSegment(i - 1).x)
	                * (s_right.x - right.getSegment(i - 1).x)
	                + (s_right.y - right.getSegment(i - 1).y)
	                * (s_right.y - right.getSegment(i - 1).y));
	        s_right.pos = right.getSegment(i - 1).pos + dist;
	        s_right.vel = dist / s_right.dt;
	        s_right.acc = (s_right.vel - right.getSegment(i - 1).vel) / s_right.dt;
	        s_right.jerk = (s_right.acc - right.getSegment(i - 1).acc) / s_right.dt;
	      }
	    }

	    return new Trajectory.Pair(output[0], output[1]);
	  }
	  
	  
	  
	  void copyWheelPaths()
	  {
		  this.leftPath = new double[this.leftRightTraj.left.getNumSegments()][2];
		  this.rightPath = new double[this.leftRightTraj.right.getNumSegments()][2];
		  this.leftVelocity = new double[this.leftRightTraj.left.getNumSegments()][2];
		  this.rightVelocity = new double[this.leftRightTraj.right.getNumSegments()][2];
		  this.leftAccel = new double[this.leftRightTraj.left.getNumSegments()][2];
		  this.rightAccel = new double[this.leftRightTraj.right.getNumSegments()][2];
		  this.leftJerk = new double[this.leftRightTraj.left.getNumSegments()][2];
		  this.rightJerk = new double[this.leftRightTraj.right.getNumSegments()][2];
		  
		  this.leftVel = new double[this.leftRightTraj.right.getNumSegments()];
		  this.rightVel =  new double[this.leftRightTraj.right.getNumSegments()];
		  this.leftPos = new double[this.leftRightTraj.right.getNumSegments()];
		  this.rightPos =  new double[this.leftRightTraj.right.getNumSegments()];
		  this.heading =  new double[this.leftRightTraj.right.getNumSegments()];
		  this.time =  new double[this.leftRightTraj.right.getNumSegments()];
		  this.leftAcc = new double[this.leftRightTraj.right.getNumSegments()];
		  this.rightAcc = new double[this.leftRightTraj.right.getNumSegments()];
		  
		  //copy left
		  for( int i =0; i < this.leftRightTraj.left.getNumSegments(); i++)
		  {
			  this.leftPath[i][0] = this.leftRightTraj.left.getSegment(i).x;
			  this.leftPath[i][1] = this.leftRightTraj.left.getSegment(i).y;
			  this.rightPath[i][0] = this.leftRightTraj.right.getSegment(i).x;
			  this.rightPath[i][1] = this.leftRightTraj.right.getSegment(i).y;
			  
			  this.leftVelocity[i][0] = this.leftRightTraj.left.getSegment(i).dt*i;
			  this.leftVelocity[i][1] = this.leftRightTraj.left.getSegment(i).vel;
			  this.rightVelocity[i][0] = this.leftRightTraj.right.getSegment(i).dt*i;
			  this.rightVelocity[i][1] = this.leftRightTraj.right.getSegment(i).vel;
			  
			  
			  this.leftPos[i] = this.leftRightTraj.left.getSegment(i).pos;
			  this.rightPos[i] = this.leftRightTraj.right.getSegment(i).pos;
			  this.time[i] = this.leftRightTraj.left.getSegment(i).dt*i;
			  
			  this.leftVel[i] = this.leftVelocity[i][1];
			  this.rightVel[i] = this.rightVelocity[i][1];
			  this.heading[i] =	(ChezyMath.boundAngleNeg180to180Degrees(360-this.traj.segments_[i].heading*180/Math.PI));
			  
			  
			  this.leftAccel[i][0] = this.leftRightTraj.left.getSegment(i).dt*i;
			  this.leftAccel[i][1] = this.leftRightTraj.left.getSegment(i).acc;
			  this.rightAccel[i][0] = this.leftRightTraj.right.getSegment(i).dt*i;
			  this.rightAccel[i][1] = this.leftRightTraj.right.getSegment(i).acc;
			  
			  this.leftJerk[i][0] = this.leftRightTraj.left.getSegment(i).dt*i;
			  this.leftJerk[i][1] = this.leftRightTraj.left.getSegment(i).jerk;
			  this.rightJerk[i][0] = this.leftRightTraj.right.getSegment(i).dt*i;
			  this.rightJerk[i][1] = this.leftRightTraj.right.getSegment(i).jerk;

			  this.leftAcc[i] = this.leftAccel[i][1];
			  this.rightAcc[i] = this.rightAccel[i][1];
			  
		  }
	  }
	  
	  public double[] getLeftPos()
	  {
		 
			  return this.leftPos;
	  }
	  
	  public double[] getRightPos()
	  {
		 
		  return this.rightPos;
	  }
	  
	  public double[] getLeftVel()
	  {
		 
			  return this.leftVel;
	  }
	  
	  public double[] getRightVel()
	  {
		 
		  return this.rightVel;
	  }
	  
	  public double[] getHeadingDeg()
	  {
		  return this.heading;
	  }

	  public void invert()
	  {
		for (int x=0; x<this.leftVel.length-1; x++)
			System.out.println("leftVel:" + this.leftVel[x]);

		int i = heading.length-1;

		  //Invert arrays
		  double[] temp_h = new double[heading.length];
		  double[] temp_lv = new double[heading.length];
		  double[] temp_rv = new double[heading.length];
		  double[] temp_rp = new double[heading.length];
		  double[] temp_lp = new double[heading.length];
		  double[] temp_la = new double[heading.length];
		  double[] temp_ra = new double[heading.length];

		//inverting all arrays
		for(int j=i; j>=0; j--)
		{
			System.out.println("i:" + i + ", j:" + j + ", i-j:" + (i-j));
			
			temp_h[j]= heading[i-j];
			temp_lv[j]= -leftVel[i-j];
			temp_rv[j]= -rightVel[i-j];
			temp_rp[j]= rightPos[i-j];
			temp_lp[j]= leftPos[i-j];
			temp_la[j] = -leftAcc[i-j];
			temp_ra[j] = -rightAcc[i-j];
			
			
		}

		  
		double temp_lp_zero = temp_lp[0];
		double temp_rp_zero = temp_rp[0];


		//subtracting first element from all pos elements
		//and making velocities negative
		for(int j=0; j<heading.length; j++)
		{
			
			temp_lp[j]=temp_lp[j]-temp_lp_zero;
			temp_rp[j]=temp_rp[j]-temp_rp_zero;
		}

		this.heading = temp_h;
		this.leftPos =temp_lp;
		this.rightPos =temp_rp;
		this.leftVel =temp_lv;
		this.rightVel =temp_rv;
		this.leftAcc = temp_la;
		this.rightAcc = temp_ra;
		

		for (int x=0; x<this.heading.length-1; x++)
			System.out.println("heading:" + this.heading[x]);

		for (int x=0; x<this.leftVel.length-1; x++)
			System.out.println("leftVel:" + this.leftVel[x]);



	  }


	  
	  
	  
	}
	  
