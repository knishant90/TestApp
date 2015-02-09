package com.example.testapp;
import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.text.DateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.LinkedList;
import java.util.Locale;
 
import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.text.format.Time;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;


public class MainActivity extends Activity implements SensorEventListener{
	 
		//private Socket client;
		private PrintWriter printwriter;
		private EditText textField;
		private Button button;
		private String messsage;
		private SensorManager msensormanager;
		private Sensor msensor;
		private float ux,vx=0;
		private float previousAccelerationValue = 0; 
		private float previousVelocityValue = 0;
		private float displacement=0;
		long previoustimestamp=0;
		float dt,currentVelocityValue;
		float prevvalues[] = new float[3];
	    int count =0;
	    float averageBleeding =0;
	    int positiveStepCount = 0;
	    int negativeStepCount = 0;
	    int stepCount=0;
	    long previousRegionCrossingTime =0;
	    boolean flow=false;
	    boolean currentFlow = false;
	    boolean previousFlow = false;
	    float flowDifference = 0;
	    Time EndingTime;
	    float previousNegativeRegionCrossingTime;
	    float previousPositiveRegionCrossingTime;
	    KalmanFilter filter = new KalmanFilter(0);
	    Long startTime = (long) 0;
	    
	    //Cache for Acceleration values;
	    
	    ArrayList<Double> dataPoints = new ArrayList<Double>();
	    ArrayList<Long> timePoints = new ArrayList<Long>();
	    
		@Override
		protected void onCreate(Bundle savedInstanceState) {
			super.onCreate(savedInstanceState);
			setContentView(R.layout.main);
	 
			textField = (EditText) findViewById(R.id.editText1); // reference to the text field
			button = (Button) findViewById(R.id.button1); // reference to the send button
			//Accelerometer
			
			msensormanager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
			msensor = msensormanager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
			msensormanager.registerListener(this, msensor,SensorManager.SENSOR_DELAY_GAME);		
			//msensormanager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
			SendMessage test1 = new SendMessage();
			messsage = "asd";
			test1.execute();
			button.setTag(1);
			//Tag-1 refers to Inactive mode and Tag-2 refers to active mode.
			button.setOnClickListener(new View.OnClickListener(){
				public void onClick(View v) {
				final int currentMode = (Integer) button.getTag();
					if(currentMode == 1)
					{
						button.setTag(2);
						button.setText("Stop");
						currentVelocityValue = 0;
						
						previousVelocityValue = 0;
						previousAccelerationValue = 0;
						
						displacement =0;
						prevvalues[0] = 0;
						prevvalues[1] = 0;
						prevvalues[2] = 0;
						previoustimestamp =0;
						positiveStepCount = 0;		
						negativeStepCount = 0;
						stepCount =0;
						previousNegativeRegionCrossingTime = 0;
						previousPositiveRegionCrossingTime =0;
						previousRegionCrossingTime =0;
						filter = new KalmanFilter(0);
						startTime = System.currentTimeMillis();
						
						
					}
					
					else if(currentMode == 2)
					{
						double[] dataCache = new double[dataPoints.size()];
						long[] timeCache = new long[timePoints.size()];
						for (int i=0;i<dataPoints.size();i++)
						{
							dataCache[i] = dataPoints.get(i);
							timeCache[i] = timePoints.get(i);
						}
						Object[] cache = findPeaks(dataCache,timeCache);
						@SuppressWarnings("unchecked")
						ArrayList<Double> cacheSize = (ArrayList<Double>) cache[0];
						ArrayList<Long> cacheTime = (ArrayList<Long>) cache[1];
						textField.setText(String.valueOf(cacheSize.size()));
						
						messsage = cacheSize.toString() + " Times:" + cacheTime.toString();
						
						dataPoints.clear();
						button.setTag(1);
						button.setText("Start");
					}
					
				/*currentVelocityValue = 0;
				
				previousVelocityValue = 0;
				previousAccelerationValue = 0;
				
				displacement =0;
				prevvalues[0] = 0;
				prevvalues[1] = 0;
				prevvalues[2] = 0;
				previoustimestamp =0;
				positiveStepCount = 0;		
				negativeStepCount = 0;
				stepCount =0;
				previousNegativeRegionCrossingTime = 0;
				previousPositiveRegionCrossingTime =0;
				previousRegionCrossingTime =0;
				filter = new KalmanFilter(0);
				startTime = System.currentTimeMillis();*/
			}
		});
	}
		
		public static Object[] findPeaks(double[] dataPoints,long[] timePoints)
		{
			ArrayList<Double> peaks = new ArrayList<Double>();
			ArrayList<Long> peakTimes = new ArrayList<Long>();
			double[] dataPointsCopy = Arrays.copyOf(dataPoints, dataPoints.length);
			Arrays.sort(dataPointsCopy);
			double min = dataPointsCopy[0];
			double max = dataPointsCopy[dataPoints.length-1];
			double sel = (max-min)/4;
			double thresh = 0;
			int extrema = 1;
			
			//Finding Derivative
			double[] diff = new double[dataPoints.length-1];
			if(dataPoints.length >2)
			{
				for(int i=0;i<dataPoints.length-2;i++)
				{
					diff[i] = dataPoints[i+1]-dataPoints[i];
				}
			}
			
			//Finding where the derivatives change sign including endpoints
			ArrayList<Integer> indices = new ArrayList<Integer>();
			//indices.add(0);
			for(int i=0;i<diff.length-2;i++)
			{
				int k=1;
				if(diff[i+k]==0)
				{
					while(diff[i+k]==0)
					{
						k++;
					}
				}
				double sign = diff[i] * diff[i+k];
				i=i+k-1;
				if(sign <0)
				{
					indices.add(i+2);
				}
				if((diff[i] == 0) && (diff[i+1] ==0))
				{
					
				}
				/*else if(((diff[i]==0) || diff[i+1] ==0))
				{
					indices.add(i+2);
				}
				else if(((diff[i]==0) || diff[i+1] ==0) && (indices.contains(i-1)))
				{
					indices.add(i+2);
				}*/

			}
			indices.add(diff.length - 1);
			
			int minIndex = minInArrayAt(dataPoints,indices);
			double minMag = dataPoints[minIndex];
			double leftMin = minMag;
			int len = indices.size()-1;
			long minAtTime = timePoints[minIndex];
			//Original value at the indices
			ArrayList<Double> x = new ArrayList<Double>();
			x.add(dataPoints[0]);
			ArrayList<Long> xTime = new ArrayList<Long>();
			xTime.add(timePoints[0]);
			
			for(int i=0;i<len;i++)
			{
				x.add(dataPoints[indices.get(i)-1]);
				xTime.add(timePoints[indices.get(i)-1]);
			}
			x.add(dataPoints[dataPoints.length-1]);
			xTime.add(timePoints[dataPoints.length-1]);
			if(len>2)
			{
				double tempMag = minMag;
				boolean foundPeak = false;
				long tempTime = minAtTime;
				
				if((x.get(1) - x.get(0))<=0)
				{
					if(((x.get(1) - x.get(0))>=0 && (x.get(2) - x.get(1))>=0) || ((x.get(1) - x.get(0))<=0 && (x.get(2) - x.get(1))<=0))
					{
						x.remove(2);
						xTime.remove(2);
						indices.remove(2);
						len=len-1;
					}
				}
				else
				{
					if(((x.get(1) - x.get(0))>=0 && (x.get(2) - x.get(1))>=0) || ((x.get(1) - x.get(0))<=0 && (x.get(2) - x.get(1))<=0))
					{
						x.remove(1);
						xTime.remove(1);
						indices.remove(1);
						len=len-1;
					}
					
				}
				int ii=0;
				if(x.get(0) >= x.get(1))
				{
					ii=0;
				}
				else
				{
					ii=1;
				}
				int[] maxPeaks = new int[len/2];
				int[] peakLoc = new int[len/2];
				//TimeStamp[] peakTimes = new TimeStamp[len/2];
				double[] peakMag = new double[len/2];
				long[] peakTime = new long[len/2];
				int cInd=1;
				int tempLoc = ii;
				while(ii < len-1)
		        {
					ii = ii+1; // This is a peak
					// Reset peak finding if we had a peak and the next peak is bigger
					//   than the last or the left min was small enough to reset.
					if(foundPeak)
		            {
						tempMag = minMag;
						tempTime = minAtTime;
						foundPeak = false;
		            }
		        
					// Make sure we don't iterate past the length of our vector
					if(ii == len-1)
						{
							break; // We assign the last point differently out of the loop
						}
		        
		        
					//Found new peak that was lager than temp mag and selectivity larger
					//than the minimum to its left.
					if((x.get(ii-1) > tempMag) && (x.get(ii-1) > leftMin + sel))
					{
						tempLoc = ii-1;
						tempMag = x.get(ii-1);
						tempTime = xTime.get(ii - 1);
					}	
		        
					ii = ii+1; // Move onto the valley
					//Come down at least sel from peak
					if(!foundPeak && tempMag > sel + x.get(ii-1))
					{
						foundPeak = true; // We have found a peak
						leftMin = x.get(ii-1);
						peakLoc[cInd] = tempLoc; // Add peak to index
						peakMag[cInd] = tempMag;
						peakTime[cInd] = tempTime;
						cInd = cInd+1;
					}
					else if(x.get(ii-1) < leftMin) // New left minima
					{
						leftMin = x.get(ii-1);
					}
		        }
					// Check end point
				if(x.get(x.size()-1) > tempMag && x.get(x.size()-1) > leftMin + sel)
				{
					peakLoc[cInd] = len;
					peakMag[cInd] = x.get(x.size()-1);
					peakTime[cInd] = xTime.get(xTime.size()-1);
					cInd = cInd + 1;
				}
				else if(!foundPeak && tempMag > minMag) // Check if we still need to add the last point
				{
					peakLoc[cInd] = tempLoc;
					peakMag[cInd] = tempMag;
					peakTime[cInd] = tempTime;
					cInd = cInd + 1;
				}
				else if(!foundPeak) 
		        {
					if(tempMag > dataPoints[dataPoints.length - 1] + sel)
		            	peakLoc[cInd] = tempLoc;
						peakMag[cInd] = tempMag;
						peakTime[cInd] = tempTime;
						cInd = cInd + 1;
		        }
		        
				for(int i=0;i<peakLoc.length;i++)
				{
					if(peakMag[i]>1.5)
					{
						peaks.add(peakMag[i]);
						peakTimes.add(peakTime[i]);
					}
				}
				
				// Create output
		    //peakInds = ind(peakLoc(1:cInd-1));
		    //peakMags = peakMag(1:cInd-1);
				
			}
			Object[] response = new Object[2];
			response[0] = peaks;
			response[1] = peakTimes;
			//Response response1 =  response.modify(peakTimes,peaks);
			//ArrayList<ArrayList<Integer,Double>> t = new ArrayList<ArrayList<E>>(); 
			return response;
		}
		
		
		
		public static int minInArrayAt(double[] array,ArrayList<Integer> indices)
		{
			double min=99999;
			int minIndex = 0;
			for(int i = 0;i<indices.size()-1;i++)
			{
				if(array[indices.get(i)-1]<min)
				{
					min = array[indices.get(i)-1];
					minIndex = indices.get(i) - 1;
				}
			}
			return minIndex;
		}
		
		
		public void onSensorChanged(SensorEvent event) {
			float[] values = event.values;
			/*if(filter.startingTime == 0)
			{
				filter.setStartingTime(System.currentTimeMillis());
			}*/
			if((Integer)button.getTag() == 1)
			{
				startTime = 0l;
				
			}
			else
			{
				if(startTime ==0l)
				{
					startTime = System.currentTimeMillis();
				}
				//if(System.currentTimeMillis() - startTime< 10000)
				filter.filterValue(values[2]);
				//Adding acceleration values to peakFinder array cache 
				dataPoints.add((double) values[2]);
				timePoints.add(System.currentTimeMillis() - startTime);
				messsage = "TimeStamp: " +(System.currentTimeMillis() - startTime)/10  + ": OriginalA: " + values[2]+ ": KalmanA: " + filter.currentFilterValue() + ": Count: " + filter.getSteps() + ":TimeDifference: " + filter.lastCrossingTime();
				SendMessage test1 = new SendMessage();
				test1.execute();
				//else
				//{
				//	messsage = "Complete";
				//	SendMessage test1 = new SendMessage();
				//	test1.execute();
				//}
			}
			
				
		}
		
		class KalmanFilter
		{
			float P_last = 0; 
		    //the noise in the system 
		    float Q = (float) 0.022;
		    float R = (float) 0.617;
		     
		    float K; 
		    float P; 
		    float P_temp; 
		    float x_tempestimated; 
		    float x_estimated; 
		    float z_measured; //the 'noisy' value we measured
		    float z_real = 1; //the ideal value we wish to measure
		    float LastFlowDifference = 0;
		    float x_estimatedlast = 0;
			
		    int positiveStepCount = 0;
		    int negativeStepCount = 0;
		    float max,min =0;
		    int region=0;
		    float[] stepTimes = new float[100];
		    long previousPositiveRegionCrossingTime=0;
		    long previousNegativeRegionCrossingTime = 0;
		    Long startingTime = (long) 0;
		    int count =0;
		    float currentOriginalAcceleration;
			float sum_errorkalman = 0;
		    float sum_errormeasure = 0;
			
		    KalmanFilter(float initialValue)
			{
		    	filterValue(initialValue);
			}
			
		    void setStartingTime(Long value)
		    {
		    	startingTime = value;
		    }
		    
			void filterValue(float accelerationValue)
			{
				
				
				if((System.currentTimeMillis() - startingTime < 10000) || (startingTime ==0))
				{x_tempestimated = x_estimatedlast; 
			    P_temp = P_last + Q; 
														//calculate the Kalman gain 
			    K = (float) (P_temp * (1.0/(P_temp + R)));
			        //measure 
			    z_measured = accelerationValue; //the real measurement plus noise
			        //correct 
			    x_estimated = x_tempestimated + K * (z_measured - x_tempestimated);  
			    P = (1- K) * P_temp; 
			    sum_errorkalman += (int)(z_real - x_estimated); 
			    sum_errormeasure += (int)(z_real-z_measured); 
			         
			        //update our last's 
			    P_last = P; 
			    //averageBleeding+=x_estimated;
			    //averageBleeding = averageBleeding/count;
			    
			    countSteps();
			    x_estimatedlast = x_estimated;
			    currentOriginalAcceleration = accelerationValue;}
				else
				{
					
				}
			}

			void countSteps()
			{
				if(x_estimated >= 0)
				{
					region = 1;
				}
				
				else if(x_estimated <0)
				{
					region=2;
				}
				
				if(region==1)
				{
					min =0;
					if(max >= 0.2)
					{
						if(x_estimated > 0.2)
						{
							max = x_estimated;
						}
					}
					else if(max<0.2)
					{
						if(x_estimated >=0.2)
						{
							positiveStepCount++;
							setStartingTime(System.currentTimeMillis());
							max=x_estimated;
							if(previousPositiveRegionCrossingTime==0)
							{
								previousPositiveRegionCrossingTime = System.currentTimeMillis();
							}
							else
							{
								stepTimes[count] = System.currentTimeMillis() - previousPositiveRegionCrossingTime;
								//messsage = "Original Value: "+values[2]+ ":Positive Kalman Value: " + x_estimated +": Count: " + positiveStepCount + ": Time difference: " + Math.round((event.timestamp - previousPositiveRegionCrossingTime) / 1000000.0);	
								//messsage = "Original Value: "+values[2]+ ":Positive Kalman Value: " + x_estimated +": Step Count " + positiveStepCount + ": lastFlowDifference: " + LastFlowDifference;		
								previousPositiveRegionCrossingTime = System.currentTimeMillis();
								//SendMessage sendMessageTask = new SendMessage();
								//sendMessageTask.execute();
								
								count++;
							}
						}
						if(x_estimated > max)
						{
							max = x_estimated;
						}
					}
				}
				else if(region==2)
				{
					max=0;
					if(min <= -0.2)
					{
						if(x_estimated < -0.2)
						{
							min = x_estimated;
						}
					}
					else if(min>-0.2)
					{
						if(x_estimated <=-0.2)
						{
							negativeStepCount++;
							setStartingTime(System.currentTimeMillis());
							//if((negativeStepCount == 1) && (startingTime == null))
							//{
							//	startingTime = System.currentTimeMillis();
							//}
							min=x_estimated;
							if(previousNegativeRegionCrossingTime==0)
							{
								previousNegativeRegionCrossingTime = System.currentTimeMillis();
							}
							else
							{
								stepTimes[count] = System.currentTimeMillis() - previousNegativeRegionCrossingTime;
								//messsage = "Original Value: "+values[2]+ ":Negative Kalman Value: " + x_estimated +": Count: " + negativeStepCount + ": Time difference: " + (event.timestamp - previousNegativeRegionCrossingTime)/1000000.0;	
								//messsage = "Original Value: "+values[2]+ ":Negative Kalman Value: " + x_estimated +": Step Count " + negativeStepCount + ": lastFlowDifference: " + LastFlowDifference;	
								
								previousNegativeRegionCrossingTime = System.currentTimeMillis();
								//SendMessage sendMessageTask = new SendMessage();
								//sendMessageTask.execute();
								
								count++;
							}
						}
						if(x_estimated < min)
						{
							min = x_estimated;
						}
					}
				}
			}
			
			public int getSteps()
			{
				//return (int) ((positiveStepCount + negativeStepCount) /2);
				return (int) count/2;
			}
			
			public float lastCrossingTime()
			{
				if(count>0)
					return stepTimes[count-1]/1000;
				else
					return stepTimes[0]/1000;
			}
			
			public float currentFilterValue()
			{
				return x_estimatedlast;
			}
			
			public float currentRealValue()
			{
				return currentOriginalAcceleration;
			}
		}
		
		
	class SendMessage extends AsyncTask<Void, Void, Void> {
			
		@Override
		protected Void doInBackground(Void... params) {
			try {
				Socket client = new Socket("146.244.104.40", 1137); // connect to the server
				printwriter = new PrintWriter(client.getOutputStream(), true);
				printwriter.write(messsage); // write the message to output stream
				printwriter.flush();
				printwriter.close();
				client.close(); // closing the connection
 
			} catch (Exception e) {
				e.printStackTrace();
			
			}
			return null;
		}
			BufferedWriter a = null;
	}
 
		

		@Override
		public void onAccuracyChanged(Sensor arg0, int arg1) {
			// TODO Auto-generated method stub
			
		}
}

