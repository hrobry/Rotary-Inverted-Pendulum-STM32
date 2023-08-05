

double _max = 1024;
 double _min=0;
	static double _pre_error;
	double _integral=0;
	double derivative;
	uint32_t setTime = 100;
	uint32_t now,timechange;
	static uint32_t lasttime;
	double error;
 double Pout,Iout,Dout;
	double Doutput;
 struct calculate calc;

 bool kierunek = false;

 uint32_t output;


uint32_t calculatePID( int input, int setpoint ,double P, double I, double D ) {

    now = HAL_GetTick ();
	timechange = (now - lasttime);
	if(timechange >=setTime)
	{
timechange = timechange / 1000;
		     error = setpoint - input;

		    // Proportional term
		     Pout = P * error;

		    // Integral term
		     _integral += error * setTime;
		     Iout = I * _integral;

		    // Derivative term
		     derivative = (error - _pre_error) / setTime;
		     Dout = D * derivative;

		    // Calculate total output
		      Doutput = Pout + Iout + Dout; //110+1000+0
		    	if (Doutput<0)
						{

							kierunek = true;
						}else if(Doutput>0){


							kierunek = false;
						}

		    if( Doutput < _min )
		        {Doutput =Doutput * -1;}
		    else if( Doutput > _max )
		    { output = _max;}






		    	output=(uint32_t)Doutput;




		    // Save error to previous error
		    _pre_error = error;
			lasttime = now;




	}
return output;
}
volatile void  engineControlBasicFree()
{
//static struct pidy pides={0.5,0.02,0.02};
//static struct calculate calc;

 static int encoderCounterUp2=0;
 static int ondrive = 0;

 encoderCounterUp2 = htim3.Instance->CNT;
 if(encoderCounterUp2>595 && encoderCounterUp2 <605){
	 ondrive =1;

 }

    if (encoderCounterUp2>200 && encoderCounterUp2 <1000 && ondrive==1)
    {


    calculatePID( encoderCounterUp2, 600 ,1.9,0.005,0);
 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
  //  torque = output;

		 if(kierunek)
		{


		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_SET);//podlaczenie sterowania silnikiem
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);//podlaczenie sterowania silnikiem
		// HAL_Delay(10);
		// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
		}else{



		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_SET);
	//	HAL_Delay(10);
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
		 }

     }else{


      output=0;
      ondrive = 0;
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);

     }



}


double _max = 1024;
 double _min=0;
	static double _pre_error;
	double _integral=0;
	double derivative;
	uint32_t setTime = 100;
	uint32_t now,timechange;
	static uint32_t lasttime;
	double error;
 double Pout,Iout,Dout;
	double Doutput;
 struct calculate calc;

 bool kierunek = false;

 uint32_t output;


uint32_t calculatePID( int input, int setpoint ,double P, double I, double D ) {

    now = HAL_GetTick ();
	timechange = (now - lasttime);
	if(timechange >=setTime)
	{
timechange = timechange / 1000;
		     error = setpoint - input;

		    // Proportional term
		     Pout = P * error;

		    // Integral term
		     _integral += error * setTime;
		     Iout = I * _integral;

		    // Derivative term
		     derivative = (error - _pre_error) / setTime;
		     Dout = D * derivative;

		    // Calculate total output
		      Doutput = Pout + Iout + Dout; //110+1000+0
		    	if (Doutput<0)
						{

							kierunek = true;
						}else if(Doutput>0){


							kierunek = false;
						}

		    if( Doutput < _min )
		        {Doutput =Doutput * -1;}
		    else if( Doutput > _max )
		    { output = _max;}






		    	output=(uint32_t)Doutput;




		    // Save error to previous error
		    _pre_error = error;
			lasttime = now;




	}
return output;
}
volatile struct calculate engineControlBasicFree( )
{
//static struct pidy pides={0.5,0.02,0.02};
//static struct calculate calc;

 static int encoderCounterUp2=0;

 encoderCounterUp2 = htim3.Instance->CNT;

    if (encoderCounterUp2>400 && encoderCounterUp2 <800)
    {


    calculatePID( encoderCounterUp2, 600 ,2.8,0.0001,1.9);



		 if(kierunek)
		{


		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_SET);//podlaczenie sterowania silnikiem
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);//podlaczenie sterowania silnikiem
		 HAL_Delay(10);
		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
		}else{



		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_SET);
		HAL_Delay(10);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
		 }

     }else{
      output=0;
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);

	}

return calc;


}

