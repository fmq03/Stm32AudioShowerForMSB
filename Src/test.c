#include "main.c"
void testOLED_swim(){
	for(u8 i=0;i<64;++i){
		u8 val=i;
		for(u8 j=0;j<128;++j){
			if(val>=64)val=0;
			OLED_DrawLine(j,0,j,63,0);
			OLED_DrawLine(j,0,j,val,1);
			++val;
			//OLED_Refresh();
		}
		OLED_Refresh();
	}
}
void testOLED_line(){
	for(u8 i=0;i<64;++i){
		OLED_DrawPoint(64,i,1);
		OLED_Refresh();
	}
	OLED_Clear();
	for(u8 i=1;i<64;++i){
		OLED_DrawPoint(64,63-i,1);
		OLED_Refresh();
	}
	OLED_Clear();
}
void testOLED_sin(){
	static int cur=0;
	OLED_Clear();
	for(u8 i=0;i<127;++i){
		u16 x=i+cur*50;
		float32_t xal;
		xal=x/10.0;
		OLED_DrawPoint(i,(int)(arm_sin_f32(xal)*20.0+31.0),1);
		
	}
	OLED_Refresh();
	//
	cur++;
	if(cur>=127)cur=0;
}
void testADC(){
	static u8 col=1;
	static u16 lastval=30;
	u16 adcvalue=65535;
	//HAL_Delay(1000);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,15);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
		adcvalue=HAL_ADC_GetValue(&hadc1);
		//OLED_Clear();
	//	OLED_ShowString(0,0,"ok",12,1);
		for(u8 i=col;i<col+6;++i)
		OLED_DrawLine(i,0,i,63,0);
		
		u16 val=(u16)(adcvalue/1023.0*30)+20;
		OLED_DrawPoint(col,val,1);
		if(lastval<val)OLED_DrawLine(col-1,lastval,col,val,1); else OLED_DrawLine(col,val,col-1,lastval,1);

		lastval=val;
		
	//	USARTPrintf("%d\n",adcvalue);
		
		OLED_Refresh();
		col++;
		if(col>=143)col=1;
	}

}
void testFFT(){
	
	arm_rfft_fast_instance_f32 S;
	for(u32 i=0;i<FFTNUM;++i)FFTin[i]=arrValue[i]/512.0;
	if(arm_rfft_fast_init_f32(&S,FFTNUM)==ARM_MATH_ARGUMENT_ERROR)Ding();
	arm_rfft_fast_f32(&S, FFTin, FFTout, 0);
	FFTout[0]=FFTout[1]=0;
	arm_cmplx_mag_f32(FFTout, FFTmag, FFTNUM/2); 
	u16 t=0;
	OLED_Clear();
	for(u32 i=0;i<128;++i){
		if(t==128){
			t=0;
			OLED_Refresh();
			HAL_Delay(500);
			OLED_Clear();
		}
		OLED_DrawLine(t,0,t,(u32)FFTmag[i],1);
		++t;
	}
	//OLED_DrawLine(0,63,127,63,1);
	OLED_Refresh();
	return;
}