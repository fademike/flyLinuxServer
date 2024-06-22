
#include "params.h"
#include "mavlink.h"

#define HAL_OK 0
#define HAL_ERROR -1

#define PARAM_ALL 13

struct param_struct t_param[PARAM_ALL] = {	{"flash_params", {.FLOAT=0}, MAV_PARAM_TYPE_REAL32},
											{"f_kp", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},
											{"f_ki", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
											{"pid_pr_p", {.FLOAT=60.0f}, MAV_PARAM_TYPE_REAL32},
											{"pid_pr_d", {.FLOAT=2000.0f}, MAV_PARAM_TYPE_REAL32},
											{"pid_pr_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
											{"mux_pr_chan", {.FLOAT=50.0f}, MAV_PARAM_TYPE_REAL32},
											{"mux_y_chan", {.FLOAT=1.0f}, MAV_PARAM_TYPE_REAL32},
											{"pid_y_p", {.FLOAT=-60.0f}, MAV_PARAM_TYPE_REAL32},
											{"pid_y_d", {.FLOAT=-4000.0f}, MAV_PARAM_TYPE_REAL32},
											{"pid_y_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
											{"f_kp_arm", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
											{"p_orientation", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32}};


float params_GetMemValue(int n){
		return t_param[n].param_value.FLOAT;
}


int params_getSize(void){
	return PARAM_ALL;
}
int params_setValue(int n, union param_value value, int type){
	int t = n;
	if ((t<0) || (t>=PARAM_ALL)) return HAL_ERROR;
	if (type == MAV_PARAM_TYPE_REAL32) t_param[t].param_value.FLOAT = value.FLOAT;
	else if (type == MAV_PARAM_TYPE_INT8) t_param[t].param_value.INT = (int)value.INT;
	else t_param[t].param_value.FLOAT = value.FLOAT;

	return HAL_OK;
}
int params_getValue(int n, union param_value * value, int type){
	int t = n;
	if ((t<0) || (t>=PARAM_ALL)) return HAL_ERROR;
	if (type == MAV_PARAM_TYPE_REAL32) value->FLOAT = t_param[t].param_value.FLOAT;
	else if (type == MAV_PARAM_TYPE_INT8) value->FLOAT = t_param[t].param_value.INT;
	else  value->FLOAT = t_param[t].param_value.FLOAT;
	return HAL_OK;
}
int params_getParam(int n, struct param_struct * param){
	int t = n;
	if ((t<0) || (t>=PARAM_ALL)) return HAL_ERROR;
	*param = t_param[t];
	return HAL_OK;
}
int params_getIndexById(char * id){
	int i=0;// flag of existence param
	for (i=0;i<PARAM_ALL; i++){									// find rx param in param base
		if (strcmp(id, t_param[i].param_id)==0){				// if rx param == param base
			printf("strcmp %s,  %d \n\r", t_param[i].param_id, i);
			return i+1;
		}
	}
	return 0;
}