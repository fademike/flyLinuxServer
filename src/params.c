
#include "params.h"
#include "mavlink.h"

#define HAL_OK 0
#define HAL_ERROR -1

// params for debug
struct param_struct t_param[PARAM_ALL] = {	
	[PARAM_FLASH] = {"flash_params", {.FLOAT=0}, MAV_PARAM_TYPE_REAL32},
	[PARAM_FL_KP] = {"f_kp", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_FL_KI] = {"f_ki", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_PR_P] = {"pid_pr_p", {.FLOAT=60.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_PR_D] = {"pid_pr_d", {.FLOAT=2000.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_PR_I] = {"pid_pr_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_MUX_PR_CHAN] = {"mux_pr_chan", {.FLOAT=50.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_MUX_Y_CHAN] = {"mux_y_chan", {.FLOAT=1.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_Y_P] = {"pid_y_p", {.FLOAT=-60.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_Y_D] = {"pid_y_d", {.FLOAT=-4000.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_Y_I] = {"pid_y_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_KP_ARM] = {"f_kp_arm", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_P_ORIENTATION] = {"p_orientation", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32}};

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