function updateDisplayValue(value) {

	document.getElementById('voltage').innerHTML = (value.voltage*11*(1.08)/1/1000).toFixed(3);
    //document.getElementById('str_1_0').innerHTML = (value.voltage*11*(1.05)/1/1000).toFixed(3);
    //document.getElementById('str_2_0').innerHTML = "*11*(1.05)/1";
    document.getElementById('current').innerHTML = value.current;
    document.getElementById('mainMotor').innerHTML = value.mainMotor +"("+value.mainMotorValue+")";
    document.getElementById('secondMotor').innerHTML = value.secondMotor +"("+value.secondMotorValue+")";
    //document.getElementById('mainMotorValue').innerHTML = value.mainMotorValue;
    document.getElementById('servo1').innerHTML = value.servo1 +"("+value.servo1Value+")";
    //document.getElementById('servo1Value').innerHTML = value.servo1Value;
    document.getElementById('servo2').innerHTML = value.servo2 +"("+value.servo2Value+")";
    //document.getElementById('servo2Value').innerHTML = value.servo2Value;
    document.getElementById('indication').innerHTML = value.indication;
    document.getElementById('temp').innerHTML = (value.temp1/100).toFixed(1) + ", "
                                             + (value.temp2/100).toFixed(1) + String.fromCharCode(0xB0)+"C";
    //document.getElementById('temp').innerHTML = value.temp2;
    document.getElementById('press').innerHTML = value.press+"(" + (value.press/133).toFixed(2) + ")";//"mmHg)";
	

    document.getElementById('PRY').innerHTML = (value.pitch/100).toFixed(0)+", "+
                                                    (value.roll/100).toFixed(0)+", "+
                                                    (value.yaw/100).toFixed(0);

    
    document.getElementById('GPS1').innerHTML = value.time + ", " + value.day;
    document.getElementById('GPS2').innerHTML = value.longer + ", " + value.latit;
    document.getElementById('GPS3').innerHTML = value.longer_p + ", " + value.latit_p;
    document.getElementById('GPS4').innerHTML = value.speed + ", " + value.hight;
    document.getElementById('GPS5').innerHTML = value.N_sp + ", " + value.autent + ", " + value.dir_deg;


var value1=value.yaw/100;       //yaw
var value2=value.roll/100;       //roll
var value3=value.pitch/100;       //pitch

    image_yaw = document.getElementById('compass_array');
    image_roll = document.getElementById('air_roll');
    image_pitch = document.getElementById('air_pitch');
	image_yaw.style.transform = "rotate("+value1+"deg)";
	image_roll.style.transform = "rotate("+value2+"deg)";
    image_pitch.style.top = (-220+(value3/180*600/2)) + "px";


	document.getElementById('typeManage').innerHTML = value.tx_typeManage;
	document.getElementById('CmdTimeout').innerHTML = value.tx_CmdTimeout;
	document.getElementById('tx_mainMotor').innerHTML = value.tx_mainMotor +"("+value.tx_mainMotorValue+")";
	document.getElementById('tx_secondMotor').innerHTML = value.tx_secondMotor +"("+value.tx_secondMotorValue+")";
	document.getElementById('tx_servo1').innerHTML = value.tx_servo1 +"("+value.tx_servo1Value+")";
	document.getElementById('tx_servo2').innerHTML = value.tx_servo2 +"("+value.tx_servo2Value+")";
	document.getElementById('tx_indication').innerHTML = value.tx_indication;
	document.getElementById('Stab_RPY').innerHTML = value.tx_StabRoll+","+value.tx_StabPitch+","+value.tx_StabYaw;
	document.getElementById('StabA_PR').innerHTML = value.tx_StabAngelPitch+","+value.tx_StabAngelRoll;
	document.getElementById('Stab_KM').innerHTML = value.tx_StabKoeffManage1+","+value.tx_StabKoeffManage2;
	document.getElementById('Stab_K_CH').innerHTML = value.tx_StabKoeffCh1+","+value.tx_StabKoeffCh2;
	document.getElementById('pos_rev_a1_a2').innerHTML = value.tx_pos_rev+","+value.tx_pos_ang1+","+value.tx_pos_ang2;
	document.getElementById('StabServoMax').innerHTML = value.tx_StabServoMax1+","+value.tx_StabServoMax2;
	document.getElementById('math_K_angle_bias').innerHTML = value.tx_math_K_angle+","+value.tx_math_K_bias;
	document.getElementById('m_K_meas_gyro').innerHTML = value.tx_math_K_measure+","+value.tx_math_gyroRate;
	
}

function getNextRandomValue(callback) {
    jQuery.ajax('/api/datasensor', {
        dataType: 'json',
        success: function(response) {
            console.log(response);
            callback(response);
        },
    });
}

function autoUpdate() {
	
    getNextRandomValue(function(value) {
		
        updateDisplayValue(value);
        setTimeout(autoUpdate, 250);
    });
}

autoUpdate();
