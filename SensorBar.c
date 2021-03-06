#include <avr/io.h>

#include "Main.h"
#include "PIDfollower.h"
#include "Motor.h"



uint8_t sensors_min_reading[6];
uint8_t sensors_max_reading[6];
// formula is:
//
//    0*value0 + 1000*value1 + 2000*value2 + ...
//   --------------------------------------------
//         value0  +  value1  +  value2 + ...
void llegir_barra_sensors(){

    //uint8_t aux=0;
    uint8_t i=0;
    if (_SENSOR_I2){
        g_sensors[0] = 255;
    }else{
        g_sensors[0] = 0;
    }
    
    for (i=0;i<4;i++){
        g_sensors[i+1] = readADC8(i);
    }

    if (_SENSOR_D2){
        g_sensors[5] = 255;
    }else{
        g_sensors[5] = 0;
    }    
}

uint16_t read_sensor_bar_calibrated(){
    //float sensor_locations[6]={0,114.5,214.5,285.5,393,500}; //CNY70 sensors are not distributed equidistantly so we need these calculations
    float sensor_locations[6]={0,115,211,288,384,500}; //CNY70 sensors are not distributed equidistantly so we need these calculations
    //float sensor_locations[6]={0,100,200,300,400,500}; //CNY70 sensors are not distributed equidistantly so we need these calculations

    int i = 0;
    uint8_t numSensorsActivated = 0;
    static uint32_t ultima_lectura=0;
    uint32_t lectura_actual;
    uint32_t acum=0;
    uint16_t x = 0;
    llegir_barra_sensors();

    for(i=0;i<6;i++)
    {
        uint8_t calmin,calmax;
        uint8_t denominator;

        calmax = sensors_max_reading[i];
        calmin = sensors_min_reading[i];

        denominator = calmax - calmin;

        if(denominator != 0)
            x = (uint16_t) ((g_sensors[i] - calmin) * 255 / denominator);

        if(x < 40)
            x = 0;
        else if(x > 255)
            x = 255;

        g_sensors[i] = (uint8_t) x;
        if (g_sensors[i] != 0) numSensorsActivated++;
    }

    if (numSensorsActivated>0){
        // formula is:
        //
        //    0*value0 + 1*value1 + 2*value2 + ...
        //   --------------------------------------------
        //         value0  +  value1  +  value2 + ...
        for (i=0;i<6;i++){
            acum += (sensor_locations[i]*g_sensors[i]);
        }
        lectura_actual=acum/(g_sensors[0]+g_sensors[1]+g_sensors[2]+g_sensors[3]+g_sensors[4]+g_sensors[5]);
        ultima_lectura=lectura_actual;
    }else{
        if (ultima_lectura<250)
            lectura_actual=0;
        else
            lectura_actual=500;
    }
    return (uint16_t) lectura_actual;

}

void calibrate_sensors(){
    uint8_t i=0;
    uint16_t x=0;
    uint8_t contador = 0;
    int16_t cruiseSpeed = 30;
    //uint8_t y=0;

    for (i=0;i<6;i++){
        sensors_min_reading[i] = 0;
        sensors_max_reading[i] = 0;
    }
    //Ten seconds for calibration
    for (x=0;x<1000;x++){
        llegir_barra_sensors();

        for (i=0;i<6;i++){
            if (sensors_min_reading[i] == 0) sensors_min_reading[i] = g_sensors[i];
            if (sensors_max_reading[i] == 0) sensors_max_reading[i] = g_sensors[i];
            if (g_sensors[i] < sensors_min_reading[i]) sensors_min_reading[i] = g_sensors[i];
            if (g_sensors[i] > sensors_max_reading[i]) sensors_max_reading[i] = g_sensors[i];
        }
        contador++;
        setSpeed(cruiseSpeed,-cruiseSpeed);
        if (contador >= 100){
            contador = 0;
            cruiseSpeed = cruiseSpeed * (-1);
        }
        delay_ms(10);//
        //y++;
        /*if (y < 10) PORTB &= ~(1<<LED); //LED OFF
        else if (y < 20) PORTB |= 1<<LED; //LED on
        else y = 0;*/
    }
    setSpeed(0,0);
}
