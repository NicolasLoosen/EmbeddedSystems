#include <signal.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "brick.h"

#define L_MOTOR_PORT      OUTPUT_C
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define M_MOTOR_PORT      OUTPUT_A
#define M_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define SPEED_LINEAR      75
#define SPEED_CIRCULAR    50
#define SPEED_FLAG        10

static int32_t speed_linear;
static int32_t speed_circular;
static int32_t speed_flag;

static uint8_t ir, touch;
static uint8_t motor[ 3 ] = { (uint8_t)DESC_LIMIT, (uint8_t)DESC_LIMIT, (uint8_t)DESC_LIMIT };

enum { Left, Right, Flag };

static volatile uint8_t keepRunning;

static void intHandler(int32_t dummy);
static void _run_forever( int32_t left_speed_forever, int32_t right_speed_forever );
static void _run_to_rel_position( int32_t left_speed_position, int32_t left_position, int32_t right_speed_position, int32_t right_position );
static void _run_timed( int32_t left_speed_timed, int32_t right_speed_timed, int32_t millisec );
static void _run_flag( int32_t speed_runflag, int32_t angle_flag );
static int32_t  _check_pressed(uint8_t touchsensor );
static void _stop( void );
static void surrender( int32_t sound );
static void handle_proximity( void );
static uint8_t app_init( void );
static void run( void );

int32_t main(int32_t argc,char *const argv[]) {
    keepRunning  = (uint8_t)1;
    signal(SIGINT, &intHandler);
    int32_t exitApp = 0;
    uint8_t ev3init = (uint8_t)1;
    uint8_t app_alive = (uint8_t)0;

    if ( ev3_init() < 1 ) {
       ev3init = (uint8_t)0;
    }
    if ( ev3_sensor_init() < 0 ) {
       ev3init = (uint8_t)0;
    }
    if ( ev3_tacho_init() < 0 ) {
       ev3init = (uint8_t)0;
    }
    if( ev3init == (uint8_t)1 ){
        app_alive = app_init();
    }else{
        exitApp = -1;
    }
    if( app_alive == (uint8_t)1 ) {
        run();
    }else{
        exitApp = -1;
    }
    return ( exitApp );
}

static void intHandler(int32_t dummy) {
    keepRunning = (uint8_t)0;
}

static void _run_forever( int32_t left_speed_forever, int32_t right_speed_forever ){
    set_tacho_speed_sp( motor[ Left ], left_speed_forever );
    set_tacho_speed_sp( motor[ Right ], right_speed_forever );
    multi_set_tacho_command_inx( motor, TACHO_RUN_FOREVER );
}

static void _run_to_rel_position( int32_t left_speed_position, int32_t left_position, int32_t right_speed_position, int32_t right_position ){
    set_tacho_speed_sp( motor[ Left ], left_speed_position );
    set_tacho_speed_sp( motor[ Right ], right_speed_position );
    set_tacho_position_sp( motor[ Left ], (( left_position ) * 260 / 90 ) );
    set_tacho_position_sp( motor[ Right ], (( right_position ) * 260 / 90 ) );
    multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
    sleep_ms((uint32_t)1000);
}

static void _run_timed( int32_t left_speed_timed, int32_t right_speed_timed, int32_t millisec ){
    set_tacho_speed_sp( motor[ Left ], left_speed_timed );
    set_tacho_speed_sp( motor[ Right ], right_speed_timed );
    multi_set_tacho_time_sp( motor, millisec );
    multi_set_tacho_command_inx( motor, TACHO_RUN_TIMED );
    sleep_ms((uint32_t)1000);
}

static void _run_flag( int32_t speed_runflag, int32_t angle_flag ){
    set_tacho_speed_sp( motor[ Flag ], speed_runflag );
    set_tacho_position_sp( motor[ Flag ], angle_flag );
    multi_set_tacho_command_inx( motor, TACHO_RUN_TO_ABS_POS );
    sleep_ms((uint32_t)1000);
}

static int32_t  _check_pressed( uint8_t touchsensor ){
    int32_t valuePressed = 0;
    get_sensor_value( (uint8_t)0, touchsensor, &valuePressed );
    return valuePressed;
}

static void _stop( void ){
    set_tacho_speed_sp( motor[ Left ], 0 );
    set_tacho_speed_sp( motor[ Right ], 0 );
    set_tacho_speed_sp( motor[ Flag ], 0 );
    multi_set_tacho_command_inx( motor, TACHO_STOP );
}

static void surrender(int32_t sound){
    _stop();
    if(sound == 1){
        system("aplay sound.wav");
    }
    if(sound == 2){
        system("aplay stuck.wav");
    }
    uint8_t direction = (uint8_t)1;
    for (int32_t iter = 0; iter <= 5; iter++){
        if ((keepRunning == (uint8_t)1) && (direction == (uint8_t)1)){
            _run_flag(speed_flag, -105);
            direction = (uint8_t)0;
        }else if(keepRunning == (uint8_t)1){
            _run_flag(speed_flag, -75);
            direction = (uint8_t)1;
        }else{}
    }
    _run_flag(speed_flag, 0);
    _stop();
}

static void handle_proximity( void ){
    int32_t front;
    int32_t proxi = 0;
    int32_t angle;
    int32_t surrendercount = 0;

    get_sensor_value( (uint8_t)0, ir, &proxi );
    if((keepRunning == (uint8_t)1)){
        if((_check_pressed(touch) == 1) || (proxi < 350 )) {
            if ( _check_pressed(touch) == 1) {
                surrender(1);
                _run_timed( -speed_linear, -speed_linear, 1500 );
            }
            get_sensor_value( (uint8_t)0, ir, &proxi );
            angle = -30;
            front = proxi;
            do {
                if(keepRunning == (uint8_t)1){
                    if (surrendercount > 6){
                        surrender(2);
                        surrendercount=0;
                    }
                    if ( _check_pressed(touch) == 1) {
                        surrender(1);
                        _run_timed( -speed_linear, -speed_linear, 1500 );
                    }
                    _run_to_rel_position( speed_circular, -angle, speed_circular, angle);
                    _stop();
                    proxi = 0;
                    get_sensor_value( (uint8_t)0, ir, &proxi );
                    if (  (_check_pressed(touch) == 1) || (proxi < front)) {
                        if ( angle < 0 ) {
                            angle = 60;
                        } else {
                            _run_timed( -speed_linear, -speed_linear, 1500 );
                            _stop();
                        }
                    }
                    surrendercount++;
                }
            } while (( keepRunning == (uint8_t)1 ) && ( proxi > 0 ) && ( proxi < 500 ));
        }
    }
}


static uint8_t app_init( void ){
    int32_t max_speed = 0;
    uint8_t rvalue= (uint8_t)1;
    if ( ev3_search_tacho_plugged_in( (uint8_t)L_MOTOR_PORT, (uint8_t)L_MOTOR_EXT_PORT, motor + Left, (uint8_t)0 )) {
        get_tacho_max_speed( motor[ Left ], &max_speed );
        set_tacho_command_inx( motor[ Left ], TACHO_RESET );
        speed_linear = max_speed * SPEED_LINEAR / 100;
        speed_circular = max_speed * SPEED_CIRCULAR / 100;
        speed_flag = max_speed * SPEED_FLAG / 100;
    } else {
        rvalue = (uint8_t)0 ;
    }
    if ( ev3_search_tacho_plugged_in( (uint8_t)R_MOTOR_PORT, (uint8_t)R_MOTOR_EXT_PORT, motor + Right, (uint8_t)0 )) {
        set_tacho_command_inx( motor[ Right ], TACHO_RESET );
    } else {
        rvalue = (uint8_t)0 ;
    }
    if ( ev3_search_tacho_plugged_in( (uint8_t)M_MOTOR_PORT, (uint8_t)M_MOTOR_EXT_PORT, motor + Flag, (uint8_t)0 )) {
        set_tacho_command_inx( motor[ Flag ], TACHO_RESET );
    } else {
        rvalue = (uint8_t)0 ;
    }
    if ( ev3_search_sensor( LEGO_EV3_US, &ir, (uint8_t)0 )) {
        set_sensor_mode_inx( ir, LEGO_EV3_US_US_DIST_CM );
    } else {
        rvalue = (uint8_t)0 ;
    }
    if ( ev3_search_sensor( LEGO_EV3_TOUCH, &touch, (uint8_t)0 )) {
    } else {
        rvalue = (uint8_t)0 ;
    }

    return(rvalue);
}

static void run( void ){
    int32_t proximity = 0;
    get_sensor_value((uint8_t)0 , ir, &proximity);
    if((_check_pressed(touch) != 1) && (proximity > 350)){
        _run_forever( speed_linear, speed_linear );
    }
    while (keepRunning == (uint8_t)1) {
        get_sensor_value((uint8_t)0 , ir, &proximity);

        if((_check_pressed(touch) == 1) || (proximity < 350 )){
            _stop();
            handle_proximity();
            _run_forever( speed_linear, speed_linear );
        }
    }
    _stop();
}

