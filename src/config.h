/**
 * @author  Artium Nihamkin <artium@nihamkin.com>
 * @version 2.0
 * @date December 2014 
 *
 * @section LICENSE
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Artium Nihamkin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
#ifndef SPORTS_ARENA_PORTABLE_H
#define SPORTS_ARENA_PORTABLE_H

#include <avr/pgmspace.h>
#include "pitches.h"
 
 
/* *************************************************
 * Pins (Arduino numbering)
 * *************************************************/
#define BUZZER_PIN             3 /// pin 3 is a must for proper function of tone()
#define VIBRATION_PIN          4
#define NEO_PIXELS_PIN         6
#define PUSHBUTTON_PIN         7
#define RANDOM_SEED_PIN        9
 
 
 
 
/* *************************************************
* Macros
* *************************************************/

/// Divide integers x by y and round the result up.
#define ROUND_UP_DIVISION(x,y) ((x + y - 1) / y)

/// Calculate number of frames between changes from the duty cycle in ms.
#define DUTY_CYCLE_TO_FRAMES(duty_cycle_ms) (((uint32_t)duty_cycle_ms * (uint32_t)FPS) / ((uint32_t)ONE_SECOND_MS * 2)) // notice the division by 2 (duty cycle is both high and low)

/* *************************************************
 * Types
 * *************************************************/

enum CorrelationColor {
    TARGET,
    CURSOR  
};

enum TargetBehaviour {
    NONE,
    SHAKE,
    RANDOM_WALK,
    SAME_DIRECTION,
    OPPOSING_DIRECTION  
};

/*
 * Speed in movments per seconds 
 */
enum CursorSpeed {
  SPEED_1 = 8,
  SPEED_2 = 12,  
  SPEED_3 = 15
};

typedef struct {
    CursorSpeed cursor_speed; 
    int num_targets;
    CorrelationColor corr_color;
    TargetBehaviour target_behaviour;  
    
} LevelConfig;


enum {
    NO_REPEAT = 0,
    REPEAT    = 1  
};

typedef struct {
    const int* notes;
    const int* durations;    // 1/x of the beat duration
    unsigned int note_duration_cycles; /// tempo
    bool repeat;
} Melody;
    

/* *************************************************
 * Constants
 * *************************************************/

#define NUM_LOCATIONS         16                       /// we use a 16 neo pixel ring
#define HALF_SECOND_MS        500
#define ONE_SECOND_MS         1000
#define TWO_SECOND_MS         2000
#define FPS                   100                      /// system wide value, relevant for all sequences
#define FRAME_DELAY_MS        (ONE_SECOND_MS / FPS)
#define SECONDS_PER_LEVEL     NUM_LOCATIONS
#define FRAMES_PER_LEVEL      (SECONDS_PER_LEVEL * FPS) 
#define VIB_PULSE_DUR_MS      200                      /// Typical duration of a vibration pulse

#define FLSH_WAIT_DUTY_CYCLE_MS      (1*ONE_SECOND_MS) /// time between flashes of the next level led in the waiting sequence
#define FLSH_GM_OVR_DUTY_CYCLE_MS    (1*ONE_SECOND_MS) /// time between flashes of the game over sequence
#define TIME_BETWEEN_TARGET_UPDT_MS  (ONE_SECOND_MS + HALF_SECOND_MS) /// time between targets' locations updates

#define TIMER_1_INT_TIME       50  /// The interrupt is called every 50ms, if you change this value, you need to also change the timer setup code.

#define GAME_OVER_FLASHES      4                       /// number of flashes in the game over sequence
#define READY_SET_GO_DELAY_MS (3*ONE_SECOND_MS)/2      /// delay between animation stages of ready-set-go sequence


#define NOTE_DURATION_FACTOR_M10   11    // equals 1.2. ie if note duration is X, than the note+pause after note duration is 1.2x
#define FACTOR_M10                 10

#define INVALID_LOCATION       (-1)
#define MAX_TARGETS            3



#define EEPROM_ADDR_LAST_LEVEL 0x0

/* *************************************************
 * Data and configuration
 * *************************************************/

const LevelConfig CONFIG[] = {
    {SPEED_1, 3, CURSOR, NONE},   
    {SPEED_1, 2, CURSOR, NONE},   
    {SPEED_1, 1, CURSOR, NONE},   
    {SPEED_1, 3, CURSOR, SAME_DIRECTION}, 
    {SPEED_2, 3, CURSOR, NONE}, 
    {SPEED_2, 3, CURSOR, SAME_DIRECTION}, 
    {SPEED_2, 2, CURSOR, SAME_DIRECTION}, 
    {SPEED_3, 2, CURSOR, SAME_DIRECTION},
    {SPEED_2, 2, CURSOR, OPPOSING_DIRECTION}, 
    {SPEED_2, 1, TARGET, OPPOSING_DIRECTION},
    {SPEED_3, 2, TARGET, SHAKE},
    {SPEED_3, 3, TARGET, RANDOM_WALK},
    {SPEED_3, 2, TARGET, RANDOM_WALK},
    {SPEED_3, 1, TARGET, SAME_DIRECTION},
    {SPEED_3, 1, TARGET, SHAKE},
    {SPEED_3, 1, TARGET, RANDOM_WALK},     
};

const int TOTAL_LEVELS = sizeof(CONFIG) / sizeof(CONFIG[0]);


int silent_melody_notes[]     = { HALT };
int silent_melody_durations[] = { 0 }; 
Melody silent_melody = { silent_melody_notes, silent_melody_durations, 100, false  }; // 120 BPM




const PROGMEM int test_notes[] =      { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, PAUSE, NOTE_B3, NOTE_C4, HALT };
const PROGMEM int test_durations[]  = { 4,       8,       8,       4,       4,       4,     4,       4,       0    }; 
const Melody test_melody = { test_notes, test_durations, 50, REPEAT  }; // 120 BPM = 100 cycles pre beat (20hz)
 
 
 
const int bond_notes[] = { NOTE_E4,NOTE_F4,NOTE_F4,NOTE_F4,NOTE_F4,NOTE_E4,NOTE_E4,NOTE_E4,
  NOTE_E4,NOTE_G4,NOTE_G4,NOTE_G4,NOTE_G4,NOTE_E4,NOTE_E4,NOTE_E4,
  NOTE_E4,NOTE_F4,NOTE_F4,NOTE_F4,NOTE_F4,NOTE_E4,NOTE_E4,NOTE_E4,
  NOTE_E4,NOTE_G4,NOTE_G4,NOTE_G4,NOTE_G4,NOTE_E4,NOTE_E4,NOTE_E4,
  NOTE_DS5,NOTE_D5,NOTE_B4,NOTE_A4,NOTE_B4,
  NOTE_E4,NOTE_G4,NOTE_DS5,NOTE_D5,NOTE_G4,NOTE_B4,
  NOTE_B4,NOTE_FS5,NOTE_F5,NOTE_B4,NOTE_D5,NOTE_AS5,
  NOTE_A5,NOTE_F5,NOTE_A5,NOTE_DS6,NOTE_D6, HALT };
  
const int bond_durations[] = {   8,16,16,8,4,8,8,8,
  8,16,16,8,4,8,8,8,
  8,16,16,8,4,8,8,8,
  8,16,16,8,4,8,8,8,
  8,2,8,8,1,
  8,4,8,4,8,8,
  8,8,4,8,4,8,
  4,8,4,8,3 };
  
const Melody bond_melody = { bond_notes, bond_durations, 50, REPEAT  }; // 120 BPM = 100 cycles pre beat (20hz)
 
 
 
#endif