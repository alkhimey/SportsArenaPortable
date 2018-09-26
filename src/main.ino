/**
 * @author  Artium Nihamkin <artium@nihamkin.com>
 * @version 2.0
 * @date December 2014
 *
 * @section LICENSE
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2017 Artium Nihamkin
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
 *
 * @section DESCRIPTION
 *
 * This is a protable adaptation of the prize redemption game called "Sports Arena" that was released
 * by Sammy USA Corporation in 1995. More info about the original can be found in the following link:
 * http://www.arcade-museum.com/game_detail.php?game_id=9730
 *
 * The game itself is simple. A colored cursor light is circling around. The user needs to push the button
 * in time so that the cursor light stops on top of one of the target lights.
 *
 * There are 16 different levels. Levels get progressively harder: higher speed, less targets
 * and additional surprises.
 *
 * Time limit for each level is 16 seconds. The white ring, the "background",
 * indicates how much time is left.
 *
 * NeoPixel display, level logic and animation playing is preformed from the main loop.
 * Animations are sequential, with delay function. Level recalculation performed inside a 20hz loop.
 * Sound and vibration is performed via a timer1 interrupt. The interrupt is set to 50ms intervalls.
 * This allows sound and vibration to be performed concurrently to any other logic.
 *
 *
 */

#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include "pitches.h"
#include "config.h"


/* *************************************************
 * Global Variables
 * *************************************************/

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel g_strip = Adafruit_NeoPixel(NUM_LOCATIONS, NEO_PIXELS_PIN, NEO_GRB + NEO_KHZ800);

/*
 * Currently played or waiting to be played level.
 */
uint8_t g_current_level;

/*
 * Was pushbutton pressed in last cycle?
 * We use pullup resistor so hight is actually pb off.
 */
int g_pb_last_pass = HIGH;


/*
 * Currently playing melody.
 */
const Melody*      g_curr_melody_ptr;
unsigned int g_curr_note_pos;

unsigned int g_sound_timer = 0;

/*
 * This counts timer interrupt calls.
 */
unsigned int g_vibration_timer = 0;

/* *************************************************
 * Functions
 * *************************************************/

void setup() {

    g_strip.begin();
    
    g_strip.setBrightness(10); // oh my sore eyes...

    g_strip.show(); // initialize all pixels to 'off'

    randomSeed(analogRead(RANDOM_SEED_PIN));
    g_current_level = 0;
    g_pb_last_pass = false;
    Serial.begin(9600);
    Serial.println( "Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);

    pinMode(PUSHBUTTON_PIN, INPUT);
    digitalWrite(PUSHBUTTON_PIN, HIGH); // turn on pullup resistors

    pinMode(BUZZER_PIN, OUTPUT);

    pinMode(VIBRATION_PIN, OUTPUT);
    digitalWrite(VIBRATION_PIN, LOW);

    /* Setup sound */
    g_curr_melody_ptr = NULL;
    g_curr_note_pos  = 0;;
    g_sound_timer = 0;

    /* setup vibration */
    g_vibration_timer = 0;



    /* Set timer1 interrupt to 20Hz */
    cli();//stop interrupts
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    OCR1A = 781; // approximately 20Hz
    TCCR1B |= (1 << WGM12);// turn on CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10);  // 1024 presxaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();//allow interrupts

    Serial.println("Setup is complete");

    /******************
     * Main Game Loop *
     ******************/

    if(is_initial_program_load()) {
        Serial.println("IPL occured, reset level to 0 ");
        g_current_level = 0;
    } else {
        g_current_level = EEPROM.read(EEPROM_ADDR_LAST_LEVEL); 
        Serial.print("Read level from eeprom: ");
        Serial.println(g_current_level);
    }

    while(g_current_level < TOTAL_LEVELS)
    {
        Serial.print("Writing level to eeprom: ");
        Serial.println(g_current_level);
        EEPROM.write(EEPROM_ADDR_LAST_LEVEL, g_current_level);
        
        start_level_waiting_seq(g_current_level);
        ready_set_go_seq();
        if (play_level(g_current_level) == false) {
            // failure sequence is played from inside "play_lavel"

            continue; // restart without advancing the level
        }

        g_current_level++;
    }

}


void loop() {
    // Everything is in setup function
}

/**
 * Determine if this is the first power-up after
 * fresh program was loaded into the flash.
 */
boolean is_initial_program_load()
{
    const String version_date = __DATE__ __TIME__;
    uint16_t len = version_date.length();
    boolean is_ipl = false;

    for (unsigned int i = 0; i < len; i++) {
        int addr = EEPROM_ADDR_VERSION_DATE + i;

        if (EEPROM.read(addr) != version_date[i]) {
            EEPROM.write(addr, version_date[i]);
            is_ipl = true;
        } 
    }

    return is_ipl;
}

/*
 * Plays the level logic
 *
 * Each level has it's own configuration parametrs which are
 * stroed in the CONFIG constant.
 *
 */
boolean play_level(int lvl_idx) {

    /* initial setup */
    int bg_size = g_strip.numPixels();

    int cursor_loc = 0;
    int cursor_counter = 0; // count frames untill cursor needs to move to next position
    const int cursor_frames_per_move = FPS / (int)CONFIG[lvl_idx].cursor_speed;
    const int target_frames_per_move = TIME_BETWEEN_TARGET_UPDT_MS / FRAME_DELAY_MS;
    int prev_target_mv_dir = -1;

    int target_loc[MAX_TARGETS];
    int target_counter = 0; // targets move or shake

    const int target_loc_base = random(15);
    for(int i = 0; i < MAX_TARGETS; i++) {

        target_loc[i] = INVALID_LOCATION; // target does not exist

        if(i < CONFIG[lvl_idx].num_targets) {
            target_loc[i] = (target_loc_base + i * (NUM_LOCATIONS / CONFIG[lvl_idx].num_targets)) % NUM_LOCATIONS;
        }
    }

    init_pb_last_pass();

    /* Play the level for a predefined duration */
    for(int i = 0; i < FRAMES_PER_LEVEL; i++) {

        /* accept push button rising edge logic */
        if(is_pb_rising()) {

            for(int i = 0; i < MAX_TARGETS; i++) {
                if(target_loc[i] != INVALID_LOCATION && target_loc[i] == cursor_loc) {

                    stop_melody();

                    /* TODO: victory sequence */

                    /* player hit a target */
                    return true;
                }
            }

            stop_melody();

            /* reaching here indicates that the player was not sucessfull */
            failure_seq(cursor_loc, target_loc);

            /* after game over sequence, return to main menu */
            return false;
        }


        /* background indicates how much seconds left before the end of level */
        bg_size = ROUND_UP_DIVISION((FRAMES_PER_LEVEL - i), FPS);

        if(cursor_counter != cursor_frames_per_move) {
            cursor_counter ++;
        } else {
            cursor_counter = 0;
            cursor_loc = (cursor_loc + 1) % NUM_LOCATIONS;
        }


        if(target_counter < target_frames_per_move) {

            target_counter ++ ;
        } else {
            target_counter = 0;

            int move_direction = 0; // -1, 0, 1

            switch(CONFIG[lvl_idx].target_behaviour) {

                case RANDOM_WALK:
                    for(int i = 0; i < MAX_TARGETS; i++) {
                        if(target_loc[i] != INVALID_LOCATION) {
                            target_loc[i] = (target_loc[i] + NUM_LOCATIONS + random(3) - 2) % NUM_LOCATIONS;
                        }
                    }
                    move_direction = 0;
                    break;

                case SHAKE:
                    move_direction = -prev_target_mv_dir;
                    prev_target_mv_dir = move_direction;
                    break;
                case SAME_DIRECTION:
                    move_direction = 1;
                    break;
                case OPPOSING_DIRECTION:
                    move_direction = -1;
                    break;
                case NONE:
                    move_direction = 0;
                    break;
            }



            for(int i = 0; i < MAX_TARGETS; i++) {
                if(target_loc[i] != INVALID_LOCATION) {
                    target_loc[i] = (target_loc[i] + NUM_LOCATIONS + move_direction) % NUM_LOCATIONS;
                }
            }

        }

        /* draw everything */

        g_strip.clear();

        for(int j=0; j < bg_size; j++) {
            g_strip.setPixelColor(j, COLOR_BG);
        }

        g_strip.setPixelColor(cursor_loc, COLOR_CURSOR);

        for(int i = 0; i < MAX_TARGETS; i++) {
            if(target_loc[i] != INVALID_LOCATION &&
               !(target_loc[i] == cursor_loc && CONFIG[lvl_idx].corr_color == CURSOR)) {
                 g_strip.setPixelColor(target_loc[i], COLOR_TARGET);
            }
        }

        g_strip.show();


        delay(FRAME_DELAY_MS);
    }

    /* timeout */
    failure_seq(cursor_loc, target_loc);

    return false;
}

/*
 * Ready, set go sequence
 *
 * Plays a "traffic light" animation.
 *
 */
void ready_set_go_seq() {

    for(int i=0; i<3; i++) {
        request_vibration(VIB_PULSE_DUR_MS);

        /* green */
        g_strip.clear();
        g_strip.setPixelColor(0,                 g_strip.Color(0, 255, 0));
        g_strip.setPixelColor(NUM_LOCATIONS - 1, g_strip.Color(0, 255, 0));

        if(i<2) {
            /* yellow */
            g_strip.setPixelColor(1,                 g_strip.Color(255, 255, 0));
            g_strip.setPixelColor(NUM_LOCATIONS - 2, g_strip.Color(255, 255, 0));
            g_strip.setPixelColor(2,                 g_strip.Color(255, 255, 0));
            g_strip.setPixelColor(NUM_LOCATIONS - 3, g_strip.Color(255, 255, 0));
        }

        if(i < 1) {
            /* red */
            g_strip.setPixelColor(3,                 g_strip.Color(255, 0, 0));
            g_strip.setPixelColor(NUM_LOCATIONS - 4, g_strip.Color(255, 0, 0));
            g_strip.setPixelColor(4,                 g_strip.Color(255, 0, 0));
            g_strip.setPixelColor(NUM_LOCATIONS - 5, g_strip.Color(255, 0, 0));
        }

        g_strip.show();

        delay(READY_SET_GO_DELAY_MS);
    }
}


/*
 * Start level waiting sequence
 *
 * Will display the number of passed levels with the next level blinking.
 * Pushing the button will start that level.
 *
 */
void start_level_waiting_seq(int next_lvl_idx) {

    const uint32_t FRAMES_PER_FLASHING_CHANGE = DUTY_CYCLE_TO_FRAMES(FLSH_WAIT_DUTY_CYCLE_MS);
    uint32_t flash_counter = FRAMES_PER_FLASHING_CHANGE;
    boolean flashing_led_on = true;

    init_pb_last_pass();


    // TODO: Artium rm debug -or- waiting melody
    request_melody(&bond_melody);

    while(1) {

        if(is_pb_rising()) {
           return;
        }

        if(flash_counter < FRAMES_PER_FLASHING_CHANGE) {
            flash_counter++;
        } else {
            flash_counter = 0;
            flashing_led_on = !flashing_led_on;

            g_strip.clear();

            for(int j=0; j < g_current_level; j++) {
                g_strip.setPixelColor(j, COLOR_LEVEL_SELECT);
            }

            if(flashing_led_on) {
                g_strip.setPixelColor(next_lvl_idx, COLOR_LEVEL_SELECT);
            }

            g_strip.show();
        }

        delay(FRAME_DELAY_MS);
    }
}

/*
 * This sequence is played when player fails to complete the level
 *
 * The cursor and the targets will be flashing.
 */
void failure_seq(int cursor_loc, int* target_loc) {

    const uint32_t FRAMES_PER_FLASHING_CHANGE = DUTY_CYCLE_TO_FRAMES(FLSH_GM_OVR_DUTY_CYCLE_MS);

    unsigned int seq_counter = 0;
    uint32_t flash_counter = FRAMES_PER_FLASHING_CHANGE;
    boolean flashing_on = true; // this will cause to finish with 'on'

    // multiply by 2 because FRAMES_PER_FLASHING_CHANGE is half a cycle
    while(seq_counter < GAME_OVER_FLASHES * 2 * FRAMES_PER_FLASHING_CHANGE) {

        if(flash_counter < FRAMES_PER_FLASHING_CHANGE) {
            flash_counter++;
        } else {
            flash_counter = 0;
            flashing_on = !flashing_on;

            g_strip.clear();

            if(flashing_on == false) {
                request_vibration(VIB_PULSE_DUR_MS);
            } else {
                for(int i = 0; i < MAX_TARGETS; i++) {
                    if(target_loc[i] != INVALID_LOCATION) {
                         g_strip.setPixelColor(target_loc[i], COLOR_TARGET);
                    }
                }

                g_strip.setPixelColor(cursor_loc, COLOR_CURSOR);
            }

            g_strip.show();
        }

        seq_counter++;
        delay(FRAME_DELAY_MS);
    }

    delay(1.5 * ONE_SECOND_MS);
}



/**
 * Returns true is pb is rising (went from off to on). Updates
 * last pass variable
 */
boolean is_pb_rising() {

    int pb_this_pass = digitalRead(PUSHBUTTON_PIN);

    if(pb_this_pass == LOW && g_pb_last_pass == HIGH) {
        g_pb_last_pass = LOW;

        Serial.println("PB is raising...");
        return true;
    }

    g_pb_last_pass = pb_this_pass;
    return false;
}

void init_pb_last_pass() {
    g_pb_last_pass = digitalRead(PUSHBUTTON_PIN);
}


void request_vibration(int duration_ms) {
    g_vibration_timer = duration_ms / TIMER_1_INT_TIME;
}

void request_melody(const Melody* melody_ptr) {
    noTone(BUZZER_PIN);
    g_curr_melody_ptr = melody_ptr;
    g_curr_note_pos = 0;
    g_sound_timer = 0;
}

void stop_melody() {
   request_melody(&silent_melody);
}

/**
 * timer1 interrupt, used to control sound and vibration (20Hz)
 */
ISR(TIMER1_COMPA_vect){
    if(g_vibration_timer > 0) {
        g_vibration_timer--;
        digitalWrite(VIBRATION_PIN, HIGH);
    } else {
        digitalWrite(VIBRATION_PIN, LOW);
    }

    if(g_sound_timer > 0) {
        g_sound_timer--;
    } else {

        int note = *(g_curr_melody_ptr->notes + g_curr_note_pos);
        int dur_div = *(g_curr_melody_ptr->durations + g_curr_note_pos);

        if(note == HALT) {
            if (g_curr_melody_ptr->repeat) {
                g_curr_note_pos = 0; // implicit pause of one cycle before melody restarts
            }
        } else {

            // tone duration in ms excluding the silence time between notes
            int tone_dur_ms = (g_curr_melody_ptr->note_duration_cycles * TIMER_1_INT_TIME) / dur_div;

            // number of cycles to wait untill new note can be played. includes the silence between notes
            g_sound_timer = (g_curr_melody_ptr->note_duration_cycles * NOTE_DURATION_FACTOR_M10) / (FACTOR_M10 * dur_div);

            if (note != PAUSE) {
                tone(BUZZER_PIN, note, tone_dur_ms);
            }
            g_curr_note_pos++;
        }
    }
}
