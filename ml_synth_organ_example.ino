/*
 * Copyright (c) 2023 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
 * OHNE JEDE GEWÄHR,; sogar ohne die implizite
 * Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/**
 * @file ml_synth_organ_example.ino
 * @author Marcel Licence
 * @date 26.11.2021
 *
 * @brief   This is the main project file to test the ML_SynthLibrary (organ module)
 *          It should be compatible with ESP32, ESP8266, Seedstudio XIAO, PRJC Teensy 4.1, Electrosmith Daisy Seed, Raspberry Pi Pico, STM32F4
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#include "config.h"

#include <Arduino.h>

#include <arduino-timer.h>



/*
 * Library can be found on https://github.com/marcel-licence/ML_SynthTools
 */
#ifdef USE_ML_SYNTH_PRO
#include <ml_organ_pro.h>
#else
#include <ml_organ.h>
#endif
#ifdef REVERB_ENABLED
#include <ml_reverb.h>
#endif
#include <ml_delay.h>
#ifdef OLED_OSC_DISP_ENABLED
#include <ml_scope.h>
#endif

#include <ml_types.h>


#define ML_SYNTH_INLINE_DECLARATION
#include <ml_inline.h>
#undef ML_SYNTH_INLINE_DECLARATION


#if (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG)
#include <Wire.h> /* todo remove, just for scanning */
#endif


#define N_CHORDS 105
static int chords[N_CHORDS][4] = {
    {57,60,64,0},
    {60,64,69,0},
    {64,69,72,0},
    {57,60,64,67},
    {60,64,67,69},
    {64,67,69,72},
    {67,69,72,76},
    {60,64,67,0},
    {64,67,72,0},
    {67,72,76,0},
    {60,64,67,71},
    {64,67,71,72},
    {67,71,72,76},
    {71,72,76,79},
    {62,65,69,0},
    {65,69,74,0},
    {69,74,77,0},
    {62,65,69,72},
    {65,69,72,74},
    {69,72,74,77},
    {72,74,77,81},
    {64,67,71,0},
    {67,71,76,0},
    {71,76,79,0},
    {64,67,71,74},
    {67,71,74,76},
    {71,74,76,79},
    {74,76,79,83},
    {65,69,72,0},
    {69,72,77,0},
    {72,77,81,0},
    {65,69,72,76},
    {69,72,76,77},
    {72,76,77,81},
    {76,77,81,84},
    {67,71,74,0},
    {71,74,79,0},
    {74,79,83,0},
    {67,71,74,77},
    {71,74,77,79},
    {74,77,79,83},
    {77,79,83,86},
    {69,72,76,0},
    {72,76,81,0},
    {76,81,84,0},
    {69,72,76,79},
    {72,76,79,81},
    {76,79,81,84},
    {79,81,84,88},
    {72,76,79,0},
    {76,79,84,0},
    {79,84,88,0},
    {72,76,79,83},
    {76,79,83,84},
    {79,83,84,88},
    {83,84,88,91},
    {74,77,81,0},
    {77,81,86,0},
    {81,86,89,0},
    {74,77,81,84},
    {77,81,84,86},
    {81,84,86,89},
    {84,86,89,93},
    {76,79,83,0},
    {79,83,88,0},
    {83,88,91,0},
    {76,79,83,86},
    {79,83,86,88},
    {83,86,88,91},
    {86,88,91,95},
    {77,81,84,0},
    {81,84,89,0},
    {84,89,93,0},
    {77,81,84,88},
    {81,84,88,89},
    {84,88,89,93},
    {88,89,93,96},
    {79,83,86,0},
    {83,86,91,0},
    {86,91,95,0},
    {79,83,86,89},
    {83,86,89,91},
    {86,89,91,95},
    {89,91,95,98},
    {81,84,88,0},
    {84,88,93,0},
    {88,93,96,0},
    {81,84,88,91},
    {84,88,91,93},
    {88,91,93,96},
    {91,93,96,100},
    {84,88,91,0},
    {88,91,96,0},
    {91,96,100,0},
    {84,88,91,95},
    {88,91,95,96},
    {91,95,96,100},
    {95,96,100,103},
    {86,89,93,0},
    {89,93,98,0},
    {93,98,101,0},
    {86,89,93,96},
    {89,93,96,98},
    {93,96,98,101},
    {96,98,101,105}
};



char shortName[] = "ML_Organ";

auto timer1sec = timer_create_default();
auto timer8sec = timer_create_default();

void setup()
{

    timer1sec.every(1000, loop_1Hz);
    timer8sec.every(8000, loop_8sec);

    randomSeed(analogRead(0));
    /*
     * this code runs once
     */
#ifdef MIDI_USB_ENABLED
    Midi_Usb_Setup();
#endif

#ifdef BLINK_LED_PIN
    Blink_Setup();
    Blink_Fast(1);
#endif

#ifdef ARDUINO_DAISY_SEED
    DaisySeed_Setup();
#endif

    delay(500);

#ifdef SWAP_SERIAL
    /* only one hw serial use this for ESP */
    Serial.begin(115200);
    delay(500);
#else
    Serial.begin(115200);
#endif

    Serial.println();


    Serial.printf("Loading data\n");


#ifdef ESP32
#ifdef USE_ML_SYNTH_PRO
    char user[] = "";
    System_PrintInfo(user);
#endif
#endif

#ifdef ESP8266
    Midi_Setup();
#endif

    Serial.printf("Initialize Audio Interface\n");
    Audio_Setup();

#ifdef TEENSYDUINO
    Teensy_Setup();
#else

#ifdef ARDUINO_SEEED_XIAO_M0
    pinMode(7, INPUT_PULLUP);
    Midi_Setup();
    pinMode(LED_BUILTIN, OUTPUT);
#else

#ifndef ESP8266 /* otherwise it will break audio output */
    Midi_Setup();
#endif
#endif

#endif


#ifdef USE_ML_SYNTH_PRO
    OrganPro_Setup(SAMPLE_RATE);
#else
    Organ_Setup(SAMPLE_RATE);
#endif

#ifdef REVERB_ENABLED
    /*
     * Initialize reverb
     * The buffer shall be static to ensure that
     * the memory will be exclusive available for the reverb module
     */
    //static float revBuffer[REV_BUFF_SIZE];
    static float *revBuffer = (float *)malloc(sizeof(float) * REV_BUFF_SIZE);
    Reverb_Setup(revBuffer);
#endif

#ifdef MAX_DELAY
    /*
     * Prepare a buffer which can be used for the delay
     */
    //static int16_t delBuffer1[MAX_DELAY];
    //static int16_t delBuffer2[MAX_DELAY];
    static int16_t *delBuffer1 = (int16_t *)malloc(sizeof(int16_t) * MAX_DELAY);
    static int16_t *delBuffer2 = (int16_t *)malloc(sizeof(int16_t) * MAX_DELAY);
    Delay_Init2(delBuffer1, delBuffer2, MAX_DELAY);
#endif

#ifdef MIDI_BLE_ENABLED
    midi_ble_setup();
#endif

#ifdef USB_HOST_ENABLED
    Usb_Host_Midi_setup();
#endif

#ifdef ESP32
    Serial.printf("ESP.getFreeHeap() %d\n", ESP.getFreeHeap());
    Serial.printf("ESP.getMinFreeHeap() %d\n", ESP.getMinFreeHeap());
    Serial.printf("ESP.getHeapSize() %d\n", ESP.getHeapSize());
    Serial.printf("ESP.getMaxAllocHeap() %d\n", ESP.getMaxAllocHeap());

    /* PSRAM will be fully used by the looper */
    Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
#endif

    Serial.printf("Firmware started successfully\n");

#ifdef NOTE_ON_AFTER_SETUP
#ifdef USE_ML_SYNTH_PRO
    OrganPro_NoteOn(0, 60, 127);
    OrganPro_SetLeslCtrl(127);
#ifndef SOC_CPU_HAS_FPU
    Serial.printf("Synth might not work because CPU does not have a FPU (floating point unit)");
#endif
#else
    Organ_NoteOn(0, 60, 127);
    Organ_SetLeslCtrl(127);
    Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
    Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
#endif
#endif

#ifdef MIDI_STREAM_PLAYER_ENABLED
    MidiStreamPlayer_Init();

    char midiFile[] = "/song.mid";
    MidiStreamPlayer_PlayMidiFile_fromLittleFS(midiFile, 1);
#endif

#if (defined MIDI_VIA_USB_ENABLED) || (defined OLED_OSC_DISP_ENABLED)
#ifdef ESP32
    Core0TaskInit();
#else
#error only supported by ESP32 platform
#endif
#endif

     //loop_8sec(NULL);
}

#ifdef ESP32
/*
 * Core 0
 */
/* this is used to add a task to core 0 */
TaskHandle_t Core0TaskHnd;

inline
void Core0TaskInit()
{
    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "CoreTask0", 8000, NULL, 999, &Core0TaskHnd, 0);
}

void Core0TaskSetup()
{
    /*
     * init your stuff for core0 here
     */
         Serial.println("Initting core0...");


#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_Setup();
#endif
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Setup();
#endif
}

void Core0TaskLoop()
{
    /*
     * put your loop stuff for core0 here
     */
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Loop();
#endif

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_Process();
#endif


    Organ_NoteOn(0, 60, 127);

   delay(500);
       Organ_NoteOff(0, 60);
       delay(500);

}

void Core0Task(void *parameter)
{
    Core0TaskSetup();

    while (true)
    {
        Core0TaskLoop();

        /* this seems necessary to trigger the watchdog */
        delay(1);
        yield();
    }
}
#endif /* ESP32 */

bool loop_1Hz(void *)
{
   Serial.println("Hi 1hz");
#ifdef CYCLE_MODULE_ENABLED
    CyclePrint();
#endif
#ifdef BLINK_LED_PIN
    Blink_Process();
#endif
return true;
}

/*
bool loop_8sec(void *)
{
      Serial.println("Hi from 8sec loop");

    static bool noteOn = true;
    static int *lastChord = NULL;
    if (noteOn)
    {
        Serial.println("Note currently on, going to turn off");
                          Organ_NoteOff(0, 60);

        int *chord = lastChord;
        if (chord != NULL) {
            Serial.println("Have last chord...");
            for (int i = 0; i < 4; i++) {
              break;
                if (*chord == 0) {
                    break;
                }
                char numberArray[20];
                itoa(*chord, numberArray, 10);
                Serial.println(numberArray);
                Organ_NoteOff(0, *chord++);
            }
        }
    }
    else
    {
        Serial.println("Note currently off, going to turn on");
                  Organ_NoteOn(0, 60, 120);
        int chordIdx = random(N_CHORDS);
        int *chord = chords[chordIdx];
        lastChord = chord;
        for (int i = 0; i < 4; i++) {
          break;
            if (*chord == 0) {
                break;
            }
            char numberArray[20];
            itoa(*chord, numberArray, 10);
            Serial.println(numberArray);
            Organ_NoteOn(0, *chord++, 120);
            break;
        }
    }
    noteOn = !noteOn;
    return true;
}
*/
bool loop_8sec(void *)
{
      Serial.println("Hi from 8sec loop");

    static bool noteOn = false;
    if (noteOn)
    {
                Organ_NoteOff(0, 60);

    }
    else
    {
          Organ_NoteOn(0, 60, 120);

    }
    noteOn = !noteOn;
    return true;
}

void loop()
{
    timer1sec.tick();
    timer8sec.tick();


    /*
     * MIDI processing
     */
    Midi_Process();
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_ProcessSync();
#endif
#ifdef MIDI_STREAM_PLAYER_ENABLED
    MidiStreamPlayer_Tick(SAMPLE_BUFFER_SIZE);
#endif

#ifdef MIDI_BLE_ENABLED
    midi_ble_loop();
#endif

#ifdef USB_HOST_ENABLED
    Usb_Host_Midi_loop();
#endif

#ifdef MIDI_USB_ENABLED
    Midi_Usb_Loop();
#endif

    /*
     * And finally the audio stuff
     */
#ifdef USE_ML_SYNTH_PRO
#if (defined ESP8266) || (defined ARDUINO_SEEED_XIAO_M0) || (defined ARDUINO_RASPBERRY_PI_PICO) || (defined ARDUINO_GENERIC_RP2040)
#error Configuration is not supported
#else
    float mono[SAMPLE_BUFFER_SIZE], left[SAMPLE_BUFFER_SIZE], right[SAMPLE_BUFFER_SIZE];
    OrganPro_Process_fl(mono, SAMPLE_BUFFER_SIZE);

#ifdef INPUT_TO_MIX
    Audio_Input(left, right);
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] += (left[i] * 0.5f + right[i] * 0.5f) * 32.0f;
    }
#endif

#ifdef REVERB_ENABLED
    Reverb_Process(mono, SAMPLE_BUFFER_SIZE);
#endif

    Rotary_Process(left, right, mono, SAMPLE_BUFFER_SIZE);

#ifdef MAX_DELAY
    /*
     * post process delay
     */
    Delay_Process_Buff2(left, right, SAMPLE_BUFFER_SIZE);
#endif

    /*
     * Output the audio
     */
    Audio_Output(left, right);

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_AddSamples(left, right, SAMPLE_BUFFER_SIZE);
#ifdef TEENSYDUINO
    static uint8_t x = 0;
    x++;
    if (x == 0)
    {
        ScopeOled_Process();
    }
#endif
#endif
#endif
#else
    int32_t mono[SAMPLE_BUFFER_SIZE];
    Organ_Process_Buf(mono, SAMPLE_BUFFER_SIZE);
#ifdef REVERB_ENABLED
    float mono_f[SAMPLE_BUFFER_SIZE];
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono_f[i] = mono[i];
    }
    Reverb_Process(mono_f, SAMPLE_BUFFER_SIZE); /* post reverb */
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] = mono_f[i];
    }
#endif
    Audio_OutputMono(mono);
#endif
}

/*
 * MIDI via USB Host Module
 */
#ifdef MIDI_VIA_USB_ENABLED
void App_UsbMidiShortMsgReceived(uint8_t *msg)
{
#ifdef MIDI_TX2_PIN
    Midi_SendShortMessage(msg);
#endif
    Midi_HandleShortMsg(msg, 8);
}
#endif

/*
 * MIDI callbacks
 */
inline void Organ_PercSetMidi(uint8_t setting, uint8_t value)
{
    if (value > 0)
    {
#ifdef USE_ML_SYNTH_PRO
        OrganPro_PercussionSet(setting);
#else
        Organ_PercussionSet(setting);
#endif
    }
}

inline void Organ_SetDrawbarInv(uint8_t id, uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetDrawbar(id, value);
#else
    Organ_SetDrawbar(id, value);
#endif
}

inline void Organ_SetCtrl(uint8_t unused __attribute__((unused)), uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(value);
#else
    Organ_SetLeslCtrl(value);
#endif
}

inline void Organ_SetLeslieSpeedNorm(uint8_t unused __attribute__((unused)), uint8_t speed)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(speed);
#else
    Organ_SetLeslCtrl(speed);
#endif
}

inline void Organ_ModulationWheel(uint8_t unused __attribute__((unused)), uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(value);
#else
    Organ_SetLeslCtrl(value);
#endif
}

inline void Reverb_SetLevelInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Reverb_SetLevel(unused, val);
#endif
}

inline void Delay_SetOutputLevelInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Delay_SetOutputLevel(unused, val);
#endif
}

inline void Delay_SetFeedbackInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Delay_SetFeedback(unused, val);
#endif
}

#if (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG)
void ScanI2C(void)
{
#ifdef ARDUINO_GENERIC_F407VGTX
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();//I2C_SDA, I2C_SCL);
#else
    Wire.begin();
#endif

    byte address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        byte r_error;
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        r_error = Wire.endTransmission();

        if (r_error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (r_error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}
#endif /* (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG) */

