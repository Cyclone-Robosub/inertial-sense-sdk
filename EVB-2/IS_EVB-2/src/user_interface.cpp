﻿/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "globals.h"
#include "sd_card_logger.h"
#include "user_interface.h"


static bool s_ignoreCfgButtonRelease=false;
static bool s_ignoreLogButtonRelease=false;

static FuncPtrVoidVoid s_funcPtrCfgButtonPressed = NULL;
static FuncPtrVoidVoid s_funcPtrCfgButtonRelease = NULL;
static FuncPtrVoidVoid s_funcPtrLogButtonPressed = NULL;
static FuncPtrVoidVoid s_funcPtrLogButtonRelease = NULL;
static FuncPtrVoidVoid s_funcPtrBothButtonsPressed = NULL;
static FuncPtrVoidVoid s_funcPtrBothButtonsRelease = NULL;

static void on_cfg_button_pressed()
{
	// Indicate button is pressed by turning off LED
	LED_CFG_OFF();
}


static void on_cfg_button_release()
{    
    // Increment config
    g_flashCfg->cbPreset++;

    switch(g_flashCfg->cbPreset)
    {   // Skip these configs
#ifndef CONF_BOARD_SPI_ATWINC_WIFI
        case EVB2_CB_PRESET_RS422_WIFI:
#endif
#ifndef CONF_BOARD_SPI_UINS
        case EVB2_CB_PRESET_SPI_RS232:
#endif
        g_flashCfg->cbPreset++;
    }

    // Restart config
    if(g_flashCfg->cbPreset >= EVB2_CB_PRESET_COUNT)
    {
        g_flashCfg->cbPreset = 1;
    }
        
    com_bridge_apply_preset(g_flashCfg);
    board_IO_config();
    g_nvr_manage_config.flash_write_needed = true;
	g_nvr_manage_config.flash_write_enable = true;
    refresh_CFG_LED();    
}


bool logger_ready()
{
    return (g_status.evbStatus&EVB_STATUS_SD_CARD_READY) && !(g_status.evbStatus&EVB_STATUS_SD_ERR_CARD_MASK);
}


static void on_log_button_pressed()
{
	// Indicate button is pressed by turning off LED
	LED_LOG_OFF();    
}


static void on_log_button_release()
{
    if( logger_ready() )
    {
        // Toggle logger enable
        enable_logger(!g_loggerEnabled);
//         if(g_loggerEnabled){    g_enableLogger = -1;    }
//         else{                   g_enableLogger = 1;     }
    }
}


static void on_both_buttons_pressed()
{   // Reset uINS
	ioport_set_pin_output_mode(INS_RESET_PIN_PIN, IOPORT_PIN_LEVEL_LOW);
    s_ignoreCfgButtonRelease = true;
    s_ignoreLogButtonRelease = true;
}


static void on_both_buttons_release()
{   // De-assert uINS reset
    ioport_set_pin_input_mode(INS_RESET_PIN_PIN, 0, 0);
    refresh_CFG_LED();
}


#define BUTTON_DEBOUNCE_TIME_MS 100
void step_user_interface(uint32_t time_ms)
{
    bool cfgButtonDown = !ioport_get_pin_level(BUTTON_CFG_PIN);
    bool logButtonDown = !ioport_get_pin_level(BUTTON_LOG_PIN);
    bool bothButtonsDown = cfgButtonDown && logButtonDown;
    static bool cfgButtonDownLast = cfgButtonDown;
    static bool logButtonDownLast = logButtonDown;
    static bool bothButtonsDownLast = bothButtonsDown;
	static uint32_t cfgButtonTimeMs = 0;	// for button debouncing
	static uint32_t logButtonTimeMs = 0;
	static uint32_t bothButtonTimeMs = 0;

    if (cfgButtonDownLast != cfgButtonDown && time_ms-cfgButtonTimeMs > BUTTON_DEBOUNCE_TIME_MS)
    {   // Button toggled        
        cfgButtonDownLast = cfgButtonDown;
		cfgButtonTimeMs = time_ms;
        if(cfgButtonDown)
        {      
            if(s_funcPtrCfgButtonPressed){ s_funcPtrCfgButtonPressed(); }
        }
        else
        {
            if(!s_ignoreCfgButtonRelease)
            {
                if(s_funcPtrCfgButtonRelease){ s_funcPtrCfgButtonRelease(); }
            }
            s_ignoreCfgButtonRelease = false;
        }
    }

    if(logButtonDownLast != logButtonDown && time_ms-logButtonTimeMs > BUTTON_DEBOUNCE_TIME_MS)
    {   // Button toggled
        logButtonDownLast = logButtonDown;
		logButtonTimeMs = time_ms;
        if(logButtonDown)
        {      
            if(s_funcPtrLogButtonPressed){ s_funcPtrLogButtonPressed(); }
        }
        else
        {              
            if(!s_ignoreLogButtonRelease)
            {
                if(s_funcPtrLogButtonRelease){ s_funcPtrLogButtonRelease(); }
            }
            s_ignoreLogButtonRelease = false;
        }            
    }
    
    if(bothButtonsDownLast != bothButtonsDown && time_ms-bothButtonTimeMs > BUTTON_DEBOUNCE_TIME_MS)
    {   // Both buttons toggled
        bothButtonsDownLast = bothButtonsDown;
		bothButtonTimeMs = time_ms;
        if(bothButtonsDown)
        {                
            if(s_funcPtrBothButtonsPressed){ s_funcPtrBothButtonsPressed(); }  
        }
        else
        {
            if(s_funcPtrBothButtonsRelease){ s_funcPtrBothButtonsRelease(); }  
        }
    }
    
    if( logger_ready() )
    {   // Handle logger commands via g_status.loggerMode
        if(g_status.loggerMode!=EVB2_LOG_NA)
        {
            switch(g_status.loggerMode)
            {
                case EVB2_LOG_CMD_START:    enable_logger(true);    break;
                case EVB2_LOG_CMD_STOP:     enable_logger(false);   break;  // disable logger
            }
            g_status.loggerMode = EVB2_LOG_NA;
        }
    }
    else
    {   
        if(g_loggerEnabled==1)
        {   // Disable logger
            enable_logger(false);
        }
    }
    
}


void evbUiInit()
{
    s_funcPtrCfgButtonPressed = on_cfg_button_pressed;
    s_funcPtrCfgButtonRelease = on_cfg_button_release;
    s_funcPtrLogButtonPressed = on_log_button_pressed;
    s_funcPtrLogButtonRelease = on_log_button_release;
    s_funcPtrBothButtonsPressed = on_both_buttons_pressed;
    s_funcPtrBothButtonsRelease = on_both_buttons_release;
}

void evbUiInit(
    FuncPtrVoidVoid fpCfgButtonPressed, FuncPtrVoidVoid fpCfgButtonRelease, 
    FuncPtrVoidVoid fpLogButtonPressed, FuncPtrVoidVoid fpLogButtonRelease, 
    FuncPtrVoidVoid fpBothButtonsPressed, FuncPtrVoidVoid fpBothButtonsRelease )
{
    s_funcPtrCfgButtonPressed = fpCfgButtonPressed;
    s_funcPtrCfgButtonRelease = fpCfgButtonRelease;
    s_funcPtrLogButtonPressed = fpLogButtonPressed;
    s_funcPtrLogButtonRelease = fpLogButtonRelease;
    s_funcPtrBothButtonsPressed = fpBothButtonsPressed;
    s_funcPtrBothButtonsRelease = fpBothButtonsRelease;
}


