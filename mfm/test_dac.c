#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "mcp4725.h"

// For creating emulator file you may want to set this to 1 since emulator
// file not always good if data recovered from multiple reads. For creating
// extracted data file 0 is better.
#define SINGLE_READ_GOOD 0

// 1 if you don't want the drive trying to seek to the correct track when it
// finds the wrong cylinder
#define DISABLE_OFFTRACK_SEEK 0

// Maximum voltage DAC can generate. 0 is minumum
#define DAC_FULL_SCALE 11.4
// Maximum voltage to use for retries
// 6.0 for RD53/Micropolis 1325 with 50k resistor to U31 pin 2
// 6.0 for RD54/Maxtor XT-2190 with 10k resistor U14 pin 13
// 6.0 for Miniscribe 6085 with 10k resistor to U6 pin 9
#define MAX_DAC 6.1
// Minimum voltage to use for retries
// 5.0 for RD53/Micropolis 1325 with 50k resistor
// 3.5 for RD54/Maxtor XT-2190 with 10k resistor
// 5.0 for Miniscribe 6085 with 10k resistor
#define MIN_DAC 5.0
// Nominal DAC voltage (head centered on track)
// 5.5 for RD53/Micropolis 1325 with 50k resistor
// 5.0 for RD54/Maxtor XT-2190 with 10k resistor
// 5.5 for Miniscribe 6085
#define NOMINAL_DAC 5.5
// Convert voltage to DAC binary value
#define DAC_COUNT(x) round(4095 * x / DAC_FULL_SCALE)

// Simple old * FILTER + new * (1-FILTER) filter
#define FILTER .9



// Remember to run commands to enable i2c mode when using pins for I2C-2
// config-pin P9_21 i2c
// config-pin P9_22 i2c
int main(int argc, char *argv[])
{
	int value;
    	int eeprom_value;
    	int power_down_mode;
    	int write_complete;
	int disableValue = 0;  // Default value if "off" is not specified

	 // Check if the command-line arguments contain "off"
    	for (int i = 1; i < argc; i++) {  // Start from 1 to skip the program name
       	    if (strcmp(argv[i], "off") == 0) {
	            disableValue = 1;  // Set to 1 if "off" is found
	    }
	}

	printf("Open...\n");
	mcp4725_open();

	if (disableValue == 0)
	{
		printf("Enable DAC output signal..\n");
	}
	else
	{
		printf("Disable DAC output signal..\n");
	}

	mcp4725_disable_dac(disableValue);
	
	float fval = 5.5;
	printf("Set DAC value %f\n", fval);


	mcp4725_set_dac(DAC_COUNT(fval),MCP4725_PD_NONE,0);  // MCP4725_PD_NONE = 0

   // Call the function with the addresses of the variables
    mcp4725_get_status(&value, &eeprom_value, &power_down_mode, &write_complete);

    // Print the results
    printf("Current DAC Output Value: %d\n", value);
    printf("EEPROM Saved Value: %d\n", eeprom_value);
    printf("Power-down mode: %d\n", power_down_mode);
    printf("EEPROM Write Completion Status: %d\n", write_complete);
}
