#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <inttypes.h>
#include <msg.h>

// This is simple code to control the MCP4725 analog to digital converter.
// Linux seems to have a kernel driver but it doesn't seem to have
// documentation on how to use it. It was easier to write my own code to
// access it.
//

static int file;

// Open and configure the i2c interface
void mcp4725_open()
{
  file = open("/dev/i2c-2", O_RDWR);
  if (file < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }
  if (ioctl(file, I2C_SLAVE, 0x60) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }
}

// Close the i2c interface
void mcp4725_close()
{
   close(file);
}


// Set the DAC and optionally eeprom. Value is 0-4095
// set_eeprom non zero writes value to both DAC and EEPROM
void mcp4725_set_dac(int value, int powerdown, int set_eeprom)
{
   int rc;

   if (set_eeprom) {
      unsigned char buf[3];

      buf[0] = 0x60 | (powerdown << 1); // Write to DAC register and EEPROM
      buf[1] = value >> 4;
      buf[2] = value << 4;
      if ((rc = write(file, buf, sizeof(buf))) != sizeof(buf)) {        
        /* ERROR HANDLING: i2c transaction failed */
        if (rc == -1) {
            // Print the error message using perror
            perror("Failed to write to the I2C device");
        } else {
         printf("DAC error #1, wrote %d bytes\n",rc); 
        }

        exit(1);
      } 
   } else {
      unsigned char buf[2];

      buf[0] = (value >> 8) | (powerdown << 4);
      buf[1] = value;
      if ((rc = write(file, buf, sizeof(buf))) != sizeof(buf)) {
        /* ERROR HANDLING: i2c transaction failed */
        if (rc == -1) {
            // Print the error message using perror
            perror("Failed to write to the I2C device");
        } else {
           printf("DAC error #2, wrote %d bytes\n",rc); 
        }
        exit(1);
      } 
   }
}

// Get the device status. value is the current DAC value 0-4095. eeprom_value
// is the value programmed in the EEPROM. pd is the two power down bits.
// busy is non zero if EEPROM write is in progress
void mcp4725_get_status(int *value, int *eeprom_value, int *pd, 
   int *eeprom_write_complete) 
{
   unsigned char buf[5];
   int rc;

   /* Using I2C Read, equivalent of i2c_smbus_read_byte(file) */
   if ((rc = read(file, buf, sizeof(buf))) != sizeof(buf)) {
     /* ERROR HANDLING: i2c transaction failed */             
      if (rc == -1) {
         // Print the error message using perror
         perror("Failed to write to the I2C device");
      } else {
         printf("DAC error #3, read %d bytes\n",rc); 
      }
     exit(1);
   }
   if (value != NULL)
      *value = ((int) buf[1] << 4) | (buf[2] >> 4);
   if (eeprom_value != NULL)
      *eeprom_value = (((int) (buf[3] & 0xf)) << 8) | buf[4];
   if (pd != NULL)
      *pd = (buf[0] >> 1) & 3;
   if (eeprom_write_complete != NULL)
      *eeprom_write_complete = buf[0] >> 7;
}

// This isn't really directly controlling the DAC, it is using GPIO
void mcp4725_disable_dac(int disable)
{
   static int dac_fd = -1;

   if (dac_fd == -1) {
      dac_fd = open("/sys/class/gpio/gpio69/direction", O_RDWR);
      if (dac_fd < 0) {
         msg(MSG_FATAL, "Unable to open DAC control pin\n");
         exit(1);
      }
   }
   if (disable) {
      write(dac_fd, "high", 4);
   } else {
      write(dac_fd, "low", 3);
   }
}
