// This module is routines for reading the disk drive.
// drive_setup sets up the drive for reading.
// drive_read_disk reads the disk drive.
// drive_read_track read a track of deltas from a drive. It steps the
//    head if necessary.
// 
// The drive must be at track 0 on startup or drive_seek_track0 called.
//
// 06/02/2023 DJG Fixed write fault error reading NEC drive
// 07/05/2019 DJG Added support for using recovery signal
// 03/09/2018 DJG Added logic to not retry when requested to read more
//   cylinders or heads than analyze determined
// 11/05/2016 DJG Made retry seek progression more like it was before change below
// 10/16/2016 DJG Added control over seek on retry
// 10/02/2016 DJG Rob Jarratt change to clean up confusing code.
//
// Copyright 2023 David Gesswein.
// This file is part of MFM disk utilities.
//
// MFM disk utilities is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MFM disk utilities is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MFM disk utilities.  If not, see <http://www.gnu.org/licenses/>.
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <inttypes.h>
#include <time.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "cmd.h"
#include "cmd_write.h"
#include "deltas_read.h"
#include "pru_setup.h"
#include "drive.h"
#include "board.h"

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
#define MAX_DAC 6.0
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


// Read the disk.
//  drive params specifies the information needed to decode the drive and what
//    files should be written from the data read
//
// drive_params: Drive parameters
// deltas: Memory array containing the raw MFM delta time transition data
void drive_read_disk(DRIVE_PARAMS *drive_params, void *deltas, int max_deltas, int dac_servo_mode)
{   
   // Loop variable
   int cyl, head;
   int cntr;
   // Retry counter, counts up
   int err_cnt;
      // No seek retry counter, counts down
   int no_seek_count;
   // Read status
   SECTOR_DECODE_STATUS sector_status = 0;
   // The status of each sector
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   // We use this to summarize the list above has any errors
   int sect_err;
   // For retry we do various length seeks. This is the maximum length we will do
   int seek_len;
   // Cylinder difference when a seek error occurs
   int seek_difference;

   // Timing stuff for testing
#ifdef CLOCK_MONOTONIC_RAW
#define CLOCK CLOCK_MONOTONIC_RAW
#else
#define CLOCK CLOCK_MONOTONIC
#endif
   struct timespec tv_start;
   double start, last = 0;
   double min = 9e9, max = 0, tot = 0;
   int count = 0;
   int recovery_active = 0;


   int last_good = 0;

   // Voltage to start with for each head
   float head_dac[MAX_HEAD];
   // zero step to lower voltage
   int head_dir[MAX_HEAD];
   float cur_dac=NOMINAL_DAC;

   if (dac_servo_mode)
   {
      mcp4725_open();
      // Disable DAC output
      mcp4725_disable_dac(1);

      for (cntr = 0; cntr < MAX_HEAD; cntr++) {
         head_dac[cntr] = NOMINAL_DAC;
         head_dir[cntr] = 1;
      }
   }


   // Start up delta reader
   deltas_start_thread(drive_params);

   mfm_decode_setup(drive_params, 1);
   
   for (cyl = 0; cyl < drive_params->num_cyl; cyl++) {
      if (cyl % 5 == 0)
         msg(MSG_PROGRESS, "At cyl %d\r", cyl);
      for (head = 0; head < drive_params->num_head; head++) {
         int recovered = 0;
         int retries;

         if (dac_servo_mode)
            cur_dac = head_dac[head];

         err_cnt = 0;
         no_seek_count = drive_params->no_seek_retries;
         seek_len = 1;
         // Clear sector status here so we can see if different sectors
         // successfully read on different reads.
         mfm_init_sector_status_list(sector_status_list, drive_params->num_sectors);
         if (cyl >= drive_params->noretry_cyl || 
             head >= drive_params->noretry_head) {
            retries = 0;
         } else {
            retries = drive_params->retries;
         }
         do {
            if (dac_servo_mode)
            {
               mcp4725_set_dac(DAC_COUNT(cur_dac), MCP4725_PD_NONE, 0);
               printf("Setting DAC to %.2f cyl %d head %d last good %d\n",cur_dac, cyl, head, last_good);
            }
            else
            {
               // First error retry without seek, second with third without etc
               // Hopefully by moving head it will settle slightly differently
               // allowing data to be read
               if (no_seek_count <= 0) {
                  int clip_seek_len = seek_len;
   
                  // In recovery mode the drive will perform a microstep instead
                  // of a full cylinder step. The sequency of microsteps positions
                  // will repeat after a drive dependent number of steps
                  if (drive_params->recovery) {
                     if (!recovery_active) {
                        drive_enable_recovery(1);
                        recovery_active = 1;
                     }
                     drive_step(drive_params->step_speed, 1, 
                        DRIVE_STEP_NO_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
                  } else {
                     if (cyl + seek_len >= drive_params->num_cyl) {
                        clip_seek_len = drive_params->num_cyl - cyl - 1;
                     }
                     if (cyl + seek_len < 0) {
                        clip_seek_len = -cyl;
                     }
                     //printf("seeking %d %d %d\n",err_cnt, seek_len, seek_len + cyl);
                     if (clip_seek_len != 0) {
                        drive_step(drive_params->step_speed, clip_seek_len, 
                           DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
                        drive_step(drive_params->step_speed, -clip_seek_len, 
                           DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
                     }
                     if (seek_len < 0) {
                        seek_len = seek_len * 2;
                        if (seek_len <= -drive_params->num_cyl) {
                           seek_len = -1;
                        }
                     }
                     seek_len = -seek_len;
                     if (seek_len == 0) {
                        seek_len = 1;
                     }
                  }
                  no_seek_count = drive_params->no_seek_retries;
               }
               no_seek_count--;
            }

            clock_gettime(CLOCK, &tv_start);
            start = tv_start.tv_sec + tv_start.tv_nsec / 1e9;
            if (last != 0) {
#if 0
               if (start-last > 40e-3)
                  printf("gettime delta ms %f\n", (start-last) * 1e3);
#endif
               if (start-last > max)
                  max = start-last;
               if (start-last < min)
                  min = start-last;
               tot += start-last;
               count++;
            }
            last = start;

            drive_read_track(drive_params, cyl, head, deltas, max_deltas,0); // int dac_servo_mode
            
            if (dac_servo_mode)
               mfm_init_sector_status_list(sector_status_list, drive_params->num_sectors);

            sector_status = mfm_decode_track(drive_params, cyl, head,
               deltas, &seek_difference, sector_status_list);

            // See if sector list shows any with errors. The sector list
            // contains the information on the best read for each sector so
            // even if the last read had errors we may have recovered all the
            // data without errors.
            sect_err = 0;

            if (dac_servo_mode)
               last_good = 0;
      
            for (cntr = 0; cntr < drive_params->num_sectors; cntr++) {
               if (UNRECOVERED_ERROR(sector_status_list[cntr].status)) {
                  sect_err = 1;
               } 
               else if (dac_servo_mode) {
                  last_good++;
               }
            }
            if (!sect_err) {
     //          msg(MSG_INFO, "Good multiple read cyl %d head %d\n", cyl, head);
            }

            if (dac_servo_mode)
               {
               // Try to get single read without error
 #if SINGLE_READ_GOOD
               if (sector_status & (SECT_BAD_HEADER | SECT_BAD_DATA)) {
                  sect_err = 1;
               }
 #endif
            }
            // Looks like a seek error. Sector headers which don't have the
            // expected cylinder such as when tracks are spared can trigger
            // this also
            if ((sector_status & SECT_WRONG_CYL)) {
               if (dac_servo_mode)
               {
               //if ((sector_status & SECT_WRONG_CYL) && seek_difference > 1) {
               //if ((sector_status & SECT_WRONG_CYL) && (seek_difference > 1 || new_track)) {
                  if (drive_at_track0()) {
                     printf("At track0 seek err\n");
                     seek_difference = cyl;
                  }
               }
#if DISABLE_OFFTRACK_SEEK
               if (err_cnt < retries) {
                  msg(MSG_ERR, "Retrying seek cyl %d, cyl off by %d\n", cyl,
                        seek_difference);

                  if (dac_servo_mode)
                  {
                     // Disable DAC output
                     mcp4725_disable_dac(1);
                     usleep(10); // Allow lines to settle
                  }
                  // Retry seek
                  drive_step(drive_params->step_speed, seek_difference, 
                     DRIVE_STEP_NO_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
                  
                  if (dac_servo_mode)
                  {
                     printf("Step done\n");
                     //sleep(2);
                     mcp4725_disable_dac(0);
                     printf("DAC set back to %f\n", cur_dac);
                     //sleep(2);
                  }
               }
#endif
            }

            if (UNRECOVERED_ERROR(sector_status) && !sect_err) {
               recovered = 1;
            }

            if (dac_servo_mode)
            {
               if (sect_err) {
                  if (head_dir[head]) { 
                     cur_dac += (MAX_DAC - NOMINAL_DAC) / retries * 2;
                     if (cur_dac > MAX_DAC) {
                        cur_dac = NOMINAL_DAC;
                        head_dir[head] = 0;
                     }
                  } else {
                     cur_dac -= (NOMINAL_DAC - MIN_DAC) / retries * 2;
                     if (cur_dac < MIN_DAC) {
                        cur_dac = NOMINAL_DAC;
                        head_dir[head] = 1;
                     }
                  }
               } else {
                  head_dac[head] = head_dac[head] * FILTER + cur_dac * (1.0 - FILTER);
               }
            }

         // repeat until we get all the data or run out of retries
         } while (sect_err && err_cnt++ < retries);
                  if (recovery_active) {
           drive_enable_recovery(0);
           // My ST225 doesn't follow the manual behavior of inactivating seek
           // complete after recovery goes inactive while it repositions the 
           // heads. The heads seem to be moving since I get data from several
           // cylinders. If a step is done the drive gets confused and needs 
           // to be power cylcled. This delay seems to make it work.
           usleep(25000);
           // Wait for seek complete 
           if (pru_exec_cmd(CMD_CHECK_READY, 0)) {
              drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
              exit(1);
           }
           recovery_active = 0;
         }
         if (err_cnt > 0) {
            if (err_cnt == retries + 1) {
               msg(MSG_ERR, "Retries failed cyl %d head %d\n", cyl, head);
            } else {
               msg(MSG_INFO, "All sectors recovered %safter %d retries cyl %d head %d\n",
                     recovered ? "from multiple reads " : "", err_cnt, cyl, head);
            }
         }
      }
   }

   mfm_decode_done(drive_params);

   printf("Track read time in ms min %f max %f avg %f\n", min * 1e3, 
         max * 1e3, tot * 1e3 / count);

   deltas_stop_thread();

   if (dac_servo_mode)
      mcp4725_close();

   return;
}

// Read a track of deltas from a drive
//
// drive_params: Drive parameters
// cyl, head: Cylinder and head reading. Head is stepped to cylinder
//    and head selected
// deltas: Memory array containing the raw MFM delta time transition data
// max_deltas: Size of deltas array
// return_write_fault: Non zero if function should return if write fault error present
//   otherwise write fault is fatal error.
//
// return non zero if write fault present and return_write_fault true.
int drive_read_track(DRIVE_PARAMS *drive_params, int cyl, int head, 
      void *deltas, int max_deltas, int return_write_fault) { //},int dac_servo_mode ) { 
   
   int dac_servo_mode = 1;

   if (cyl != drive_current_cyl()) {
      if (dac_servo_mode)
      {
         // Disable DAC output
         mcp4725_disable_dac(1);
         usleep(10); // Allow lines to settle
         printf("Stepping cyl %d cur %d\n", cyl, drive_current_cyl());
      }
      
      drive_step(drive_params->step_speed, cyl - drive_current_cyl(), 
         DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
   }


   // Reenable DAC output
   if (dac_servo_mode)
      mcp4725_disable_dac(0);

      // Analyze can change so set it every time
   pru_write_word(MEM_PRU0_DATA, PRU0_START_TIME_CLOCKS,
      drive_params->start_time_ns / CLOCKS_TO_NS);

   drive_set_head(head);

   if (pru_exec_cmd(CMD_READ_TRACK, 0) != 0) {
      drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
      if (drive_has_write_fault() && return_write_fault) {
         return 1;
      } else {
         exit(1);
      }
   } else {
      // OK to start reading deltas
      deltas_start_read(cyl, head);
   }
   return 0;
}
