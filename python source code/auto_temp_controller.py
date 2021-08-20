import sys
import os
import binascii
import time
import configparser
import logging
import datetime
import traceback
import random
import collections
from math import ceil
from bisect import bisect_left
import shutil


#######################################
# Define Global Variables
#######################################
config_file = 'temp_config.cfg'
log_file = 'temp_controller_log.log'
csv_temp_log_file = 'temps_vs_time_log.csv'
converted_command_file = 'converted_command.txt'
excel_command_file = 'excel_to_python_commands.txt'
excel_command_file_copy = 'excel_to_python_commands_copy.txt'


#######################################
# Logging methods
#######################################
# For reference logging levels include: CRITICAL, ERROR, WARNING, INFO, DEBUG
# Default to INFO
def writeOut(msg):
    logging.info(msg)
    print(msg)

def writeOutDebug(msg):
    logging.debug('   ' + msg)
    print('   ' + msg)

def writeOutCrit(msg):
    logging.critical(msg)
    print('CRITICAL: ' + msg)

def writeOutWarn(msg):
    logging.warning(msg)
    print('Warning: ' + msg)

#######################################
# Interpolate method
#######################################

# Return the current value and current slope
class Interpolate(object):
    def __init__(self, x_list, y_list):
        if any(x >= y for x, y in zip(x_list, x_list[1:])):
            writeOutCrit('Not all temperature profile points are increasing in time')
            sys.exit()
        self.x_list = x_list
        self.y_list = y_list
        intervals = zip(x_list, x_list[1:], y_list, y_list[1:])
        self.slopes = [(y2 - y1)/(x2 - x1) for x1, x2, y1, y2 in intervals]

    def __call__(self, x):
        if x <= self.x_list[0]:
            curr_val = self.y_list[0]
            return (curr_val, 0) # Assume slope = 0
        elif x >= self.x_list[-1]:
            curr_val = self.y_list[-1]
            return (curr_val, 0) # Assume slope = 0
        else:
            i = bisect_left(self.x_list, x) - 1
            curr_val = self.y_list[i] + self.slopes[i] * (x - self.x_list[i])
            return (curr_val, self.slopes[i]) # Return actual slope = 0

#######################################
# Read Data From Excel
#######################################

global last_line,failure
last_line = None
failure = False
def get_entry_from_file():
    global last_line,failure
    # Expects a string with: time into log in seconds, white space, current temperature
    try:
        # Copy the Excel command file before reading to avoid interference
        shutil.copyfile(excel_command_file, excel_command_file_copy)
    except Exception as ex:
        if last_line is None:
            # Don't give an error if on the 1st call
            return None
        writeOutCrit('Failed to copy Excel command file!\n' + str(ex))
        failure = True
        return None

    # Parse the copied file
    try:
        with open(excel_command_file_copy,'r') as f:
            lines = f.readlines()
        lines = [line.strip() for line in lines]
        if(last_line is None or lines[-1] != last_line):
            # First pass or there is new data in the file
            line = lines[-1]
            if last_line is None and len(line) == 0:
                # No data yet
                return None
            if last_line is None:
                # 1st data
                line = lines[0]
                last_line = line
            else:
                # Need to locate location of new data
                found = False
                for line_in_data in lines:
                    if not found:
                        # Found old data location, mark it so next line is output
                        if line_in_data == last_line:
                            found = True
                    else:
                        line = line_in_data
                        last_line = line
                        break
            return line
        else:
            return None
    except Exception as ex:
        failure = True
        writeOutCrit('Failed to read Excel command file!\n' + str(ex))

    try:
        # Delete the copied file
        os.remove(excel_command_file_copy)
    except Exception as ex:
        failure = True
        writeOutCrit('Failed to remove Excel command file copy!\n' + str(ex))    
    return None        

#######################################
# Main Code
#######################################
try:

    # Store the start time
    start_time = time.time()

    # Configure logging
    logging.basicConfig(level=logging.DEBUG, filename=log_file, filemode='w', format='%(asctime)-15s %(levelname)-8s %(message)s')
    writeOut("*** Temp Controller Script Start ***")

    #######################################
    # Get the average sample time
    #######################################
    # The first item in the file should be the average sample time in seconds
    while True:
        line = get_entry_from_file()
        if line is not None:
            avg_sample_time_hr = float(line)/3600
            break
    writeOut('Average sample time (hr): ' + repr(avg_sample_time_hr))

    #######################################
    # Read data from config file
    #######################################
    writeOut('Reading Config File')
    config = configparser.ConfigParser()
    config.read(config_file)
    writeOut('Finished Reading Config File')
    writeOut('Processing Config File')

    # Output temp limits
    writeOutDebug('Reading in min / max output temp limits')
    max_output_temp = config.getfloat('GENERAL', 'max_output_temp')
    min_output_temp = config.getfloat('GENERAL', 'min_output_temp')
    writeOutDebug('~ max_output_temp: ' + repr(max_output_temp))
    writeOutDebug('~ min_output_temp: ' + repr(min_output_temp))

    # Body temp limits
    writeOutDebug('Reading in min / max body temp limits')
    max_body_temp = config.getfloat('GENERAL', 'max_body_temp')
    min_body_temp = config.getfloat('GENERAL', 'min_body_temp')
    writeOutDebug('~ max_body_temp: ' + repr(max_body_temp))
    writeOutDebug('~ min_body_temp: ' + repr(min_body_temp))

    # PID gain terms
    writeOutDebug('Reading in PID gain terms')
    proportional_term = config.getfloat('PID_GAIN_TERMS', 'proportional_term')
    integral_time_min = config.getfloat('PID_GAIN_TERMS', 'integral_time_min')
    derivative_time_min = config.getfloat('PID_GAIN_TERMS', 'derivative_time_min')
    integral_persistence_time_min = config.getfloat('PID_GAIN_TERMS', 'integral_persistence_time_min')
    writeOutDebug('~ proportional_term: ' + repr(proportional_term))
    writeOutDebug('~ integral_time_min: ' + repr(integral_time_min))
    writeOutDebug('~ derivative_time_min: ' + repr(derivative_time_min))
    writeOutDebug('~ integral_persistence_time_min: ' + repr(integral_persistence_time_min))

    # Calculate gain terms in the form they will be used
    kp = proportional_term
    ki = kp / (integral_time_min / 60.) #different form, based on hours
    kd = kp * (derivative_time_min / 60.) #different form, based on hours

    # Temperature profile
    writeOutDebug('Reading in target temp profile')
    temp_profile_times = []
    temp_profiles = []
    for option in config.options('TARGET_TEMP_PROFILE'):
        temp_profile_times.append(float(option))
        temp_profiles.append(float(config.get('TARGET_TEMP_PROFILE', option)))
        writeOutDebug('~ target time, temp: ' + repr(float(option)) + ', ' + repr(float(config.get('TARGET_TEMP_PROFILE', option))))
    # Setup the temperature profile interpolator
    temp_profile_interp = Interpolate(temp_profile_times, temp_profiles)

    # Temperature dropout settings
    writeOutDebug('Reading in temperature dropout settings')
    dropout_delta_temp_falling = config.getfloat('TEMP_DROPOUT_SETTINGS', 'dropout_delta_temp_falling')
    dropout_delta_temp_steady = config.getfloat('TEMP_DROPOUT_SETTINGS', 'dropout_delta_temp_steady')
    dropout_delta_temp_rising = config.getfloat('TEMP_DROPOUT_SETTINGS', 'dropout_delta_temp_rising')
    writeOutDebug('~ dropout_delta_temp_falling: ' + repr(dropout_delta_temp_falling))
    writeOutDebug('~ dropout_delta_temp_steady: ' + repr(dropout_delta_temp_steady))
    writeOutDebug('~ dropout_delta_temp_rising: ' + repr(dropout_delta_temp_rising))

    writeOut('Finished Processing Config File')
    
    ###########################################
    # Methods for formatting TC-720 commands
    ###########################################
    writeOutDebug('Initializing TC-720 command methods')
    def calculate_checksum(command): #part of generating command for temp controller
        checksum = 0
        for letter in command:
            checksum += int(ord(letter))
        return str(hex(checksum))[-2:]
            
    def create_command(set_temp): #creates command to be read by the TC-720
        set_temp *= 100
        if set_temp < 0:
            set_temp = 2**16 - abs(int(set_temp))
        command = '1c'
        command += str(hex(int(set_temp)))[2:].zfill(4)
        command += calculate_checksum(command)
        return command

    def write_out_temp(set_temp):
        print('Writing out temperature of ' + str(set_temp)+ ' degrees')
        with open(converted_command_file,'w+') as out_file:
            out_file.write(create_command(set_temp))
            out_file.close()

    #########################################
    # MAIN LOOP
    #########################################
    try:
        writeOut("Entering Main Code Loop")

        # Initialize temperature control variables
        num_readings = 0
        last_integral = 0
        last_error = 0
        time_temp_deque = collections.deque()
        deque_len = int(ceil(integral_persistence_time_min / avg_sample_time_hr / 60))
        
        # Setup output CSV file for temp logs
        writeOut('Beginning temperature control')
        first_reading = True

        with open(csv_temp_log_file,'w+') as csv_temp_log:

            while True:
                try:                                           
                    # Wait for update from Excel/VBA
                    try:
                        # Parse new data from the text file
                        newData = get_entry_from_file()
                        if newData is None:
                            if failure:
                                break
                            continue
                        writeOut('Received Input from Excel: "' + newData + '"')

                        try:
                            newDataParts = newData.split() # split by whitespace
                            absTime_hr = float(newDataParts[0]) / 3600.0 # convert from s to hr
                            newVal = float(newDataParts[1])
                            validFraction = float(newDataParts[2])

                        except Exception as ex:
                            writeOutCrit("Error parsing data string: '" + newData + "'")
                            break

                    except EOFError:
                        # No more data coming in; close the script
                        writeOutWarn('Excel stopped sending data; Will close')
                        break
                    
                    # Only update if the latest readings are valid (not all dropped data)
                    validReading = validFraction > 0
                    if validReading:
                        # Parse as double with error trapping
                        body_temp = float(newVal)
                    else:
                        writeOutWarn('Body temp readings are invalid: Applying temp data dropout methods')

                    # Bound rat temp readings
                    if body_temp > max_body_temp:
                        body_temp = max_body_temp
                        writeOutWarn('Body temp capped at max temp')
                    elif body_temp < min_body_temp:
                        body_temp = min_body_temp
                        writeOutWarn('Body temp capped at min temp')

                    # Store the initial time so that elapsed time can be determined
                    if first_reading:
                        first_absTime_hr = absTime_hr                        
                        
                    # Lookup the current target temperature based on the elapsed time
                    elapsedTime_hr = absTime_hr - first_absTime_hr
                    tgt_temp_and_slope = temp_profile_interp(elapsedTime_hr)
                    tgt_temp = tgt_temp_and_slope[0]
                    tgt_slope = tgt_temp_and_slope[1]

                    # Calculate the current error
                    curr_err = body_temp - tgt_temp

                    # Calculate the PID controller terms
                    if first_reading:
                        first_reading = False
                        delta_time_hr = 0
                        time_temp_deque.append((delta_time_hr, curr_err))

                        integral_term = 0
                        derivative_term = 0
                        # Write the file header
                        csv_temp_log.write('Reading,Notocord_Time_s,Elapsed_Time_hr,Valid_Reading_%,RatTemp,TgtTemp,TempErr,TempOutput\n')
                    else:

                        # Store the current delta time and error
                        delta_time_hr = absTime_hr - last_reading_time_hr

                        # Add the current delta time and temp to the deque
                        if validReading:
                            time_temp_deque.append((delta_time_hr, curr_err))
                        else:
                            # Current temp not known - default to no error
                            time_temp_deque.append((delta_time_hr, 0))

                        if len(time_temp_deque) > deque_len:
                            removed = time_temp_deque.popleft()
                            # Remove the contribution from the oldest data
                            removed_delta_time_hr = removed[0]
                            removed_err = removed[1]
                            last_integral -= removed_err * removed_delta_time_hr

                        # Calculate the PID gain terms
                        if validReading:
                            integral_term = last_integral + curr_err * delta_time_hr
                            derivative_term = (curr_err - last_error) / delta_time_hr

                    # Calculate the ideal temperature controller output
                    if validReading:
                        # Nominal PID temperature output
                        temp_output = kp * curr_err + ki * integral_term + kd * derivative_term
                    else:
                        # Output temperature based on dropout logic
                        if abs(tgt_slope) < .001:
                            # Zero or close -> target temperature is steady
                            temp_output = tgt_temp + dropout_delta_temp_steady
                            writeOutWarn('Applying temp data dropout method for temperature phase: steady')
                        elif tgt_slope > 0:
                            # Target temperature is rising
                            temp_output = tgt_temp + dropout_delta_temp_rising
                            writeOutWarn('Applying temp data dropout method for temperature phase: rising')
                        else:
                            # Target temperature is falling
                            temp_output = tgt_temp + dropout_delta_temp_falling
                            writeOutWarn('Applying temp data dropout method for temperature phase: falling')

                    # Bound temperature output
                    if temp_output > max_output_temp:
                        temp_output = max_output_temp
                        writeOutWarn('Temp output capped at max temp')
                    elif temp_output < min_output_temp:
                        temp_output = min_output_temp
                        writeOutWarn('Temp output capped at min temp')

                    # Update the PID storage values
                    if validReading:
                        last_integral += curr_err * delta_time_hr
                        last_error = curr_err

                    last_reading_time_hr = absTime_hr
                    num_readings += 1

                    # Create a command file for the latest output temperature
                    write_out_temp(temp_output)

                    # Log the latest data
                    new_log_data = repr(num_readings) + ', ' + repr(3600*absTime_hr) + ', ' + repr(elapsedTime_hr) + ', ' + \
                        repr(validFraction) + ', ' + \
                        repr(body_temp) + ', ' + repr(tgt_temp) + ', ' + repr(curr_err) + ', ' + repr(temp_output)
                    writeOutDebug('Reading, Notocord_Time_s, Elapsed_Time_hr, Valid_Reading_%, RatTemp, TgtTemp, TempErr, TempOutput: ' + new_log_data)
                    csv_temp_log.write(new_log_data + '\n')
                    csv_temp_log.flush()

                except:
                    writeOut("Error Parsing Input")
                    writeOut(traceback.format_exc())
                    break

        writeOut('Ending temperature control')
        writeOutWarn("*** Exiting Temp Controller Script ***")

    except Exception as e_main_loop:
        writeOutCrit(str(e_main_loop))
        input('\n\nError in Main Loop\n\nExiting: Press ENTER to exit. . .')
        time.sleep(3)

except Exception as e_init:
    writeOutCrit(str(e_init))
    input('\n\nError in Initialization\n\nExiting: Press ENTER to exit. . .')
    time.sleep(3)