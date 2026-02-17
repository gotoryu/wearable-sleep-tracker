import os
import paho.mqtt.client as mqtt
import time
from queue import Queue
from threading import Thread
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from scipy.signal import find_peaks
from scipy.optimize import curve_fit

########## MQTT-Server-Client variables ##########
global flag_connected
flag_connected = 0

# Thread-safe Queues for data
pulse_data_queue = Queue()
movement_data_queue = Queue()
##################################################

################ current raw data ################
pulse_values = np.array([], dtype=np.float64)
rec_p_time = np.array([], dtype=np.float64)
pulse_time = np.array([], dtype=np.float64)

gyro_time = np.array([], dtype=np.float64)
rec_g_time = np.array([], dtype=np.float64)
g_mean_x = np.array([], dtype=np.float64)
g_mean_y = np.array([], dtype=np.float64)
g_mean_z = np.array([], dtype=np.float64)
gyro_x = np.array([], dtype=np.float64)
gyro_y = np.array([], dtype=np.float64)
gyro_z = np.array([], dtype=np.float64)
##################################################

################ processed data ##################
detected_peaks = np.array([], dtype=np.int32)
p_val_count = 0

g_mean_sum = np.array([], dtype=np.float64)
g_val_count = 0
g_integrals = np.array([], dtype=np.float64)
gyro_threshold = 0.0
##################################################

######### saved movement and pulse data ##########
heartbeat = 0
heartbeat_values = np.array([], dtype=np.float64)
movement_values = np.array([], dtype=np.float64)
##################################################

################### time frame ###################
time_window = 10 # time window in seconds
window_size = -1  # size of currently observed data
first_ts_pulse = -1
first_ts_gyro = -1
##################################################

###### variables for Fourier Transformation ######
fft_pulse_values = np.array([], dtype=np.float64)
detected_fft_peaks = np.array([], dtype=np.int32)
fft_x_axis = None
fft_start = -1
fft_end = -1
##################################################

#################### results #####################
heartbeat_detected = False
##################################################

# calculate the window size by the delta time of currently observed values
def calculate_window_size_dynamically():
    return round(time_window * 1000 / np.mean(np.diff(rec_p_time)))

def clean_heartbeat_data():
    global heartbeat_values, pulse_time
    i = 1
    initial_time_value = pulse_time[-i] - time_window * 1000
    while pulse_time[-i] > initial_time_value: i += 1
    mean = np.mean(heartbeat_values[-i:])
    print("last values: ", heartbeat_values[-i:])
    if np.std(heartbeat_values[-i:]) < 12:
        while i > 0:
            if abs(heartbeat_values[-i] - mean) > 10:
                print("removing value: ", heartbeat_values[-i])
                np.delete(heartbeat_values, heartbeat_values.size - i)
                np.delete(pulse_time, pulse_time.size - i)
            i -= 1
    else:
        print("high deviation of values")

# analyze pulse via scipy-peak-detection
def analyze_pulse_data():
    global pulse_values, rec_p_time, detected_peaks, time_window
    heartbeat = 0

    # if there is enough values to calculate a pulse from
    if pulse_values.size >= window_size:
        # dynamic threshold for peak recognition with mean and deviation
        min_peak_height = np.mean(pulse_values) + 0.5 * np.std(pulse_values)

        # peak recognition (scipy)
        detected_peaks, _ = find_peaks(pulse_values, height=min_peak_height, distance=0.25 * time_window)

        # if a peak was detected
        if detected_peaks.size > 1:
            peak_time_stamps = rec_p_time[detected_peaks]
            average_peak_interval = np.mean(np.diff(peak_time_stamps))
            heartbeat = 60000 / average_peak_interval
            # the human pulse stays more or less always between 40 and 200 bpm
            if 40 < heartbeat < 200:
                print(f"hearbeat (peak detection): {heartbeat:.1f} bpm")
            else:
                print("hearbeat (peak detection): ---- bpm")
        else:
            print("hearbeat (peak detection): no peaks")
            
# Define Gaussian function
def gaussian(x, A, x0, sigma):
    return A * np.exp(-(x - x0)**2 / (2 * sigma**2))
    
def gaussian_fit_frequency():
    global fft_pulse_values, fft_x_axis, fft_start, fft_end, detected_fft_peaks, heartbeat
    # Select a window around the peak
    min_peak_height = (
        np.mean(fft_pulse_values[fft_start:fft_end])
        + 0.4 * np.std(fft_pulse_values[fft_start:fft_end])
    )
    detected_fft_peaks, _ = find_peaks(
        fft_pulse_values[fft_start:fft_end], 
        height=min_peak_height
    )
    detected_fft_peaks += fft_start
    if detected_fft_peaks.size > 0:
        #find the peak with the highest value
        peak_index = detected_fft_peaks[max(
            range(len(fft_pulse_values[detected_fft_peaks])), key = fft_pulse_values[detected_fft_peaks].__getitem__
        )]
        
        window_size = 3  # Adjust based on the data
        start = max(fft_start, peak_index - window_size)
        end = min(len(fft_pulse_values[fft_start:fft_end]), peak_index + window_size)
        x_data = fft_x_axis[start:end]
        y_data = fft_pulse_values[start:end]

        try:
            # Perform Gaussian fit
            popt, _ = curve_fit(gaussian, x_data, y_data, p0=[np.max(y_data), fft_x_axis[peak_index], 5.0])
            _, refined_frequency, _ = popt

            heartbeat = refined_frequency

            print(f"hearbeat (fft+gauss fit):  {refined_frequency:.1f} bpm", end="\t", flush=True)
        except:
            print(f"hearbeat (fft+gauss fit):  ---- bpm", end="\t", flush=True)
        print("peak at: ", fft_x_axis[peak_index])
    else:
        print(f"hearbeat (fft+gauss fit):  no peaks")

# show data with matplotlib (background thread)
def display_data_live():
    global pulse_values, rec_p_time, detected_peaks
    
    def update(frame):
        global first_ts_pulse, first_ts_gyro, window_size
        global pulse_values, rec_p_time, detected_peaks
        global p_val_count, pulse_time, heartbeat, heartbeat_values
        global gyro_x, gyro_y, gyro_z, g_mean_x, g_mean_y, g_mean_z, rec_g_time
        global fft_pulse_values, fft_x_axis, fft_start, fft_end, detected_fft_peaks
        global g_val_count, gyro_sum, g_integrals, gyro_time, gyro_threshold

        # refresg data from queues
        while not pulse_data_queue.empty():
            data = pulse_data_queue.get()
            if first_ts_pulse < 0: first_ts_pulse = data["timestamp"][0]
            new_timestamp = data["timestamp"][0] - first_ts_pulse

            pulse_values = np.append(pulse_values, data["value"])
            rec_p_time = np.append(rec_p_time, new_timestamp)
            
            # limit values
            if window_size > 0:
                pulse_values = pulse_values[-window_size:]
                rec_p_time = rec_p_time[-window_size:]
            
            pulse_data_queue.task_done()
        
        while not movement_data_queue.empty():
            data = movement_data_queue.get()
            if first_ts_gyro < 0: first_ts_gyro = data["timestamp"][0]
            new_timestamp = data["timestamp"][0] - first_ts_pulse
            #gyro_time = np.append(rec_g_time, new_timestamp)
            rec_g_time = np.append(rec_g_time, new_timestamp)
            
            gyro_x = np.append(gyro_x, data["gyro_x"])
            gyro_y = np.append(gyro_y, data["gyro_y"])
            gyro_z = np.append(gyro_z, data["gyro_z"])
            
            g_mean_x = np.append(g_mean_x, np.mean(gyro_x))
            g_mean_y = np.append(g_mean_y, np.mean(gyro_y))
            g_mean_z = np.append(g_mean_z, np.mean(gyro_z))
            
            gyro_x = abs(gyro_x)
            gyro_y = abs(gyro_y)
            gyro_z = abs(gyro_z)
            gyro_sum = gyro_x + gyro_y + gyro_z
            gyro_sum -= np.mean(gyro_sum)
            
            # limit values
            if window_size > 0:
                gyro_x = gyro_x[-window_size:]
                gyro_y = gyro_y[-window_size:]
                gyro_z = gyro_z[-window_size:]
                gyro_sum = gyro_sum[-window_size:]
                g_mean_x = g_mean_x[-window_size:]
                g_mean_y = g_mean_y[-window_size:]
                g_mean_z = g_mean_z[-window_size:]
                rec_g_time = rec_g_time[-window_size:]
            
            g_val_count += 1
            recent_g_integral = 0
            # every 10 seconds
            if g_val_count == int(60/time_window * window_size / 2):
                for i in range(window_size): recent_g_integral += gyro_sum[i] if gyro_sum[i] > 0 else 0
                g_integrals = np.append(g_integrals, recent_g_integral)
                gyro_time = np.append(gyro_time, new_timestamp/60000)
                g_val_count = 0
            
            movement_data_queue.task_done()

        gyro_x -= np.mean(gyro_x)
        gyro_y -= np.mean(gyro_y)
        gyro_z -= np.mean(gyro_z)

        if rec_p_time.size != 0 and rec_g_time.size != 0:
            if window_size < 0:
                window_size = calculate_window_size_dynamically()
                print("window_size: ", window_size)

            print("######################################################################")
            fft_pulse_values = abs(np.fft.fft(pulse_values))
            analyze_pulse_data()

            # Plot 2: smoothed Pulse Data and Peaks
            ax2.clear()
            ax2.plot(rec_p_time, pulse_values, color="orange")
            #ax2.plot(rec_p_time, np.mean(pulse_values), label="mean", color="purple")
            ax2.plot(rec_p_time[detected_peaks], pulse_values[detected_peaks], 'x', color="red")
            ax2.set_title("Pulse Analysis (Smoothed & Peaks)")
            ax2.set_xlabel("Timestamp")
            ax2.set_ylabel("Pulse Value")
            ax2.legend()

            heartbeat_values = np.append(heartbeat_values, heartbeat)
            pulse_time = np.append(pulse_time, rec_p_time[-1])
                
            #Plot 3: FFT
            if pulse_values.size == window_size:
                if fft_start < 0:
                    fft_start = round(0.04 * window_size)
                    fft_end = round(0.18 * window_size)
                
                fft_x_axis = (np.fft.fftfreq(window_size, np.mean(np.diff(rec_p_time))/1000) * 60)
                gaussian_fit_frequency()

                ax3.clear()
                ax3.plot(fft_x_axis[fft_start:fft_end], fft_pulse_values[fft_start:fft_end], color="green")
                ax3.plot(fft_x_axis[detected_fft_peaks], fft_pulse_values[detected_fft_peaks], 'x', color="red")
                ax3.set_title("FFT")
                ax3.set_xlabel("Timestamp")
                ax3.set_ylabel("Presence")
                ax3.legend()
            
            # Plot 4: gyro data
            ax5.clear()
            #ax5.plot(rec_g_time, gyro_x, label="Gyro X", color="red")
            #ax5.plot(rec_g_time, gyro_y, label="Gyro Y", color="green")
            #ax5.plot(rec_g_time, gyro_z, label="Gyro Z", color="purple")
            ax5.plot(rec_g_time, gyro_sum, color="black")
            #ax5.plot(rec_g_time, g_mean_x, label="mean", color = "black")
            ax5.set_title("Movement Data (Gyroscope)")
            ax5.set_xlabel("Timestamp")
            ax5.set_ylabel("Gyro")
            #ymin = min(min(gyro_x, default=0), min(gyro_y, default=0), min(gyro_z, default=0))
            #ymax = max(max(gyro_x, default=0), max(gyro_y, default=0), max(gyro_z, default=0), 0.3)
            #margin = abs(ymin - ymax) * 0.05
            #ax5.set_ylim(ymin-margin, ymax+margin)  # Dynamische Y-Skalierung
            ax5.legend()

            ax6.clear()
            ax6.plot(gyro_time, g_integrals, color="blue")
            ax6.set_title("Gyro Integral")
            ax6.set_xlabel("Timestamp")
            ax6.set_ylabel("Gyro")
            ax6.legend()

            ax7.clear()
            ax7.plot(pulse_time/60000, heartbeat_values, color="red")
            ax7.set_title("Heartbeat")
            ax7.set_xlabel("Timestamp")
            ax7.set_ylabel("Heartbeat")
            ax7.set_ylim(50, 110)
            ax7.legend()
            
    # Setup für matplotlib
    fig, (ax2, ax3,  ax5, ax6, ax7) = plt.subplots(5, 1, figsize=(15, 30), layout='constrained', gridspec_kw={'height_ratios': [2, 1, 1, 1.5, 2]})
    ani = FuncAnimation(fig, update, interval=1)
    #plt.subplots_adjust(hspace=1)
    #plt.tight_layout()
    plt.show()

##########################################################
# MQTT Server-Client-Handling
def on_connect(client, userdata, flags, rc):
    global flag_connected
    flag_connected = 1
    client_subscriptions(client)
    print("Connected to MQTT server")


def on_disconnect(client, userdata, rc):
    global flag_connected
    flag_connected = 0
    print("Disconnected from MQTT server")


def callback_esp32_sensor1(client, userdata, msg):
    try:
        timestamp, value = map(int, msg.payload.decode('utf-8').split(','))
        pulse_data_queue.put({
            "timestamp": np.array([timestamp]), 
            "value": np.array([value], dtype=np.float64)
            })
    except ValueError as e:
        print(f"Error parsing pulse data: {e}")


def callback_esp32_sensor2(client, userdata, msg):
    try:
        parts = msg.payload.decode('utf-8').split(',')
        data = {
            "timestamp": np.array([int(parts[0])]),
            "acc_x": np.array([float(parts[1])]),
            "acc_y": np.array([float(parts[2])]),
            "acc_z": np.array([float(parts[3])]),
            "gyro_x": np.array([float(parts[4])]),
            "gyro_y": np.array([float(parts[5])]),
            "gyro_z": np.array([float(parts[6])])
        }
        movement_data_queue.put(data)
    except (ValueError, IndexError) as e:
        print(f"Error parsing movement data: {e}")


def client_subscriptions(client):
    client.subscribe("esp32/#")
    client.subscribe("rpi/broadcast")

# Hauptskript
display_thread = Thread(target=display_data_live, daemon=True)
display_thread.start()

client = mqtt.Client("rpi_client1", protocol=mqtt.MQTTv311)
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.message_callback_add('esp32/pulse', callback_esp32_sensor1)
client.message_callback_add('esp32/movement', callback_esp32_sensor2)
client.connect('127.0.0.1', 1883)
client.loop_start()

try:
    while True:
        time.sleep(4)
        if flag_connected != 1:
            print("Trying to reconnect to MQTT server...")
except KeyboardInterrupt:
    print("Shutting down...")
    client.loop_stop()
    client.disconnect()
    print("Stopped.")
