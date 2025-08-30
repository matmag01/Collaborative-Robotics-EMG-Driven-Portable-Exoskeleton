import time
import socket
from scipy import signal
from scipy.signal import butter, filtfilt
import numpy as np
import matplotlib.pyplot as plt
from TMSiSDK.device import ChannelType
from TMSiSDK.tmsi_sdk import TMSiSDK, DeviceType, DeviceInterfaceType
from TMSiSDK.tmsi_errors.error import TMSiError
from TMSiSDK.device.devices.saga.saga_API_enums import SagaBaseSampleRate
from TMSiSDK.device.devices.saga.saga_API import TMSiGetDeviceData
from TMSiSDK.device.tmsi_device_enums import MeasurementType
from TMSiProcessing.filters.real_time_filter import RealTimeFilter

import serial
import time


#Functions for signal processing

def butter_bandpass(lowcut, highcut, fs, val, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    [b, a] = butter(order, [low, high], btype='band')
    emg_filtered = filtfilt(b, a, val)
    return emg_filtered


def lowpass(lowcut, fs, val, order=5):
    nyquist = 0.5 * fs  
    low = lowcut / nyquist
    [b, a] = signal.butter(order, low, btype='low')

    emg_filtered = signal.filtfilt(b, a, val)
    return emg_filtered

# Wi-fi connection: 

ip = '192.168.4.1' 
port = 80          

conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
conn.settimeout(5)
conn.connect((ip, port))

# Connection to TMSi device

try:
    # Initialize TMSiSDK and discover devices
    sdk = TMSiSDK()
    print('Looking for TMSi devices...')
    sdk.discover(dev_type=DeviceType.saga,
                 dr_interface=DeviceInterfaceType.docked, ds_interface=DeviceInterfaceType.usb)
    discoveryList = sdk.get_device_list(DeviceType.saga)

    if len(discoveryList) > 0:
        print('Opening SAGA device...')
        # Open the first SAGA device found
        for i, _ in enumerate(discoveryList):
            dev = discoveryList[i]
            if dev.get_dr_interface() == DeviceInterfaceType.docked:
                dev.open()
                break

        print('Configuring channels...')
        # Setting sample rate at 500 Hz
        dev.set_device_sampling_config(base_sample_rate=SagaBaseSampleRate.Decimal,
                                       channel_type=ChannelType.BIP, channel_divider=8
                                       )
        dev.set_device_sampling_config(channel_type=ChannelType.AUX,
                                       channel_divider=8
                                       )

        # Set the channels to acquire
        AUX_list = []  
        BIP_list = [0]  

        ch_list = dev.get_device_channels()
        AUX_count, BIP_count = 0, 0
        enable_channels = []

        for idx, ch in enumerate(ch_list):
            if ch.get_channel_type() == ChannelType.AUX and AUX_count in AUX_list:
                enable_channels.append(idx)
                AUX_count += 1
            elif ch.get_channel_type() == ChannelType.BIP and BIP_count in BIP_list:
                enable_channels.append(idx)
                BIP_count += 1

        dev.set_device_active_channels(enable_channels, True)
        # Start sampling
        dev.start_measurement(MeasurementType.SAGA_SIGNAL)

        # Retrieve and process data in real-time
        print("Reading data in real-time. Press Ctrl+C to stop.")

    # Create the real time filter
    filter = RealTimeFilter(dev)
    filter.start()

    emg = []
    is_baseline_processing_done = False
    trigger_active = False

    #start processing of signal's buffers

    while filter.filter_thread.sampling:
        if not filter.filter_thread.q_filtered_sample_sets.empty():
            data = filter.filter_thread.q_filtered_sample_sets.get()[0]
            emg.extend(data)

            if len(emg) > 5000 and not is_baseline_processing_done: # baseline identification
                print('In baseline processing')
                baseline = np.array(emg)
                
                #baseline processing:
                emg_filtered = butter_bandpass(20, 249, 500,
                                               baseline.T,
                                               5)
                emg_rectified = np.abs(emg_filtered)

                emg_moving_avg_200ms = np.convolve(emg_rectified.flatten(), np.ones(100) / 100, mode='same')
                emg_moving_avg_200ms = emg_moving_avg_200ms.reshape(-1, 1)
                emg_envelope = lowpass(10, 500, emg_moving_avg_200ms.T, 5)
                emg_envelope = emg_envelope.reshape(-1, 1)


                #threshold definition:
                th = np.mean(emg_envelope) + 7 * np.std(emg_envelope)
                th2= np.mean(emg_envelope) + 4 * np.std(emg_envelope)
                print(th)
                is_baseline_processing_done = True

                emg=[]

            if is_baseline_processing_done:

                if len(emg) >= 100: #identification of buffers made of 100 samples

                    sample = np.array(emg)
                    emg=[]

                    #signal processing:
                    emg_filtered = butter_bandpass(20, 249, 500,
                                                sample.T,
                                                5)
                    emg_rectified = np.abs(emg_filtered)

                    emg_moving_avg_200ms = np.convolve(emg_rectified.flatten(), np.ones(20) / 20, mode='same')
                    emg_moving_avg_200ms = emg_moving_avg_200ms.reshape(-1, 1)
                    emg_envelope = lowpass(10, 500, emg_moving_avg_200ms.T, 5)
                    emg_envelope = emg_envelope.reshape(-1, 1)

                    if np.all(emg_envelope>th)and not trigger_active: #checking if buffer is over the onset threshold 
                        trigger=1
                        conn.send(str(trigger).encode()) #sending the trigger to Arduino via Wi-fi
                        trigger_active = True
                        time.sleep(0.5)  # [s]
                        try:
                            from_server = conn.recv(1024)  # [bytes]
                            print(from_server.decode())
                        except socket.timeout:
                            print("No response received (timeout)")
                        trigger=0

                    if np.all(emg_envelope < th2) and trigger_active: #checking if buffer is under the offset threshold, meaning that the contraction is ended
                        trigger_active = False #flag that permits to start finding another contraction


except KeyboardInterrupt:
    filter.stop()