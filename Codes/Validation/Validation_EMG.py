

import sys
import os
import scipy
from scipy import signal
from scipy.signal import butter, filtfilt
import time
import socket
import csv
import time
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog


#functions for processing the signal

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

#find and open CSV file

Example_dir = os.path.dirname(os.path.realpath(__file__))  
modules_dir = os.path.join(Example_dir, '../..')  
measurements_dir = os.path.join(Example_dir, '../measurements')  
sys.path.append(modules_dir)

class CSVReader:
    def __init__(self, filename=None):
        if filename == None:
            root = tk.Tk()

            filename = filedialog.askopenfilename(title='Select csv-file', filetypes=('csv-files', '*.csv'))
            root.withdraw()

        self.filename = filename
        print('Reading file ', filename)
        self._readFile(filename)

    def _readFile(self, filename):
        # Open the csv file to read its
        with open(filename, 'r', newline='', encoding='utf16') as csvfile:
            reader = csv.reader(csvfile, delimiter=';')
            samples = []
            # Loop through the csv file
            for i, row in enumerate(reader):
                if i == 0:
                    self.ch_names = row[:-1]
                    self.num_channels = len(self.ch_names)
                    continue  # move to next iteration
                if i == 1:
                    # The last value for each row is the sampling frequency.
                    # Assigned only at the first valid line to avoid redundancy.
                    self.sample_rate = float(row[-1].replace(',', '.'))
                samples.append([float(val.replace(',', '.'))
                                for val in row[:-1]
                                ])
        self.samples = np.array(samples)
        self.num_samples = i



if __name__ == "__main__":
    # Open a dialog to let the user pick the file to read
    root = tk.Tk()
    filename = filedialog.askopenfilename(title='Select file',
                                            filetypes=(('All files', '*.*'),
                                                        ('CSV files', '*.csv'),
                                                        ('XDF files', '*.xdf'),
                                                        ('Poly5 files', '*.poly5')
                                                        ))
    root.withdraw()  # close the dialog window

    # Get the file format and read data accordingly
    fmt = os.path.splitext(filename)[-1].lstrip('.')
    if filename.endswith('.csv'):
        data = CSVReader(filename=filename)

        # Data ectraction from csv file
        samples = data.samples
        num_samples = data.num_samples
        ch_names = data.ch_names
        num_channels = data.num_channels
        f = data.sample_rate
        emg_signal = samples[:, 0]
        EMG_time = np.linspace(0, num_samples / f, num_samples)
    else:
        raise NotImplementedError(
            'File format {} not supported! Supported formats are CSV'
        )

   
    print('\tNo. of channels names: {}'.format(num_channels))
    print('\tChannel names: '.format(ch_names), end='\t')
    for ch in ch_names:
        print('{}\t|'.format(ch), end='\t')
    print()  
    print('\tFirst 10 samples: ')
    for sample in samples[:10]:
        print('\t\t\t', end='')
        for val in sample:
            print('{}\t|'.format(val), end='\t')
        print()  
    print('\tNo. of samples: {}'.format(num_samples))
    print('\tSampling frequency: {}Hz'.format(f))

    nsamp = 100

    # Get number of bipolar channels
    num_bipolar_channels = 0
    for name in ch_names:
        if name.startswith('BIP'):
            num_bipolar_channels += 1

    cnt=0
    mean=0

    data_processed = np.zeros((num_samples,
                                num_bipolar_channels)
                                )
    data_processed_baseline = np.zeros((5000,
                                num_bipolar_channels)
                                )

    # Initialize a buffer to accumulate samples
    sample_buffer = np.zeros((nsamp,
                                num_bipolar_channels)
                                )
 

    t_i = time.time()
    baseline=samples[0:5000]
    sig=samples[5001:]

    base_signal=samples[:, :num_bipolar_channels]  #samples of the signal

    #defining variables:
    window_size=100 #buffer lenght 
    emg = [] #list of raw samples
    complete_signal=[] #list of all processed samples
    is_baseline_processing_done = False #flag to define baseline processing
    trigger_active = False #flag for finding onsets

    t = np.linspace(0, len(base_signal)/500, len(base_signal))

    trigger_on_times = [] #list of trigger onset instants 
    trigger_off_times = [] #list of trigger offset instants



    for i in range(0,len(base_signal)): #real time simulation of data retrieving 
        data = base_signal[i]
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
                complete_signal.extend(emg_envelope) #to accumulate signal for final plot

                #threshold definition:
                th = np.mean(emg_envelope) + 9 * np.std(emg_envelope)
                th2= np.mean(emg_envelope) + 3 * np.std(emg_envelope)
                print(th)
                is_baseline_processing_done = True
                emg=[]

        if is_baseline_processing_done:

            if len(emg) > 100: #identification of buffers made of 100 samples

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
                complete_signal.extend(emg_envelope) #to accumulate signal for final plot

                if np.all(emg_envelope>th) and not trigger_active: #checking if buffer is over the onset threshold 
                    trigger=1
                    trigger_active = True
                    trigger=0
                    trigger_on_times.append(t[i])
                    print(f"Trigger inviato al tempo {t[i]:.2f}s!")



                if np.all(emg_envelope < th2) and trigger_active:  #checking if buffer is under the offset threshold, meaning that the contraction is ended
                    trigger_active = False ##flag that permits to start finding another contraction
                    trigger_off_times.append(t[i])
                    print(f"Trigger disattivato al tempo {t[i]:.2f}s!")



    time_processed = t[:len(complete_signal)]


    #plot of raw signal with threshold and triggers
    plt.figure(figsize=(12, 6))
    plt.plot(t, base_signal, label='Raw signal')

    
    for time in trigger_on_times:
        plt.axvline(x=time, color='green', linestyle='--', label='Trigger ON')

    for time in trigger_off_times:
        plt.axvline(x=time, color='purple', linestyle='--', label='Trigger OFF')

    legend_labels = []

    if trigger_on_times:
        for time in trigger_on_times:
            legend_labels.append(f'Trigger ON at {time:.2f}s')

    
    if trigger_off_times:
        for time in trigger_off_times:
            legend_labels.append(f'Trigger OFF at {time:.2f}s')

    plt.title('Raw EMG Signal')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.legend(legend_labels, loc='upper left', bbox_to_anchor=(0, 1)) 

    plt.show()


    #plot processed signal with threshold and triggers
    plt.figure(figsize=(12, 6))
    plt.plot(time_processed, complete_signal, label='Processed Signal', color='blue')

    plt.axhline(y=th, color='red', linestyle='--', label=f'Threshold th = {th:.2f}')
    plt.axhline(y=th2, color='orange', linestyle='--', label=f'Threshold th2 = {th2:.2f}')

    for time in trigger_on_times:
        plt.axvline(x=time, color='green', linestyle='--', label=f'Trigger ON at {time:.2f}s')

    for time in trigger_off_times:
        plt.axvline(x=time, color='purple', linestyle='--', label=f'Trigger OFF at {time:.2f}s')

    legend_labels = []

    legend_labels.append(f'Threshold th = {th:.2f}')
    legend_labels.append(f'Threshold th2 = {th2:.2f}')

    if trigger_on_times:
        for time in trigger_on_times:
            legend_labels.append(f'Trigger ON at {time:.2f}s')

    if trigger_off_times:
        for time in trigger_off_times:
            legend_labels.append(f'Trigger OFF at {time:.2f}s')


    plt.title('Processed EMG Signal with Thresholds and Triggers')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.legend(legend_labels, loc='upper left', bbox_to_anchor=(0, 1))  

    plt.show()




