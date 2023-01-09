
#Leonel Gouveia Ergin & Valentin Sokolow: Octobre 2019 - Janvier 2020

###### IMPORTS ######
import os
import numpy as np
import scipy.interpolate as spi
import scipy.fftpack as spf
from extrafunctions import butter_bandpass_filter as bbf
from extrafunctions import butter_bandpass as bb
import matplotlib.pyplot as plt
from scipy.signal import freqz
import csv
from termcolor import colored
from colorama import init

import os

os.chdir('/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/resultsR')

###### INFORMATION & OPTIONS ######
init()

resampling= 0

displayplt2 = (input('Display Primary plots?[y for yes,n for no]')=='y')
displayplt1 = (input('Display Secondary plots?[y for yes,n for no]')=='y')
print('Resampling is ON') if resampling else print('Resampling is OFF')
savefig = False


##### READ NORM POND ####
with open('pond2631.csv', 'r') as file:
    lines = file.readlines()
    rawdata = []
    for line in lines:
        rawdata.append(line.split(','))

nprawdata = np.array(rawdata)
data = nprawdata[1:,:].astype(np.float64)
pondfreqs = data[:,0]
Wk = data[:,1]/1000.0
Wd = data[:,2]/1000.0



if(displayplt1):
    fig1, plt1 = plt.subplots(1,2)
    fig1.suptitle('Basic Information', fontsize=16)
    fig1.tight_layout()
    plt1[0].plot(pondfreqs,Wk,'-',pondfreqs,Wd,'-')
    plt1[0].set_xscale('log')
    plt1[0].set_yscale('log')
    plt1[0].grid(True)
    plt1[0].set_xlabel('Frequency [Hz]')
    plt1[0].set_ylabel('Coefficients [-]')
    plt1[0].set_title('Frequency domain weigths')
    plt1[0].legend(['Wk', 'Wd'], loc='best')
    xmin, xmax = plt1[1].get_xlim()
    ymin, ymax = plt1[1].get_ylim()

    ratio = 1
    plt1[1].set_aspect(ratio)


###### READ FILE & EXTRACT DATA  ######

qdd = np.loadtxt('Bumpy_10m_s_Simple_contact_qdd.res')

# vert_acc = np.loadtxt('Belgian_road_15m_s_qdd.res')[:, 3]
# hor_acc = np.loadtxt('Belgian_road_15m_s_Horizontal_Acc.res')
# lat_acc = np.loadtxt('Belgian_road_15m_s_Lateral_Acc.res')

x = qdd[:, 1]
y = qdd[:, 2]
z = qdd[:, 3]

time = qdd[:, 0]


###### TIME VECTOR ANALYSIS ######

timedelta = np.empty(len(time)-1)
for index, t in enumerate(time[:-1]):
    timedelta[index] = time[index+1]-time[index]

freqmean = 1/np.mean(timedelta)
timevar = np.var(time)


fs = np.floor(freqmean)
print(f'Frequency Mean: {freqmean:.2f} Hz\nTime Variance:  {timevar:.2f}')
print(f'Resampling frequency : {fs} Hz'+ (colored('\n/!\\ Caution! Sampling frequency is too low. Please use a better device','red') if fs<201 else ''))
print(f'Measurement duration : {time[-1]} s')



###### UNIFORM RESAMPLING #######
resampfx = spi.interp1d(time, x)
resampfy = spi.interp1d(time, y)
resampfz = spi.interp1d(time, z)
resamptime = np.arange(0 , time[-1] , 1/fs)
time = time[0:len(resamptime)]
x = x[0:len(resamptime)]
y = y[0:len(resamptime)]
z = z[0:len(resamptime)]
resampx = resampfx(resamptime)
resampy = resampfy(resamptime)
resampz = resampfz(resamptime)



workx = x if resampling==False else resampx
worky = y if resampling==False else resampy
workz = z if resampling==False else resampz

if False:

    zffterror = (np.abs((np.fft.fft(z))[0:z.size//2]) - np.abs((np.fft.fft(resampz))[0:resampz.size//2]))
    freq = np.linspace(0.0, fs/(2.0), time.size//2)
    plt.plot(freq,(np.abs((np.fft.fft(z))[0:z.size//2])),'-',freq,np.abs((np.fft.fft(resampz))[0:resampz.size//2]),freq,zffterror,'-')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Amplitude [-]')
    plt.title('Resampling error for Device 3')
    #plt.plot(freq,zffterror,'-')
    plt.xscale('log')
    plt.yscale('log')
    plt.legend(['Original signal', 'Resampled signal','Error'], loc='best')
    plt.savefig('device3error.eps', format='eps')
    plt.show()


RMS = np.sqrt(np.mean(x**2+y**2+z**2))
print(f'RMS value of signal: {RMS:.2f} m/s^2')

###### FILTER ######

# desired cutoff frequencies (in Hz) (according to the norms, they should be 1/3 octave above and below our desired frequency study resp. 0.5 and 80.0)
lowcut = 0.4
highcut = fs/2-1 if fs<201 else 100.0
xfiltered = bbf(workx, lowcut, highcut, fs, order=2)
yfiltered = bbf(worky, lowcut, highcut, fs, order=2)
zfiltered = bbf(workz, lowcut, highcut, fs, order=2)
RMS = np.sqrt(np.mean(xfiltered**2+yfiltered**2+zfiltered**2))
print(f'RMS value of filtered signal: {RMS:.2f} m/s^2')


if(displayplt1):

    b, a = bb(lowcut, highcut, fs, order=2)
    w, h = freqz(b, a, worN=1000)
    plt1[1].plot((fs * 0.5 / np.pi) * w, abs(h))
    plt1[1].set_xscale('log')
    plt1[1].set_yscale('log')
    plt1[1].grid(True)
    plt1[1].set_xlabel('Frequency [Hz]')
    plt1[1].set_ylabel('Amplitude [-]')
    plt1[1].set_title('Band-pass filter')
    xmin, xmax = plt1[1].get_xlim()
    ymin, ymax = plt1[1].get_ylim()
    #print(np.log(xmax-xmin)/np.log(ymax-ymin))
    ratio = 1
    plt1[1].set_aspect(0.52)
    mng = plt.get_current_fig_manager()
    #mng.resize(*mng.window.maxsize())
    #mng.window.state('zoomed') #works fine on Windows!
    plt.show(block=False)


###### SPECTRAL ANALYSIS ######

xfft = ((np.fft.fft(xfiltered))[0:xfiltered.size//2])
yfft = ((np.fft.fft(yfiltered))[0:yfiltered.size//2])
zfft = ((np.fft.fft(zfiltered))[0:zfiltered.size//2])

freq = np.linspace(0.0, fs/(2.0), time.size//2)


###### APPLY COEFFICIENTS #######

wkpondfunc = spi.interp1d(pondfreqs, Wk)
wkpondcoeffs = wkpondfunc(freq)
wdpondfunc = spi.interp1d(pondfreqs, Wd)
wdpondcoeffs = wdpondfunc(freq)


print('\n\nAnalysing Comfort')
xpondfft = wdpondcoeffs*xfft
ypondfft = wdpondcoeffs*yfft
zpondfft = wkpondcoeffs*zfft


xpond = np.real(np.fft.ifft(np.concatenate((xpondfft,np.flip(xpondfft)),axis=None)[0:-1]))
ypond = np.real(np.fft.ifft(np.concatenate((ypondfft,np.flip(ypondfft)),axis=None)[0:-1]))
zpond = np.real(np.fft.ifft(np.concatenate((zpondfft,np.flip(zpondfft)),axis=None)[0:-1]))

totvib = np.sqrt(xpond**2 + ypond**2 + zpond**2)

RMStotvib =np.sqrt(np.mean(totvib**2))

VDV = np.power(np.mean(totvib**4 ), 0.25)
methodcoeff = RMStotvib/VDV/np.power(time[-1] , 0.25)
print(methodcoeff)

if(methodcoeff > 1.75):
    print('The basic method cannot be used, see ISO2631-1:1997[F] Section C.2.2.3 for more information')
    exit()
else:

    if(RMStotvib<0.315):
         message = 'not at all uncomfortable (5/5)'
         color= 'green'
    elif(RMStotvib<0.63):
        message = 'slightly uncomfortable (4/5)'
        color= 'green'
    elif(RMStotvib<1):
        message = 'quite uncomfortable (3/5)'
        color= 'yellow'
    elif(RMStotvib<1.6):
        message =  'uncomfortable (2/5)'
        color= 'yellow'
    elif(RMStotvib<2.5):
        message = 'very uncomfortable (1/5)'
        color= 'red'
    elif(RMStotvib>2):
        message = 'extremely uncomfortable (0/5)'
        color= 'red'

    print(f'RMS acceleration: {RMStotvib:.04f} m/s^2')
    print('This vehicle is '+ colored(message,color) +' according to the ISO2631 comfort norm (Section C.2.3)')

if(displayplt2):
    fig2, plt2 = plt.subplots(3,3)
    fig2.tight_layout()
    fig2.suptitle('Comfort Analysis',x=0.05,y=0.995,fontsize=16)
    plt2[0,0].plot(time, x-np.mean(x), '-', resamptime, resampx-np.mean(resampx), '-',resamptime, xfiltered, '-')
    plt2[0,0].legend(['data (centered)', 'resampled(centered)','filtered'], loc='best')
    start = time[-1]/5
    #plt2[0,0].set_xlim(start,start+0.2)
    plt2[0,0].set_xlabel('Time [s]')
    plt2[0,0].set_ylabel('Amplitude [-]')
    plt2[0,0].set_title('X Signal')


    plt2[0,1].plot(time, y-np.mean(y), '-', resamptime, resampy-np.mean(resampy), '-',resamptime, yfiltered, '-')
    plt2[0,1].legend(['data (centered)', 'resampled(centered)','filtered'], loc='best')
    start = time[-1]/5
    #plt2[0,1].set_xlim(start,start+0.2)
    plt2[0,1].set_xlabel('Time [s]')
    plt2[0,1].set_ylabel('Amplitude [-]')
    plt2[0,1].set_title('Y Signal')


    plt2[0,2].plot(time, z-np.mean(z), '-', resamptime, resampz-np.mean(resampz), '-',resamptime, zfiltered, '-')
    plt2[0,2].legend(['data (centered)', 'resampled(centered)','filtered'], loc='best')
    start = time[-1]/5
    #plt2[0,2].set_xlim(start,start+0.2)
    plt2[0,2].set_xlabel('Time [s]')
    plt2[0,2].set_ylabel('Amplitude [-]')
    plt2[0,2].set_title('Z Signal')

    w=40
    def moving_average(x, w=40):
        return np.convolve(x, np.ones(w), 'same') / w

    xfft = ((np.fft.fft(workx))[0:xfiltered.size//2])
    yfft = ((np.fft.fft(worky))[0:yfiltered.size//2])
    zfft = ((np.fft.fft(workz))[0:zfiltered.size//2])
    freq = np.linspace(0.0, fs/(2.0), time.size//2)

    plt2[1,0].plot(freq,abs(xfft),freq[w:],moving_average(abs(xfft))[w:])
    plt2[1,0].set_xscale('log')
    plt2[1,0].set_yscale('log')
    plt2[1,0].legend(['signal', f'running average (n={w})'], loc='best')
    plt2[1,0].set_xlabel('Frequency [Hz]')
    plt2[1,0].set_ylabel('Amplitude [-]')
    plt2[1,0].set_title('FFT of X Signal')

    plt2[1,1].plot(freq,abs(yfft),freq[w:],moving_average(abs(yfft))[w:])
    plt2[1,1].set_xscale('log')
    plt2[1,1].set_yscale('log')
    plt2[1,1].legend(['signal', f'running average (n={w})'], loc='best')
    plt2[1,1].set_xlabel('Frequency [Hz]')
    plt2[1,1].set_ylabel('Amplitude [-]')
    plt2[1,1].set_title('FFT of Y Signal')

    plt2[1,2].plot(freq,abs(zfft),freq[w:],moving_average(abs(zfft))[w:])
    plt2[1,2].set_xscale('log')
    plt2[1,2].set_yscale('log')
    plt2[1,2].legend(['signal', f'running average (n={w})'], loc='best')
    plt2[1,2].set_xlabel('Frequency [Hz]')
    plt2[1,2].set_ylabel('Amplitude [-]')
    plt2[1,2].set_title('FFT of Z Signal')


    plt2[2,0].plot(freq,abs(xpondfft),freq[w:],moving_average(abs(xpondfft))[w:])
    plt2[2,0].set_xscale('log')
    plt2[2,0].set_yscale('log')
    plt2[2,0].legend(['filtered and weighted signal', f'running average (n={w})'], loc='best')
    plt2[2,0].set_xlabel('Frequency [Hz]')
    plt2[2,0].set_ylabel('Amplitude [-]')

    plt2[2,1].plot(freq,abs(ypondfft),freq[w:],moving_average(abs(ypondfft))[w:])
    plt2[2,1].set_xscale('log')
    plt2[2,1].set_yscale('log')
    plt2[2,1].legend(['filtered and weighted signal', f'running average (n={w})'], loc='best')
    plt2[2,1].set_xlabel('Frequency [Hz]')
    plt2[2,1].set_ylabel('Amplitude [-]')

    plt2[2,2].plot(freq,abs(zpondfft),freq[w:],moving_average(abs(zpondfft))[w:])
    plt2[2,2].set_xscale('log')
    plt2[2,2].set_yscale('log')
    plt2[2,2].legend(['filtered and weighted signal', f'running average (n={w})'], loc='best')
    plt2[2,2].set_xlabel('Frequency [Hz]')
    plt2[2,2].set_ylabel('Amplitude [-]')
    mng = plt.get_current_fig_manager()
    #mng.window.state('zoomed') #works fine on Windows!
    plt.show(block=False)

print('\n\nAnalysing health')
xpondfft = wdpondcoeffs*xfft*1.4
ypondfft = wdpondcoeffs*yfft*1.4
zpondfft = wkpondcoeffs*zfft

xpond = np.real(np.fft.ifft(np.concatenate((xpondfft,np.flip(xpondfft)),axis=None)[0:-1]))
ypond = np.real(np.fft.ifft(np.concatenate((ypondfft,np.flip(ypondfft)),axis=None)[0:-1]))
zpond = np.real(np.fft.ifft(np.concatenate((zpondfft,np.flip(zpondfft)),axis=None)[0:-1]))

totvib = np.sqrt(xpond**2 + ypond**2 + zpond**2)

RMStotvib =np.sqrt(np.mean(totvib**2))

VDV = np.power(np.mean(totvib**4 ), 0.25)
methodcoeff = RMStotvib/VDV/np.power(time[-1] , 0.25)

if(methodcoeff > 1.75):
    print('The basic method cannot be used, see ISO2631-1:1997[F] Section C.2.2.3 for more information')
    exit()
else:
    if(RMStotvib<0.25):
        message = 'healthy for up to 24h of exposure'
        color = 'green'
    elif(RMStotvib<0.4):
        message = 'healthy for up to 10h of exposure'
        color = 'green'
    elif(RMStotvib<0.63):
        message= 'healthy for up to 5h of exposure'
        color = 'yellow'
    elif(RMStotvib<1):
        message   = 'healthy for up to 2h of exposure'
        color = 'yellow'
    elif(RMStotvib<1.6):
        message = 'healthy for up to 40 min of exposure'
        color = 'magenta'
    elif(RMStotvib<2.5):
        message = 'healthy for up to 20 min of exposure'
        color = 'magenta'
    elif(RMStotvib<3):
        message  = 'healthy for up to 10 min of exposure'
        color = 'red'
    elif(RMStotvib<6):
        message = 'Unhealthy for 10 min of exposure'
        color = 'red'
    else:
        message = 'Unhealthy for any amount of time'
        color = 'red'

    print(f'RMS acceleration: {RMStotvib:.04f} m/s^2')
    print('These vibrations are '+colored(message,color) + ' according to the ISO2631 health norm (Section B.3.1)')

if(displayplt2):
    fig2, plt2 = plt.subplots(3,3)
    fig2.tight_layout()
    fig2.suptitle('Health Analysis',x=0.05,y=0.995, fontsize=16)
    plt2[0,0].plot(time, x-np.mean(x), '-', resamptime, resampx-np.mean(resampx), '-',resamptime, xfiltered, '-')
    plt2[0,0].legend(['data (centered)', 'resampled(centered)','filtered'], loc='best')
    start = time[-1]/5
    #plt2[0,0].set_xlim(start,start+0.2)
    plt2[0,0].set_xlabel('Time [s]')
    plt2[0,0].set_ylabel('Amplitude [-]')
    plt2[0,0].set_title('X Signal')


    plt2[0,1].plot(time, y-np.mean(y), '-', resamptime, resampy-np.mean(resampy), '-',resamptime, yfiltered, '-')
    plt2[0,1].legend(['data (centered)', 'resampled(centered)','filtered'], loc='best')
    start = time[-1]/5
    #plt2[0,1].set_xlim(start,start+0.2)
    plt2[0,1].set_xlabel('Time [s]')
    plt2[0,1].set_ylabel('Amplitude [-]')
    plt2[0,1].set_title('Y Signal')


    plt2[0,2].plot(time, z-np.mean(z), '-', resamptime, resampz-np.mean(resampz), '-',resamptime, zfiltered, '-')
    plt2[0,2].legend(['data (centered)', 'resampled(centered)','filtered'], loc='best')
    start = time[-1]/5
    #plt2[0,2].set_xlim(start,start+0.2)
    plt2[0,2].set_xlabel('Time [s]')
    plt2[0,2].set_ylabel('Amplitude [-]')
    plt2[0,2].set_title('Z Signal')

    w=40
    def moving_average(x, w=40):
        return np.convolve(x, np.ones(w), 'same') / w

    xfft = ((np.fft.fft(workx))[0:xfiltered.size//2])
    yfft = ((np.fft.fft(worky))[0:yfiltered.size//2])
    zfft = ((np.fft.fft(workz))[0:zfiltered.size//2])
    freq = np.linspace(0.0, fs/(2.0), time.size//2)

    plt2[1,0].plot(freq,abs(xfft),freq[w:],moving_average(abs(xfft))[w:])
    plt2[1,0].set_xscale('log')
    plt2[1,0].set_yscale('log')
    plt2[1,0].legend(['signal', f'running average (n={w})'], loc='best')
    plt2[1,0].set_xlabel('Frequency [Hz]')
    plt2[1,0].set_ylabel('Amplitude [-]')
    plt2[1,0].set_title('FFT of X Signal')

    plt2[1,1].plot(freq,abs(yfft),freq[w:],moving_average(abs(yfft))[w:])
    plt2[1,1].set_xscale('log')
    plt2[1,1].set_yscale('log')
    plt2[1,1].legend(['signal', f'running average (n={w})'], loc='best')
    plt2[1,1].set_xlabel('Frequency [Hz]')
    plt2[1,1].set_ylabel('Amplitude [-]')
    plt2[1,1].set_title('FFT of Y Signal')

    plt2[1,2].plot(freq,abs(zfft),freq[w:],moving_average(abs(zfft))[w:])
    plt2[1,2].set_xscale('log')
    plt2[1,2].set_yscale('log')
    plt2[1,2].legend(['signal', f'running average (n={w})'], loc='best')
    plt2[1,2].set_xlabel('Frequency [Hz]')
    plt2[1,2].set_ylabel('Amplitude [-]')
    plt2[1,2].set_title('FFT of Z Signal')


    plt2[2,0].plot(freq,abs(xpondfft),freq[w:],moving_average(abs(xpondfft))[w:])
    plt2[2,0].set_xscale('log')
    plt2[2,0].set_yscale('log')
    plt2[2,0].legend(['filtered and weighted signal', f'running average (n={w})'], loc='best')
    plt2[2,0].set_xlabel('Frequency [Hz]')
    plt2[2,0].set_ylabel('Amplitude [-]')

    plt2[2,1].plot(freq,abs(ypondfft),freq[w:],moving_average(abs(ypondfft))[w:])
    plt2[2,1].set_xscale('log')
    plt2[2,1].set_yscale('log')
    plt2[2,1].legend(['filtered and weighted signal', f'running average (n={w})'], loc='best')
    plt2[2,1].set_xlabel('Frequency [Hz]')
    plt2[2,1].set_ylabel('Amplitude [-]')

    plt2[2,2].plot(freq,abs(zpondfft),freq[w:],moving_average(abs(zpondfft))[w:])
    plt2[2,2].set_xscale('log')
    plt2[2,2].set_yscale('log')
    plt2[2,2].legend(['filtered and weighted signal', f'running average (n={w})'], loc='best')
    plt2[2,2].set_xlabel('Frequency [Hz]')
    plt2[2,2].set_ylabel('Amplitude [-]')
    mng = plt.get_current_fig_manager()
    mng.window.state('zoomed') #works fine on Windows!
    plt.show(block=False)

if(displayplt1|displayplt2):
    plt.show()
    
input('')