from __future__ import division

import numpy
import scipy
import scipy.signal
import math 

# Note, after writing this and comparing against octave,
# it seems to line up, but seems a lot suckier compared to
# whatever MATLAB is doing.
# def resample(x, p, q):
#     """Polyphase filter resample, based on octave signal's resample.m"""
#     log10_rejection = -3
#     stopband_cutoff_f = 1/(2 * max(p,q))
#     roll_off_width = stopband_cutoff_f / 10

#     rejection_dB = -20*log10_rejection
#     L = math.ceil((rejection_dB-8.0) / (28.714 * roll_off_width))
    
#     t = numpy.arange(-L, L+1)
#     ideal_filter = 2*p*stopband_cutoff_f*numpy.sinc(2*stopband_cutoff_f*t)  

#     if rejection_dB >= 21 and rejection_dB <= 50:
#       beta = 0.5842 * (rejection_dB-21.0)**0.4 + 0.07886 * (rejection_dB-21.0)
#     elif rejection_dB > 50:
#       beta = 0.1102 * (rejection_dB-8.7)
#     else:
#       beta = 0

#     h = numpy.kaiser(2*L+1,beta) * ideal_filter

#     nz_pre = math.floor(q-L%q);
#     hpad = prepad(h,h.shape[0] + nz_pre);
#     Ly = math.ceil(x.shape[0]*p/q)
    
#     offset = math.floor((L+nz_pre)/q);
#     nz_post = 0;
#     while math.ceil(((x.shape[0]-1)*p + nz_pre + h.shape[0] + nz_post)/q) - offset < Ly:
#         nz_post += 1
#     hpad = postpad(hpad,h.shape[0] + nz_pre + nz_post);

#     xfilt = upfirdn(x, hpad, p, q)
#     return xfilt[offset:offset+Ly]

def resample(x, p, q):
    bta = 5
    N = 10
    pqmax = max(p, q)
    fc = (1/2)/pqmax
    L = 2*N*pqmax + 1
    h = p*scipy.signal.firwin(L-1, 2*fc, window=('kaiser', bta))
    Lhalf = (L-1)/2
    Lx = len(x)

    nz = math.floor(q-(Lhalf % q))
    z = numpy.zeros(nz)
    Lhalf += nz

    delay = math.floor(math.ceil(Lhalf)/q)
    nz1 = 0
    while math.ceil(((Lx - 1)*p + len(h) + nz1)/q) - delay < math.ceil(Lx*p/q):
        nz1 = nz1+1;
    h = numpy.hstack([h, numpy.zeros(nz1)]);
    y = upfirdn(x,h,p,q)
    Ly = math.ceil(Lx*p/q)
    y = y[delay:]
    y = y[:Ly]
    return y

def upfirdn(x, h, p, q):
    # Allocate an expanded array to hold x interspersed with p-1 zeros,
    # padded with enough zeros for the fir filter
    x_expanded = numpy.zeros((x.shape[0] - 1)*p + h.shape[0])

    # Insert x values every p elements
    x_expanded[:x.shape[0]*p:p] = x

    # Run filter
    x_filt = scipy.signal.lfilter(h, 1, x_expanded)
    return x_filt

def prepad(x, l, c=0):
    pad = max(l - x.shape[0], 0)
    if pad == 0:
        return x.copy()
    x_pad = numpy.empty(l)
    x_pad[:pad] = c
    x_pad[pad:] = x[:l-pad]
    return x_pad

def postpad(x, l, c=0):
    pad = max(l - x.shape[0], 0)
    if pad == 0:
        return x.copy()
    x_pad = numpy.empty(l)
    x_pad[:-pad] = x[:l-pad]
    x_pad[-pad:] = c
    return x_pad

def make_ping_channel(delay=0, 
                      freq=25e3, 
                      ampl=.5, 
                      zeros=64, 
                      count=1024,
                      sample_rate=300e3):
    w = 2*math.pi*freq/sample_rate
    sinwave = ampl*numpy.sin(w*(numpy.arange(count)-delay))

    delay_i = round(delay)
    window = numpy.zeros(count)
    window[zeros+delay_i:] = numpy.minimum(numpy.exp(numpy.arange(count - zeros - delay_i)/10), 2)-1

    noise = numpy.random.normal(0, .01, count)

    return sinwave * window + noise

def make_ping(delays=[0, 0, 0, 0], args={}):
    return numpy.vstack(make_ping_channel(delay=delay, **args) for delay in delays)
