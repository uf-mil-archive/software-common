from __future__ import division
from hydrophones import util

import numpy
import scipy
import math
import sys
import matplotlib.pyplot as plt

class AlgorithmError(Exception):
    pass

def preprocess(samples, sample_rate):
    """Preprocesses hydrophone data to be used with compute_deltas"""
    samples = samples[:, :round(0.001 * sample_rate)].copy()
    samples -= numpy.mean(samples[:, 0:32], 1)[:, numpy.newaxis]
    samples = bandpass(samples, sample_rate)
    samples /= numpy.amax(numpy.abs(samples), 1)[:, numpy.newaxis]
    upsamples, upsample_rate = upsample(samples, sample_rate, 3e6)
    return upsamples, upsample_rate

def bandpass(samples, sample_rate):
    """Applies a 20-30khz bandpass FIR filter"""
    fir = scipy.signal.firwin(128,
                              [19e3/(sample_rate/2), 31e3/(sample_rate/2)],
                              pass_zero=False)
    return scipy.signal.lfilter(fir, 1, samples)

def bin_to_freq(bin, sample_rate, fft_length):
    return (sample_rate/2) / (fft_length/2) * bin

def check_data(samples, sample_rate, plot=False):
    """Checks whether samples are likely a solid ping and returns the frequency."""
    samples = samples.copy()
    samples -= numpy.mean(samples[:, 0:32], 1)[:, numpy.newaxis]
    sample_count = samples.shape[1]
    
    samples = bandpass(samples, sample_rate)
    samples_window = samples * numpy.hamming(sample_count)
    plt.figure()
    plt.plot(samples_window.transpose())
    
    fft_length = 2048
    samples_fft = numpy.absolute(numpy.fft.fft(samples_window, fft_length, axis=1))[:, :fft_length/2]
    peaks = numpy.argmax(samples_fft, axis=1)
    
    # Sort peaks, take mean of the middle
    middle_peaks = numpy.sort(peaks)[1:3] #FIXME
    peak = numpy.mean(middle_peaks)

    freq = bin_to_freq(peak, sample_rate, fft_length)
    
    if plot:
        plt.figure()
        plt.plot(bin_to_freq(numpy.arange(0, fft_length/2), sample_rate, fft_length), 
                 samples_fft.transpose())
        plt.title('FFT')

    return True, freq

def upsample(samples, sample_rate, desired_sample_rate):
    """Upsamples data to have approx. desired_sample_rate."""
    upfact = round(desired_sample_rate/sample_rate)
    upsamples = numpy.empty((samples.shape[0], samples.shape[1]*upfact))
    for i in xrange(samples.shape[0]):
        upsamples[i, :] = util.resample(samples[i, :], upfact, 1)
    return upsamples, sample_rate*upfact

def compute_deltas(samples, sample_rate, ping_freq, template_periods=3, plot=False):
    """
    Computes N-1 position deltas for N channels, by making a template
    for the first channel and matching to all subsequent channels.
    """
    period = int(round(sample_rate / ping_freq))
    template, template_pos = make_template(samples[0, :],
                                           .05,
                                           period*template_periods*3+1)
    start = template_pos - period//2
    stop = template_pos + period//2

    deltas = numpy.empty(samples.shape[0]-1)
    for i in xrange(deltas.shape[0]):
        pos = match_template(samples[i+1, :], start, stop, template)
        deltas[i] = pos - template_pos

    if plot:
        plot_start = start - period
        plot_stop = stop + template.shape[0] + period
        plt.ioff()
        plt.figure()
        plt.plot(template)
        plt.title('Template')

        for i in xrange(deltas.shape[0]):
            plt.figure()
            plt.plot(numpy.arange(plot_start, plot_stop), samples[i+1, plot_start:plot_stop])
            pos = template_pos + int(round(deltas[i]))
            plt.plot(numpy.arange(pos, pos + template.shape[0]), template)
            plt.title('Channel %d' % (i+1))
    return deltas

def make_template(channel, thresh, width):
    """
    Returns a template of the specified width, centered at the first
    point of the channel to exceed thresh.
    """
    pos = 0
    while pos < channel.shape[0] and abs(channel[pos]) < thresh:
        pos += 1
    pos -= width//2
    if pos < 0 or pos + width >= channel.shape[0]:
        raise AlgorithmError('Template positioned such that start or end points lie outside of data')
    return channel[pos:pos+width], pos

def match_template(channel, start, stop, template):
    """
    Matches template to channel, returning the point where the start
    of the template should be placed.
    """
    start = max(start, 0)
    stop = min(stop, channel.shape[0] - template.shape[0])
    mad = mean_absolute_difference(channel, start, stop, template)
    min_pt = find_zero(mad)
    if min_pt is None:
        raise AlgorithmError('No minimums near numpy.argmin??')

    return start + min_pt

def mean_absolute_difference(channel, start, stop, template):
    """
    Slides the template along channel from start to stop, giving the
    mean absolute difference of the template and channel at each
    location.
    """
    width = template.shape[0]
    mad = numpy.zeros(stop-start)
    for i in xrange(start, stop):
        mad[i-start] = numpy.mean(numpy.abs(channel[i:i+width] - template))
    return mad

def find_zero(data):
    """
    Finds the sub-sample position of the first zero in a continuous signal,
    by first finding the lowest absolute value, then taking the gradient
    and performing a linear interpolation near the lowest absolute value.
    """
    approx = numpy.argmin(numpy.abs(data))
    d_data = numpy.gradient(data)

    for pos in xrange(max(approx-3,0), min(approx+3, d_data.shape[0]-1)):
        if numpy.sign(d_data[pos]) != numpy.sign(d_data[pos+1]):
            y2 = d_data[pos+1]
            y1 = d_data[pos]
            x2 = pos+1
            x1 = pos
            return -(x2-x1)/(y2-y1)*y1 + x1
    return approx

def compute_pos_4hyd(deltas, sample_rate, v_sound, dist_h, dist_h4):
    """
    Solves the 4 hydrophone case (3 deltas) for heading, declination,
    and sph_dist using a bunch of trig. Future work would be to phrase
    this as a NLLSQ problem or something, and adapt it to more
    hydrophones.
    """
    assert(len(deltas) == 3)

    y1 = deltas[0]/sample_rate*v_sound
    y2 = deltas[1]/sample_rate*v_sound
    y3 = deltas[2]/sample_rate*v_sound

    dist = abs((y1**2 + y2**2 - 2*dist_h**2)/(2*y1 + 2*y2))
    cos_alpha1 = (2*dist*y1 + y1**2 - dist_h**2)/(-2*dist*dist_h);
    cos_alpha2 = -(2*dist*y2 + y2**2 - dist_h**2)/(-2*dist*dist_h);
    cos_alpha = (cos_alpha1 + cos_alpha2)/2;

    cos_beta = (2*dist*y3 + y3**2 - dist_h4**2)/(-2*dist*dist_h4);

    dist_x = cos_alpha*dist
    dist_y = cos_beta*dist
    if dist**2 - (dist_x**2 + dist_y**2) < 0:
        dist_z = 0
    else:
        dist_z = -math.sqrt(dist**2 - (dist_x**2 + dist_y**2))
    return numpy.array([dist_x, dist_y, dist_z])

if __name__ == '__main__':
    sample_rate = 300e3
    if len(sys.argv) > 1:
        samples = numpy.loadtxt(sys.argv[1], delimiter=',').transpose()
    else:
        samples = util.make_ping([0, .25, 1.234, -5], {'freq': 25e3, 'sample_rate': sample_rate})

    try:
        plt.figure()
        plt.plot(samples.transpose())
        plt.title('Raw ping')
        
        valid, freq = check_data(samples, sample_rate, plot=True)
        print 'freq', freq

        samples_proc, sample_rate_proc = preprocess(samples, sample_rate)
        plt.figure()
        plt.plot(samples_proc.transpose())
        plt.title('Processed ping')

        deltas = compute_deltas(samples_proc, sample_rate_proc, freq, plot=True)
        print 'deltas', deltas

        pos = compute_pos_4hyd(deltas, sample_rate_proc, 1497, 2.286000e-02, 2.286000e-02)
        print 'pos (hyd coordinates)', pos
    finally:
        plt.show()

