//
//  simple_fft.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/31/13.
//
//

#ifndef __SystemGenerator__simple_fft__
#define __SystemGenerator__simple_fft__

unsigned int bitReverse(unsigned int x, int log2n);

//frequency to fft-bin(index in result) equation is  freq = (bin_id * sample_freq/2) / (N/2)
// typically only first N/2 values will be useful 
template<class Iter_T> void fft(Iter_T a, Iter_T b, int log2n);


void fft_test();
#endif /* defined(__SystemGenerator__simple_fft__) */
