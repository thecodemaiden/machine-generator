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
template<class Iter_T> void fft(Iter_T a, Iter_T b, int log2n);

#endif /* defined(__SystemGenerator__simple_fft__) */
