#include "csmartmean.h"

#include <math.h>

CSmartMean::CSmartMean(unsigned int winSize) {
    setWinSize(winSize);
}

void CSmartMean::setWinSize(unsigned int winSize) {

    if( winSize==mWinSize )
        return;

    mValCount = 0;

    mMeanCorr = 0.0;
    mMean = 0.0;
    mWinSize = winSize;

    mGamma = (static_cast<double>(mWinSize) - 1.) / static_cast<double>(mWinSize);
}

void CSmartMean::reset(){
    mValCount = 0;
    mMeanCorr = 0.0;
    mMean = 0.0;
}

double CSmartMean::addValue(double val) {
    mValCount++;

    mMeanCorr = mGamma * mMeanCorr + (1. - mGamma) * val;
    mMean = mMeanCorr / (1. - pow(mGamma, mValCount));

    return mMean;
}

