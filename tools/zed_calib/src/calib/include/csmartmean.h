#ifndef CSMARTMEAN_H
#define CSMARTMEAN_H

/*!
 * \brief The CSmartMean class is used to
 * make a mobile window mean of a sequence of values
 * and reject outliers.
 * Tutorial:
 * https://www.myzhar.com/blog/tutorials/tutorial-exponential-weighted-average-good-moving-windows-average/
 */
class CSmartMean {
  public:
    CSmartMean(unsigned int winSize=100);

    int getValCount() {
        return mValCount;   ///< Return the number of values in the sequence
    }

    double getMean() {
        return mMean;   ///< Return the updated mean
    }

    /*!
     * \brief addValue
     * Add a value to the sequence
     * \param val value to be added
     * \return mean value
     */
    double addValue(double val);

    /*!
     * \brief setWinSize
     * Set the size of the mean window
     * \param size size of the window
     */
    void setWinSize(unsigned int winSize);

    /*!
     * \brief reset
     * Reset the mean
     */
    void reset();

private:
    int mWinSize; ///< The size of the window (number of values ti evaluate)
    int mValCount; ///< The number of values in sequence

    double mMeanCorr; ///< Used for bias correction
    double mMean;     ///< The mean of the last \ref mWinSize values

    double mGamma; ///< Weight value
};

#endif // CSMARTMEAN_H
