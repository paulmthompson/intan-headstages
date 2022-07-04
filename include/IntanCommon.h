
#ifndef INTANCOMMON_H
#define INTANCOMMON_H

#include <vector>

class IntanCommon
{
    public:

    enum ZcheckCs {
		ZcheckCs100fF,
		ZcheckCs1pF,
		ZcheckCs10pF
	};

    enum ZcheckPolarity {
        ZcheckPositiveInput,
        ZcheckNegativeInput
    };

    void enableDsp(bool enabled);
    void enableZcheck(bool enabled);
    void setZcheckDacPower(bool enabled);
    void setZcheckScale(ZcheckCs scale);

    double setDspCutoffFreq(double newDspCutoffFreq);
    double getDspCutoffFreq() const;

    double setUpperBandwidth(double upperBandwidth);

    protected:
        double sampleRate;
        int adcBufferBias;

        int muxBias;

        int twosComp;

        int weakMiso;
	    int absMode;
	    int dspEn;
	    int dspCutoffFreq;

        int zcheckLoad;
        int zcheckScale;
        int zcheckEn;
        int zcheckDacPower;
        int zcheckSelect;

        int rH1Dac1;
	    int rH1Dac2;
	    int rH2Dac1;
	    int rH2Dac2;

        std::vector<int> ampPwr;

        double rH1FromUpperBandwidth(double upperBandwidth) const;
	    double rH2FromUpperBandwidth(double upperBandwidth) const;
        double rLFromLowerBandwidth(double lowerBandwidth) const;
        double upperBandwidthFromRH1(double rH1) const;
        double upperBandwidthFromRH2(double rH2) const;
        double lowerBandwidthFromRL(double rL) const;

};

#endif // INTANCOMMON_H