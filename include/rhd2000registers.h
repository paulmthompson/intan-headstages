//----------------------------------------------------------------------------------
// rhd2000registers.h
//
// Intan Technoloies RHD2000 Rhythm Interface API
// Rhd2000Registers Class Header File
// Version 1.4 (26 February 2014)
//
// Copyright (c) 2013-2014 Intan Technologies LLC
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the
// use of this software.
//
// Permission is granted to anyone to use this software for any applications that
// use Intan Technologies integrated circuits, and to alter it and redistribute it
// freely.
//
// See http://www.intantech.com for documentation and product information.
//----------------------------------------------------------------------------------

#ifndef RHD2000REGISTERS_H
#define RHD2000REGISTERS_H

#include "IntanCommon.h"

class Rhd2000Registers : public IntanCommon
{

public:
    Rhd2000Registers(double sampleRate);

    void defineSampleRate(double newSampleRate);

    void setFastSettle(bool enabled);

    void setDigOutLow();
    void setDigOutHigh();
    void setDigOutHiZ();

    void enableAux1(bool enabled);
    void enableAux2(bool enabled);
    void enableAux3(bool enabled);

    void disableDsp();

    void setZcheckPolarity(ZcheckPolarity polarity);
    int setZcheckChannel(int channel);

    void setAmpPowered(int channel, bool powered);
    void powerUpAllAmps();
    void powerDownAllAmps();

    int getRegisterValue(int reg) const;

    double setLowerBandwidth(double lowerBandwidth);

    int createCommandListRegisterConfig(std::vector<int> &commandList, bool calibrate);
    int createCommandListTempSensor(std::vector<int> &commandList);
    int createCommandListUpdateDigOut(std::vector<int> &commandList);
    int createCommandListZcheckDac(std::vector<int> &commandList, double frequency, double amplitude);

    enum Rhd2000CommandType {
        Rhd2000CommandConvert,
        Rhd2000CommandCalibrate,
        Rhd2000CommandCalClear,
        Rhd2000CommandRegWrite,
        Rhd2000CommandRegRead
    };

    int createRhd2000Command(Rhd2000CommandType commandType);
    int createRhd2000Command(Rhd2000CommandType commandType, int arg1);
    int createRhd2000Command(Rhd2000CommandType commandType, int arg1, int arg2);

private:

    // RHD2000 Register 0 variables
    int adcReferenceBw;
    int ampFastSettle;
    int ampVrefEnable;
    int adcComparatorBias;
    int adcComparatorSelect;

    // RHD2000 Register 1 variables
    int vddSenseEnable;

    // RHD2000 Register 3 variables
    int muxLoad;
    int tempS1;
    int tempS2;
    int tempEn;
    int digOutHiZ;
    int digOut;

    // RHD2000 Register 5 variables
    int zcheckDacPower;
    int zcheckConnAll;
    int zcheckSelPol;

    // RHD2000 Register 6 variables
    //int zcheckDac;     // handle Zcheck DAC waveform elsewhere

    // RHD2000 Register 8-13 variables
    int offChipRH1;
    int offChipRH2;
    int offChipRL;
    int adcAux1En;
    int adcAux2En;
    int adcAux3En;
    int rLDac1;
    int rLDac2;
    int rLDac3;

    static const int MaxCommandLength = 1024; // size of on-FPGA auxiliary command RAM banks

};

#endif // RHD2000REGISTERS_H
