//----------------------------------------------------------------------------------
// rhs2000registers.h
//
// Intan Technoloies RHS2000 Interface API
// Rhs2000Registers Class Header File
// Version 1.01 (28 March 2017)
//
// Copyright (c) 2013-2017 Intan Technologies LLC
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

#ifndef RHS2000REGISTERS_H
#define RHS2000REGISTERS_H

#include "IntanCommon.h"

#include <vector>

class Rhs2000Registers : public IntanCommon
{

public:
    enum StimStepSize {
        StimStepSizeMin = 0,
        StimStepSize10nA = 1,
        StimStepSize20nA = 2,
        StimStepSize50nA = 3,
        StimStepSize100nA = 4,
        StimStepSize200nA = 5,
        StimStepSize500nA = 6,
        StimStepSize1uA = 7,
        StimStepSize2uA = 8,
        StimStepSize5uA = 9,
        StimStepSize10uA = 10,
        StimStepSizeMax = 11
    };

    Rhs2000Registers(double sampleRate, StimStepSize stimStep_);

	void defineSampleRate(double newSampleRate);

	enum DigOut {
		DigOut1,
		DigOut2,
		DigOutOD
	};

	void setDigOutLow(DigOut pin);
	void setDigOutHigh(DigOut pin);
	void setDigOutHiZ(DigOut pin);

	void setZcheckDacPower(bool enabled);

	int setZcheckChannel(int channel);

	void setAmpPowered(int channel, bool powered);
	void powerUpAllAmps();
	void powerDownAllAmps();
	void setDCAmpPowered(int channel, bool powered);
	void powerUpAllDCAmps();
	void powerDownAllDCAmps();

	void setStimEnable(bool enable);

	int getRegisterValue(int reg);

	double setLowerBandwidth(double lowerBandwidth, int select);

	void setStimStepSize(StimStepSize step);
    static double stimStepSizeToDouble(StimStepSize step);
	int setPosStimMagnitude(int channel, int magnitude, int trim);
	int setNegStimMagnitude(int channel, int magnitude, int trim);

	enum ChargeRecoveryCurrentLimit {
        CurrentLimitMin = 0,
        CurrentLimit1nA,
        CurrentLimit2nA,
        CurrentLimit5nA,
        CurrentLimit10nA,
        CurrentLimit20nA,
        CurrentLimit50nA,
        CurrentLimit100nA,
        CurrentLimit200nA,
        CurrentLimit500nA,
        CurrentLimit1uA,
        CurrentLimitMax
	};

	void setChargeRecoveryCurrentLimit(ChargeRecoveryCurrentLimit limit);
    static double chargeRecoveryCurrentLimitToDouble(ChargeRecoveryCurrentLimit limit);
	double setChargeRecoveryTargetVoltage(double vTarget);

    int createCommandListRegisterConfig(std::vector<unsigned int> &commandList, bool updateStimParams);
    int createCommandListRegisterRead(std::vector<unsigned int> &commandList);
	int createCommandListZcheckDac(std::vector<unsigned int> &commandList, double frequency, double amplitude);
	int createCommandListDummy(std::vector <unsigned int> &commandList, int n);
	int createCommandListDummy(std::vector <unsigned int> &commandList, int n, unsigned int cmd);
	int createCommandListSingleRegisterConfig(std::vector<unsigned int> &commandList, int reg);
    int createCommandListSetStimMagnitudes(std::vector<unsigned int> &commandList, int channel,
                                           int posMag, int posTrim, int negMag, int negTrim);
    int createCommandListConfigChargeRecovery(std::vector<unsigned int> &commandList, ChargeRecoveryCurrentLimit currentLimit, double targetVoltage);

	enum Rhs2000CommandType {
		Rhs2000CommandConvert,
		Rhs2000CommandCalibrate,
		Rhs2000CommandCalClear,
		Rhs2000CommandRegWrite,
		Rhs2000CommandRegRead,
		Rhs2000CommandComplianceReset
	};

	unsigned int createRhs2000Command(Rhs2000CommandType commandType);
	unsigned int createRhs2000Command(Rhs2000CommandType commandType, unsigned int arg1);
	unsigned int createRhs2000Command(Rhs2000CommandType commandType, unsigned int arg1, unsigned int arg2);
	unsigned int createRhs2000Command(Rhs2000CommandType commandType, unsigned int arg1, unsigned int arg2, unsigned int uFlag, unsigned int mFlag);

private:
    StimStepSize stimStep;

	// RHS2000 Register 0 variables
	int adcBufferBias;

	// RHS2000 Register 1 variables
	int digOutOD;
	int digOut1;
	int digOut1HiZ;
	int digOut2;
	int digOut2HiZ;

	// RHS2000 Register 2 variables
	

	// RHS2000 Register 3 variables
    // int zcheckDac;     // handle Zcheck DAC waveform elsewhere

	// RHS2000 Register 4-7 variables

	int rLDac1A;
	int rLDac2A;
	int rLDac3A;
	int rLDac1B;
	int rLDac2B;
	int rLDac3B;

	// RHS2000 Register 10 variables
	std::vector<int> ampFastSettle;

	// RHS2000 Register 12 variables
	std::vector<int> ampFLSelect;

	// RHS2000 Register 32 variables
	int stimEnableA;

	// RHS2000 Register 33 variables
	int stimEnableB;

	// RHS2000 Register 34 variables
	int stimStepSel1;
	int stimStepSel2;
	int stimStepSel3;

	// RHS2000 Register 35 variables
	int stimNBias;
	int stimPBias;

	// RHS2000 Register 36 variables
	int chargeRecoveryDac;

	// RHS2000 Register 37 variables
	int chargeRecoveryCurrentLimitSel1;
	int chargeRecoveryCurrentLimitSel2;
	int chargeRecoveryCurrentLimitSel3;

	// RHS2000 Register 38 variables
	std::vector<int> dcAmpPwr;

	// RHS2000 Register 40 variables
	std::vector<int> complianceMonitor;

	// RHS2000 Register 42 variables
	std::vector<int> stimOn;

	// RHS2000 Register 44 variables
	std::vector<int> stimPol;

	// RHS2000 Register 46 variables
	std::vector<int> chargeRecoverySwitch;

	// RHS2000 Register 48 variables
	std::vector<int> cLChargeRecoveryEn;

	// RHS2000 Register 50 variables
	int faultCurrentDetect;

	// RHS2000 Register 64-79 variables
	std::vector<int> negCurrentMag;
	std::vector<int> negCurrentTrim;

	// RHS2000 Register 96-111 variables
	std::vector<int> posCurrentMag;
	std::vector<int> posCurrentTrim;

	int vectorToWord(std::vector<int> &v);

	static const int MaxCommandLength = 8192; // size of on-FPGA auxiliary command RAM banks

};

#endif // RHS2000REGISTERS_H
