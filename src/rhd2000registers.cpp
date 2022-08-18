//----------------------------------------------------------------------------------
// rhd2000registers.cpp
//
// Intan Technoloies RHD2000 Rhythm Interface API
// Rhd2000Registers Class
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

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <queue>
#include <numbers>

#include "rhd2000registers.h"

// This class creates and manages a data structure representing the internal RAM registers on
// a RHD2000 chip, and generates command lists to configure the chip and perform other functions.
// Changing the value of variables within an instance of this class does not directly affect a
// RHD2000 chip connected to the FPGA; rather, a command list must be generated from this class
// and then downloaded to the FPGA board using Rhd2000EvalBoard::uploadCommandList.

// Constructor.  Set RHD2000 register variables to default values.
Rhd2000Registers::Rhd2000Registers(double sampleRate)
{
    ampPwr.resize(64);

    defineSampleRate(sampleRate);

    // Set default values for all register settings
    adcReferenceBw = 3;         // ADC reference generator bandwidth (0 [highest BW] - 3 [lowest BW]);
                                // always set to 3
    setFastSettle(false);       // amplifier fast settle (off = normal operation)
    ampVrefEnable = 1;          // enable amplifier voltage references (0 = power down; 1 = enable);
                                // 1 = normal operation
    adcComparatorBias = 3;      // ADC comparator preamp bias current (0 [lowest] - 3 [highest], only
                                // valid for comparator select = 2,3); always set to 3
    adcComparatorSelect = 2;    // ADC comparator select; always set to 2

    vddSenseEnable = 1;         // supply voltage sensor enable (0 = disable; 1 = enable)
    // adcBufferBias = 32;      // ADC reference buffer bias current (0 [highest current] - 63 [lowest current]);
                                // This value should be set according to ADC sampling rate; set in setSampleRate()

    // muxBias = 40;            // ADC input MUX bias current (0 [highest current] - 63 [lowest current]);
                                // This value should be set according to ADC sampling rate; set in setSampleRate()

    // muxLoad = 0;             // MUX capacitance load at ADC input (0 [min CL] - 7 [max CL]); LSB = 3 pF
                                // Set in setSampleRate()

    tempS1 = 0;                 // temperature sensor S1 (0-1); 0 = power saving mode when temperature sensor is
                                // not in use
    tempS2 = 0;                 // temperature sensor S2 (0-1); 0 = power saving mode when temperature sensor is
                                // not in use
    tempEn = 0;                 // temperature sensor enable (0 = disable; 1 = enable)
    setDigOutHiZ();             // auxiliary digital output state

    weakMiso = 1;               // weak MISO (0 = MISO line is HiZ when CS is inactive; 1 = MISO line is weakly
                                // driven when CS is inactive)
    twosComp = 0;               // two's complement ADC results (0 = unsigned offset representation; 1 = signed
                                // representation)
    absMode = 0;                // absolute value mode (0 = normal output; 1 = output passed through abs(x) function)
    enableDsp(true);            // DSP offset removal enable/disable
    setDspCutoffFreq(1.0);      // DSP offset removal HPF cutoff freqeuncy

    zcheckDacPower = 1;         // impedance testing DAC power-up (0 = power down; 1 = power up)
    zcheckLoad = 0;             // impedance testing dummy load (0 = normal operation; 1 = insert 60 pF to ground)
    setZcheckScale(ZcheckCs100fF);  // impedance testing scale factor (100 fF, 1.0 pF, or 10.0 pF)
    zcheckConnAll = 0;          // impedance testing connect all (0 = normal operation; 1 = connect all electrodes together)
    setZcheckPolarity(ZcheckPositiveInput); // impedance testing polarity select (RHD2216 only) (0 = test positive inputs;
                                // 1 = test negative inputs)
    enableZcheck(false);        // impedance testing enable/disable

    setZcheckChannel(0);        // impedance testing amplifier select (0-63)

    offChipRH1 = 0;             // bandwidth resistor RH1 on/off chip (0 = on chip; 1 = off chip)
    offChipRH2 = 0;             // bandwidth resistor RH2 on/off chip (0 = on chip; 1 = off chip)
    offChipRL = 0;              // bandwidth resistor RL on/off chip (0 = on chip; 1 = off chip)
    adcAux1En = 1;              // enable ADC aux1 input (when RH1 is on chip) (0 = disable; 1 = enable)
    adcAux2En = 1;              // enable ADC aux2 input (when RH2 is on chip) (0 = disable; 1 = enable)
    adcAux3En = 1;              // enable ADC aux3 input (when RL is on chip) (0 = disable; 1 = enable)

    setUpperBandwidth(10000.0); // set upper bandwidth of amplifiers
    setLowerBandwidth(1.0);     // set lower bandwidth of amplifiers

    powerUpAllAmps();           // turn on all amplifiers
}

// Define RHD2000 per-channel sampling rate so that certain sampling-rate-dependent registers are set correctly
// (This function does not change the sampling rate of the FPGA; for this, use Rhd2000EvalBoard::setSampleRate.)
void Rhd2000Registers::defineSampleRate(double newSampleRate)
{
    this->sampleRate = newSampleRate;

    this->muxLoad = 0;

    if (sampleRate < 3334.0) {
        muxBias = 40;
        adcBufferBias = 32;
    } else if (sampleRate < 4001.0) {
        muxBias = 40;
        adcBufferBias = 16;
    } else if (sampleRate < 5001.0) {
        muxBias = 40;
        adcBufferBias = 8;
    } else if (sampleRate < 6251.0) {
        muxBias = 32;
        adcBufferBias = 8;
    } else if (sampleRate < 8001.0) {
        muxBias = 26;
        adcBufferBias = 8;
    } else if (sampleRate < 10001.0) {
        muxBias = 18;
        adcBufferBias = 4;
    } else if (sampleRate < 12501.0) {
        muxBias = 16;
        adcBufferBias = 3;
    } else if (sampleRate < 15001.0) {
        muxBias = 7;
        adcBufferBias = 3;
    } else {
        muxBias = 4;
        adcBufferBias = 2;
    }
}

// Enable or disable amplifier fast settle function; drive amplifiers to baseline
// if enabled.
void Rhd2000Registers::setFastSettle(bool enabled)
{
    ampFastSettle = (enabled ? 1 : 0);
}

// Drive auxiliary digital output low
void Rhd2000Registers::setDigOutLow()
{
    digOut = 0;
    digOutHiZ = 0;
}

// Drive auxiliary digital output high
void Rhd2000Registers::setDigOutHigh()
{
    digOut = 1;
    digOutHiZ = 0;
}

// Set auxiliary digital output to high-impedance (HiZ) state
void Rhd2000Registers::setDigOutHiZ()
{
    digOut = 0;
    digOutHiZ = 1;
}

// Enable or disable ADC auxiliary input 1
void Rhd2000Registers::enableAux1(bool enabled)
{
    adcAux1En = (enabled ? 1 : 0);
}

// Enable or disable ADC auxiliary input 2
void Rhd2000Registers::enableAux2(bool enabled)
{
    adcAux2En = (enabled ? 1 : 0);
}

// Enable or disable ADC auxiliary input 3
void Rhd2000Registers::enableAux3(bool enabled)
{
    adcAux3En = (enabled ? 1 : 0);
}

// Select impedance testing of positive or negative amplifier inputs (RHD2216 only), based
// on the variable polarity (ZcheckPositiveInput or ZcheckNegativeInput)
void Rhd2000Registers::setZcheckPolarity(ZcheckPolarity polarity)
{
    switch (polarity) {
        case ZcheckPositiveInput:
            zcheckSelPol = 0;
            break;
    case ZcheckNegativeInput:
            zcheckSelPol = 1;
            break;
    }
}

// Select the amplifier channel (0-63) for impedance testing.
int Rhd2000Registers::setZcheckChannel(int channel)
{
    if (channel < 0 || channel > 63) {
        return -1;
    } else {
        zcheckSelect = channel;
        return zcheckSelect;
    }
}

// Power up or down selected amplifier on chip
void Rhd2000Registers::setAmpPowered(int channel, bool powered)
{
    if (channel >= 0 && channel <= 63) {
        ampPwr[channel] = (powered ? 1 : 0);
    }
}

// Power up all amplifiers on chip
void Rhd2000Registers::powerUpAllAmps()
{
    for (int channel = 0; channel < 64; ++channel) {
        ampPwr[channel] = 1;
    }
}

// Power down all amplifiers on chip
void Rhd2000Registers::powerDownAllAmps()
{
    for (int channel = 0; channel < 64; ++channel) {
        ampPwr[channel] = 0;
    }
}

// Returns the value of a selected RAM register (0-17) on the RHD2000 chip, based
// on the current register variables in the class instance.
int Rhd2000Registers::getRegisterValue(int reg) const
{
    int regout;
    const int zcheckDac = 128;  // midrange

    switch (reg) {
    case 0:
        regout = (adcReferenceBw << 6) + (ampFastSettle << 5) + (ampVrefEnable << 4) +
                (adcComparatorBias << 2) + adcComparatorSelect;
        break;
    case 1:
        regout = (vddSenseEnable << 6) + adcBufferBias;
        break;
    case 2:
        regout = muxBias;
        break;
    case 3:
        regout = (muxLoad << 5) + (tempS2 << 4) + (tempS1 << 3) + (tempEn << 2) +
                (digOutHiZ << 1) + digOut;
        break;
    case 4:
        regout = (weakMiso << 7) + (twosComp << 6) + (absMode << 5) + (dspEn << 4) +
                dspCutoffFreq;
        break;
    case 5:
        regout = (zcheckDacPower << 6) + (zcheckLoad << 5) + (zcheckScale << 3) +
                (zcheckConnAll << 2) + (zcheckSelPol << 1) + zcheckEn;
        break;
    case 6:
        regout = zcheckDac;
        break;
    case 7:
        regout = zcheckSelect;
        break;
    case 8:
        regout = (offChipRH1 << 7) + rH1Dac1;
        break;
    case 9:
        regout = (adcAux1En << 7) + rH1Dac2;
        break;
    case 10:
        regout = (offChipRH2 << 7) + rH2Dac1;
        break;
    case 11:
        regout = (adcAux2En << 7) + rH2Dac2;
        break;
    case 12:
        regout = (offChipRL << 7) + rLDac1;
        break;
    case 13:
        regout = (adcAux3En << 7) + (rLDac3 << 6) + rLDac2;
        break;
    case 14:
        regout = (ampPwr[7] << 7) + (ampPwr[6] << 6) + (ampPwr[5] << 5) + (ampPwr[4] << 4) +
                (ampPwr[3] << 3) + (ampPwr[2] << 2) + (ampPwr[1] << 1) + ampPwr[0];
        break;
    case 15:
        regout = (ampPwr[15] << 7) + (ampPwr[14] << 6) + (ampPwr[13] << 5) + (ampPwr[12] << 4) +
                (ampPwr[11] << 3) + (ampPwr[10] << 2) + (ampPwr[9] << 1) + ampPwr[0];
        break;
    case 16:
        regout = (ampPwr[23] << 7) + (ampPwr[22] << 6) + (ampPwr[21] << 5) + (ampPwr[20] << 4) +
                (ampPwr[19] << 3) + (ampPwr[18] << 2) + (ampPwr[17] << 1) + ampPwr[16];
        break;
    case 17:
        regout = (ampPwr[31] << 7) + (ampPwr[30] << 6) + (ampPwr[29] << 5) + (ampPwr[28] << 4) +
                (ampPwr[27] << 3) + (ampPwr[26] << 2) + (ampPwr[25] << 1) + ampPwr[24];
        break;
    case 18:
        regout = (ampPwr[39] << 7) + (ampPwr[38] << 6) + (ampPwr[37] << 5) + (ampPwr[36] << 4) +
                (ampPwr[35] << 3) + (ampPwr[34] << 2) + (ampPwr[33] << 1) + ampPwr[32];
        break;
    case 19:
        regout = (ampPwr[47] << 7) + (ampPwr[46] << 6) + (ampPwr[45] << 5) + (ampPwr[44] << 4) +
                (ampPwr[43] << 3) + (ampPwr[42] << 2) + (ampPwr[41] << 1) + ampPwr[40];
        break;
    case 20:
        regout = (ampPwr[55] << 7) + (ampPwr[54] << 6) + (ampPwr[53] << 5) + (ampPwr[52] << 4) +
                (ampPwr[51] << 3) + (ampPwr[50] << 2) + (ampPwr[49] << 1) + ampPwr[48];
        break;
    case 21:
        regout = (ampPwr[63] << 7) + (ampPwr[62] << 6) + (ampPwr[61] << 5) + (ampPwr[60] << 4) +
                (ampPwr[59] << 3) + (ampPwr[58] << 2) + (ampPwr[57] << 1) + ampPwr[56];
        break;
    default:
        regout = -1;
    }
    return regout;
}

// Sets the on-chip RL DAC values appropriately to set a particular amplifier
// lower bandwidth (in Hz).  Returns an estimate of the actual lower bandwidth achieved.
double Rhd2000Registers::setLowerBandwidth(double lowerBandwidth)
{
    const double RLBase = 3500.0;
    const double RLDac1Unit = 175.0;
    const double RLDac2Unit = 12700.0;
    const double RLDac3Unit = 3000000.0;
    const int RLDac1Steps = 127;
    const int RLDac2Steps = 63;

    double actualLowerBandwidth;
    double rLTarget;
    double rLActual;
    int i;

    // Lower bandwidths higher than 1.5 kHz don't work well with the RHD2000 amplifiers
    if (lowerBandwidth > 1500.0) {
        lowerBandwidth = 1500.0;
    }

    rLTarget = rLFromLowerBandwidth(lowerBandwidth);

    rLDac1 = 0;
    rLDac2 = 0;
    rLDac3 = 0;
    rLActual = RLBase;

    if (lowerBandwidth < 0.15) {
        rLActual += RLDac3Unit;
        ++rLDac3;
    }

    for (i = 0; i < RLDac2Steps; ++i) {
        if (rLActual < rLTarget - (RLDac2Unit - RLDac1Unit / 2)) {
            rLActual += RLDac2Unit;
            ++rLDac2;
        }
    }

    for (i = 0; i < RLDac1Steps; ++i) {
        if (rLActual < rLTarget - (RLDac1Unit / 2)) {
            rLActual += RLDac1Unit;
            ++rLDac1;
        }
    }

    actualLowerBandwidth = lowerBandwidthFromRL(rLActual);

    /*
    std::cout << std::endl;
    std::cout << fixed << setprecision(1);
    std::cout << "Rhd2000Registers::setLowerBandwidth" << std::endl;

    std::cout << "RL DAC3 = " << rLDac3 << ", DAC2 = " << rLDac2 << ", DAC1 = " << rLDac1 << std::endl;
    std::cout << "RL target: " << rLTarget << " Ohms" << std::endl;
    std::cout << "RL actual: " << rLActual << " Ohms" << std::endl;

    std::cout << setprecision(3);

    std::cout << "Lower bandwidth target: " << lowerBandwidth << " Hz" << std::endl;
    std::cout << "Lower bandwidth actual: " << actualLowerBandwidth << " Hz" << std::endl;

    std::cout << std::endl;
    std::cout << setprecision(6);
    std::cout.unsetf(ios::floatfield);
    */

    return actualLowerBandwidth;
}

// Return a 16-bit MOSI command (CALIBRATE or CLEAR)
int Rhd2000Registers::createRhd2000Command(Rhd2000CommandType commandType)
{
    switch (commandType) {
        case Rhd2000CommandCalibrate:
            return 0x5500;   // 0101010100000000
            break;
        case Rhd2000CommandCalClear:
            return 0x6a00;   // 0110101000000000
            break;
        default:
        std::cerr << "Error in Rhd2000Registers::createRhd2000Command: " <<
                "Only 'Calibrate' or 'Clear Calibration' commands take zero arguments." << std::endl;
            return -1;
    }
}

// Return a 16-bit MOSI command (CONVERT or READ)
int Rhd2000Registers::createRhd2000Command(Rhd2000CommandType commandType, int arg1)
{
    switch (commandType) {
        case Rhd2000CommandConvert:
            if (arg1 < 0 || arg1 > 63) {
                std::cerr << "Error in Rhd2000Registers::createRhd2000Command: " <<
                        "Channel number out of range." << std::endl;
                return -1;
            }
            return 0x0000 + (arg1 << 8);  // 00cccccc0000000h; if the command is 'Convert',
                                          // arg1 is the channel number
        case Rhd2000CommandRegRead:
            if (arg1 < 0 || arg1 > 63) {
                std::cerr << "Error in Rhd2000Registers::createRhd2000Command: " <<
                        "Register address out of range." << std::endl;
                return -1;
            }
            return 0xc000 + (arg1 << 8);  // 11rrrrrr00000000; if the command is 'Register Read',
                                          // arg1 is the register address
            break;
        default:
            std::cerr << "Error in Rhd2000Registers::createRhd2000Command: " <<
                    "Only 'Convert' and 'Register Read' commands take one argument." << std::endl;
            return -1;
    }
}

// Return a 16-bit MOSI command (WRITE)
int Rhd2000Registers::createRhd2000Command(Rhd2000CommandType commandType, int arg1, int arg2)
{
    switch (commandType) {
        case Rhd2000CommandRegWrite:
            if (arg1 < 0 || arg1 > 63) {
                std::cerr << "Error in Rhd2000Registers::createRhd2000Command: " <<
                        "Register address out of range." << std::endl;
                return -1;
            }
            if (arg2 < 0 || arg2 > 255) {
                std::cerr << "Error in Rhd2000Registers::createRhd2000Command: " <<
                        "Register data out of range." << std::endl;
                return -1;
            }
            return 0x8000 + (arg1 << 8) + arg2; // 10rrrrrrdddddddd; if the command is 'Register Write',
                                                // arg1 is the register address and arg1 is the data
            break;
        default:
            std::cerr << "Error in Rhd2000Registers::createRhd2000Command: " <<
                    "Only 'Register Write' commands take two arguments." << std::endl;
            return -1;
    }
}


// Create a list of 60 commands to program most RAM registers on a RHD2000 chip, read those values
// back to confirm programming, read ROM registers, and (if calibrate == true) run ADC calibration.
// Returns the length of the command list.
int Rhd2000Registers::createCommandListRegisterConfig(std::vector<int> &commandList, bool calibrate)
{
    commandList.clear();    // if command list already exists, erase it and start a new one

    // Start with a few dummy commands in case chip is still powering up
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 63));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 63));

    // Program RAM registers
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite,  0, getRegisterValue( 0)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite,  1, getRegisterValue( 1)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite,  2, getRegisterValue( 2)));
    // Don't program Register 3 (MUX Load, Temperature Sensor, and Auxiliary Digital Output);
    // control temperature sensor in another command stream
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite,  4, getRegisterValue( 4)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite,  5, getRegisterValue( 5)));
    // Don't program Register 6 (Impedance Check DAC) here; create DAC waveform in another command stream
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite,  7, getRegisterValue( 7)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite,  8, getRegisterValue( 8)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite,  9, getRegisterValue( 9)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 10, getRegisterValue(10)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 11, getRegisterValue(11)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 12, getRegisterValue(12)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 13, getRegisterValue(13)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 14, getRegisterValue(14)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 15, getRegisterValue(15)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 16, getRegisterValue(16)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 17, getRegisterValue(17)));

    // Read ROM registers
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 63));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 62));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 61));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 60));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 59));

    // Read chip name from ROM
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 48));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 49));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 50));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 51));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 52));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 53));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 54));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 55));

    // Read Intan name from ROM
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 40));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 41));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 42));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 43));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 44));

    // Read back RAM registers to confirm programming
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  0));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  1));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  2));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  3));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  4));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  5));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  6));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  7));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  8));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead,  9));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 10));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 11));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 12));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 13));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 14));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 15));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 16));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 17));

    // Optionally, run ADC calibration (should only be run once after board is plugged in)
    if (calibrate) {
        commandList.push_back(createRhd2000Command(Rhd2000CommandCalibrate));
    } else {
        commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 63));
    }

    // Added in Version 1.2:
    // Program amplifier 31-63 power up/down registers in case a RHD2164 is connected
    // Note: We don't read these registers back, since they are only 'visible' on MISO B.
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 18, getRegisterValue(18)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 19, getRegisterValue(19)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 20, getRegisterValue(20)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 21, getRegisterValue(21)));

    // End with a dummy command
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 63));


    return commandList.size();
}

// Create a list of 60 commands to sample auxiliary ADC inputs, temperature sensor, and supply
// voltage sensor.  One temperature reading (one sample of ResultA and one sample of ResultB)
// is taken during this 60-command sequence.  One supply voltage sample is taken.  Auxiliary
// ADC inputs are continuously sampled at 1/4 the amplifier sampling rate.
//
// Since this command list consists of writing to Register 3, it also sets the state of the
// auxiliary digital output.  If the digital output value needs to be changed dynamically,
// then variations of this command list need to be generated for each state and programmed into
// different RAM banks, and the appropriate command list selected at the right time.
//
// Returns the length of the command list.
int Rhd2000Registers::createCommandListTempSensor(std::vector<int> &commandList)
{
    int i;

    commandList.clear();    // if command list already exists, erase it and start a new one

    tempEn = 1;

    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 32));     // sample AuxIn1
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 33));     // sample AuxIn2
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 34));     // sample AuxIn3
    tempS1 = tempEn;
    tempS2 = 0;
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 32));     // sample AuxIn1
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 33));     // sample AuxIn2
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 34));     // sample AuxIn3
    tempS1 = tempEn;
    tempS2 = tempEn;
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 32));     // sample AuxIn1
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 33));     // sample AuxIn2
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 34));     // sample AuxIn3
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 49));     // sample Temperature Sensor

    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 32));     // sample AuxIn1
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 33));     // sample AuxIn2
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 34));     // sample AuxIn3
    tempS1 = 0;
    tempS2 = tempEn;
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 32));     // sample AuxIn1
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 33));     // sample AuxIn2
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 34));     // sample AuxIn3
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 49));     // sample Temperature Sensor

    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 32));     // sample AuxIn1
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 33));     // sample AuxIn2
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 34));     // sample AuxIn3
    tempS1 = 0;
    tempS2 = 0;
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 32));     // sample AuxIn1
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 33));     // sample AuxIn2
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 34));     // sample AuxIn3
    commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 48));     // sample Supply Voltage Sensor

    for (i = 0; i < 8; ++i) {
        commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 32));     // sample AuxIn1
        commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 33));     // sample AuxIn2
        commandList.push_back(createRhd2000Command(Rhd2000CommandConvert, 34));     // sample AuxIn3
        commandList.push_back(createRhd2000Command(Rhd2000CommandRegRead, 63));      // dummy command
    }

    return commandList.size();
}

// Create a list of 60 commands to update Register 3 (controlling the auxiliary digital ouput
// pin) every sampling period.
//
// Since this command list consists of writing to Register 3, it also sets the state of the
// on-chip temperature sensor.  The temperature sensor settings are therefore changed throughout
// this command list to coordinate with the 60-command list generated by createCommandListTempSensor().
//
// Returns the length of the command list.
int Rhd2000Registers::createCommandListUpdateDigOut(std::vector<int> &commandList)
{
    int i;

    commandList.clear();    // if command list already exists, erase it and start a new one

    tempEn = 1;

    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    tempS1 = tempEn;
    tempS2 = 0;
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    tempS1 = tempEn;
    tempS2 = tempEn;
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    tempS1 = 0;
    tempS2 = tempEn;
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    tempS1 = 0;
    tempS2 = 0;
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));

    for (i = 0; i < 8; ++i) {
        commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
        commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
        commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
        commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 3, getRegisterValue(3)));
    }

    return commandList.size();
}

// Create a list of up to 1024 commands to generate a sine wave of particular frequency (in Hz) and
// amplitude (in DAC steps, 0-128) using the on-chip impedance testing voltage DAC.  If frequency is set to zero,
// a DC baseline waveform is created.
// Returns the length of the command list.
int Rhd2000Registers::createCommandListZcheckDac(std::vector<int> &commandList, double frequency, double amplitude)
{
    int i, period, value;
    double t;

    commandList.clear();    // if command list already exists, erase it and start a new one

    if (amplitude < 0.0 || amplitude > 128.0) {
        std::cerr << "Error in Rhd2000Registers::createCommandListZcheckDac: Amplitude out of range." << std::endl;
        return -1;
    }
    if (frequency < 0.0) {
        std::cerr << "Error in Rhd2000Registers::createCommandListZcheckDac: Negative frequency not allowed." << std::endl;
        return -1;
    } else if (frequency > sampleRate / 4.0) {
        std::cerr << "Error in Rhd2000Registers::createCommandListZcheckDac: " <<
                "Frequency too high relative to sampling rate." << std::endl;
        return -1;
    }
    if (frequency == 0.0) {
        for (i = 0; i < MaxCommandLength; ++i) {
            commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 6, 128));
        }
    } else {
        period = (int) floor(sampleRate / frequency + 0.5);
        if (period > MaxCommandLength) {
            std::cerr << "Error in Rhd2000Registers::createCommandListZcheckDac: Frequency too low." << std::endl;
            return -1;
        } else {
            t = 0.0;
            for (i = 0; i < period; ++i) {
                value = static_cast<int>(floor(amplitude * sin(2 * std::numbers::pi * frequency * t) + 128.0 + 0.5));
                if (value < 0) {
                    value = 0;
                } else if (value > 255) {
                    value = 255;
                }
                commandList.push_back(createRhd2000Command(Rhd2000CommandRegWrite, 6, value));
                t += 1.0 / sampleRate;
            }
        }
    }

    return commandList.size();
}
