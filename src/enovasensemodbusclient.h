#ifndef ENOVASENSEMODBUSCLIENT_H
#define ENOVASENSEMODBUSCLIENT_H

#include <modbus.h>

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>

class EnovasenseModbusClient
{
public:
    // Client constructor and destructor
    EnovasenseModbusClient();
    EnovasenseModbusClient(std::string IP, int port = 502);
    ~EnovasenseModbusClient();

    // Modbus ON/OFF parameters
    void close();
    int open(std::string IP, int port = 502);

    // Get the last returned error
    int lastErrorCode() const;
    std::string lastErrorString() const;

    // Enovasense module control commands
    int getCalibration();
    int setCalibration(uint16_t value);
    int startMeasurement();
    int isSoftwareLockEnabled();
    int setSoftwareLockEnabled(bool enabled);

    // Read current settings and status on the Enovasense module
    int getLaserAmplitude();
    int getLaserThreshold();
    int getCoolingTime();
    int getHeatingTime();
    int getMeasuringTime();
    int getFrequenciesNumber();
    int getOutputsNumber();

    // Status word (STAT)
    // Every method below returns:
    //      -1 on read error,
    //       0 if the bit is cleared
    //       1 if the bit is set
    int getStatusBit(int index);
    int isReady();
    int isMeasuring();
    int isCalibrationSetSuccessfully();
    int isLaserUnlocked();
    int isLaserOn();
    int isAcquisitionBufferEmpty();
    int isModuleExternalLock();
    std::vector<uint16_t> getAllFrequencies();
    int getFrequencyAt(int number);

    // Read last measures - Calibration outputs
    std::vector<uint16_t> getAllOutputs();
    int getOutputAt(int index);

    // Read last measures - Raw data
    std::vector<uint16_t> getRawData();

private:
    // Private methods used within the current class. The user isn't supposed to call them
    int32_t _writeReg(uint32_t addr, uint16_t* value, uint32_t size);
    int32_t _readReg(uint32_t addr, uint16_t* value, uint32_t size);
    int _setCalibrationTrigger(uint16_t value);
    int _setMeasurementTrigger(uint16_t value);

    // Member variables of the current class
    modbus_t* m_ctx;
    bool m_connected;
    std::string m_IP;
    int m_port;
};


#endif // ENOVASENSEMODBUSCLIENT_H
