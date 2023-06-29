#include "enovasensemodbusclient.h"

namespace EnovasenseAddressMapping
{
// MACRO FOR THE ADDRESSES
uint16_t SCAL = 0;
uint16_t CTRG = 1;
uint16_t MTRG = 2;
uint16_t SLCK = 3;
uint16_t CCAL = 10;
uint16_t VAMP = 11;
uint16_t VTHR = 12;
uint16_t COOL = 13;
uint16_t HEAT = 14;
uint16_t MEAS = 15;
uint16_t NFRQ = 16;
uint16_t NOUT = 17;
uint16_t STAT = 18;
uint16_t FRQ0 = 20;
uint16_t FRQ1 = 21;
uint16_t FRQ2 = 22;
uint16_t FRQ3 = 23;
uint16_t FRQ4 = 24;
uint16_t FRQ5 = 25;
uint16_t FRQ6 = 26;
uint16_t FRQ7 = 27;
uint16_t FRQ8 = 28;
uint16_t FRQ9 = 29;
uint16_t OUT0 = 30;
uint16_t OUT1 = 31;
uint16_t OUT2 = 32;
uint16_t OUT3 = 33;
uint16_t OUT4 = 34;
uint16_t OUT5 = 35;
uint16_t OUT6 = 36;
uint16_t OUT7 = 37;
uint16_t OUT8 = 38;
uint16_t OUT9 = 39;
uint16_t PHI0 = 40;
uint16_t AMP0 = 41;
uint16_t PHI1 = 42;
uint16_t AMP1 = 43;
uint16_t PHI2 = 44;
uint16_t AMP2 = 45;

// different bit values contained in STAT address
enum StatusBits {
    Ready = 0,
    Measuring,
    CalibrationSetSuccessfully,
    SoftwareUnlocked,
    LaserOn,
    AcquisitionBufferEmpty,
    ModuleExternalLock
};

// MACRO of error codes used in the SDK
enum ENO_ERRCODE {
    ENO_INVALIDCALIBRATION = -4,
    ENO_NOTREADY,
    ENO_TIMEOUT,
    ENO_NOTMEASURING,
    ENO_NOERROR
};
}

namespace EAM = EnovasenseAddressMapping;

// EnovasenseModbusClient's constructor without arguments
EnovasenseModbusClient::EnovasenseModbusClient() :
    m_ctx(nullptr),
    m_connected(false)
{

}

// EnovasenseModbusClient's constructor with arguments
EnovasenseModbusClient::EnovasenseModbusClient(std::string IP, int port) :
    m_ctx(nullptr),
    m_connected(false)
{
    open(IP, port);
}

// EnovasenseModbusClient's destructor
EnovasenseModbusClient::~EnovasenseModbusClient()
{
    close();
}

// close() method is called in the destructor. Closes the connection to the connected Enovasense module.
void EnovasenseModbusClient::close()
{
    if (m_ctx != nullptr) {
        if (m_connected) {
            modbus_flush(m_ctx);
            modbus_close(m_ctx);
            m_connected = false;
        }
        modbus_free(m_ctx);
        m_ctx = nullptr;
    }
}

// Opens the connection once the TCP socket has been set with the server
// Arguments: stdd::string IP : IP address of the connected Enovasense module,
//           int port        : port of the Server running on the Enovasense module.
int EnovasenseModbusClient::open(std::string IP, int port)
{
    m_IP = IP;
    m_port = port;
    m_ctx = modbus_new_tcp(IP.c_str(), port);

    if (m_ctx == nullptr) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        return -1;
    }

    int ret = modbus_connect(m_ctx);
    if (ret==-1) {
        m_connected = false;
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(m_ctx);
        m_ctx = nullptr;
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    m_connected = true;
    modbus_flush(m_ctx);
    return ret;
}

int EnovasenseModbusClient::lastErrorCode() const
{
    return errno;
}

std::string EnovasenseModbusClient::lastErrorString() const
{
    return std::string(modbus_strerror(errno));
}

// Writes the "value" set in the arguments to the register in the chosen address with the size it can take.
int32_t EnovasenseModbusClient::_writeReg(uint32_t addr, uint16_t* value, uint32_t size)
{
    int res = modbus_write_registers(m_ctx, addr, size, value);
    if (res == -1)
    {
        fprintf(stderr, "Writing failed: %s\n", modbus_strerror(errno));
    }
    return res;
}

// Reads the value contained in the register in the chosen address with the size it can take and store the result in the variable "value".
int32_t EnovasenseModbusClient::_readReg(uint32_t addr, uint16_t* value, uint32_t size)
{
    int32_t res = modbus_read_registers(m_ctx, addr, size, value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res;
}

// Gets the current calibration being used in CCAL
int EnovasenseModbusClient::getCalibration()
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::CCAL, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Sets the given calibration in the variable "value" to be used in SCAL
// Arguments: uint16_t value : calibration value (a 16 bit integer set between 1 and 999)
int EnovasenseModbusClient::setCalibration(uint16_t value)
{
    if (value < 1 || value > 999) {
        return EAM::ENO_INVALIDCALIBRATION;
    }
    int calibrationRes = modbus_write_registers(m_ctx,EAM::SCAL, 1, &value);
    if (calibrationRes==-1)
    {
        //the value given in the input is not between 1 and 999
        fprintf(stderr, "%d\n", EAM::ENO_INVALIDCALIBRATION);
        return EAM::ENO_INVALIDCALIBRATION;
    }
    _setCalibrationTrigger(0xFF);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    _setCalibrationTrigger(0x00);
    if (getStatusBit(EAM::CalibrationSetSuccessfully) && getCalibration() != value)
    {
        //coudn't find any register for this calibration number
        fprintf(stderr, "%d\n", EAM::ENO_TIMEOUT);
        return EAM::ENO_TIMEOUT;
    }
    return EAM::ENO_NOERROR;
}

// Trigger to activate the calibration set in setCalibration().
// Arguments: uint16_t value : a 16 bit integer that can take 2 values: 0x00 to desable the trigger and 0xFF to enable the trigger.
int EnovasenseModbusClient::_setCalibrationTrigger(uint16_t value)
{
    int32_t res = modbus_write_registers(m_ctx,EAM::CTRG, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Starts the Measurement when called.
int EnovasenseModbusClient::startMeasurement()
{
    int readyRes = isReady();
    if (readyRes==-1)
    {
        fprintf(stderr, "%d\n", EAM::ENO_NOTREADY);
        return EAM::ENO_NOTREADY;
    }
    _setMeasurementTrigger(0xFF);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int measureActive = isMeasuring();
    if (measureActive == -1)
    {
        fprintf(stderr, "%d\n", EAM::ENO_NOTMEASURING);
        _setMeasurementTrigger(0x00);
        return EAM::ENO_NOTMEASURING;
    }
    _setMeasurementTrigger(0x00);
    return EAM::ENO_NOERROR;
}

// Trigger to set the measurement set in startMeasurement().
// Arguments: uint16_t value : a 16 bit integer that can take 2 values: 0x00 to desable the trigger and 0xFF to enable the trigger.
int EnovasenseModbusClient::_setMeasurementTrigger(uint16_t value)
{
    int32_t res = modbus_write_registers(m_ctx,EAM::MTRG, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Checks if the Software Lock is enabled.
int EnovasenseModbusClient::isSoftwareLockEnabled()
{
    return getStatusBit(EAM::SoftwareUnlocked);
}

// Sets the Software Lock to 0x00 if false or to 0xFF if true.
int EnovasenseModbusClient::setSoftwareLockEnabled(bool enabled)
{
    uint16_t value = enabled ? 0xFF : 0x00;
    int32_t res = modbus_write_registers(m_ctx,EAM::SLCK, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Reads the current amplitude value of the laser used for the measure.
int EnovasenseModbusClient::getLaserAmplitude()
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::VAMP, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Reads the laser threshold value used in the measurement.
int EnovasenseModbusClient::getLaserThreshold()
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::VTHR, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Reads the colling time of the laser used in the measurement
int EnovasenseModbusClient::getCoolingTime()
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::COOL, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Reads the heating time of the laser used in the measurement.
int EnovasenseModbusClient::getHeatingTime()
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::HEAT, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Reads the measurement duration in milliseconds.
int EnovasenseModbusClient::getMeasuringTime()
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::MEAS, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Reads the number of frequencies used in the measurement.
int EnovasenseModbusClient::getFrequenciesNumber()
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::NFRQ, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Reads the number of outputs returned after the measurement.
int EnovasenseModbusClient::getOutputsNumber()
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::NOUT, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Reads the value of the Bit in the address STAT at the index "index".
// Arguments: int index: bit index of the STAT address
int EnovasenseModbusClient::getStatusBit(int index)
{
    uint16_t value;
    int32_t res = modbus_read_registers(m_ctx,EAM::STAT, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }
    return value & (1 << index) ? 1 : 0;
}

// Reads the value of the Bit in the address STAT at the index EAM::Ready defined in the namespace EnovasenseAddressMapping.
int EnovasenseModbusClient::isReady()
{
    return getStatusBit(EAM::Ready);
}

// Reads the value of the Bit in the address STAT at the index EAM::Measuring defined in the namespace EnovasenseAddressMapping.
int EnovasenseModbusClient::isMeasuring()
{
    return getStatusBit(EAM::Measuring);
}

// Reads the value of the Bit in the address STAT at the index EAM::CalibrationSetSuccessfully defined in the namespace EnovasenseAddressMapping.
int EnovasenseModbusClient::isCalibrationSetSuccessfully()
{
    return getStatusBit(EAM::CalibrationSetSuccessfully);
}

// Reads the value of the Bit in the address STAT at the index EAM::SoftwareUnlocked defined in the namespace EnovasenseAddressMapping.
int EnovasenseModbusClient::isLaserUnlocked()
{
    return getStatusBit(EAM::SoftwareUnlocked);
}

// Reads the value of the Bit in the address STAT at the index EAM::LaserOn defined in the namespace EnovasenseAddressMapping.
int EnovasenseModbusClient::isLaserOn()
{
    return getStatusBit(EAM::LaserOn);
}

// Reads the value of the Bit in the address STAT at the index EAM::AcquisitionBufferEmpty defined in the namespace EnovasenseAddressMapping.
int EnovasenseModbusClient::isAcquisitionBufferEmpty()
{
    return getStatusBit(EAM::AcquisitionBufferEmpty);
}

// Reads the value of the Bit in the address STAT at the index EAM::ModuleExternalLock defined in the namespace EnovasenseAddressMapping.
int EnovasenseModbusClient::isModuleExternalLock()
{
    return getStatusBit(EAM::ModuleExternalLock);
}

// Returns the vector of all frequencies used during the measurement.
std::vector<uint16_t> EnovasenseModbusClient::getAllFrequencies()
{
    int size = getFrequenciesNumber();
    std::vector<uint16_t> freqs(size);
    int32_t res = modbus_read_registers(m_ctx, EAM::FRQ0, size, freqs.data());
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return freqs;
}

// Reads the frequency at the index "index" corresponding to the X value of the addresses FRQX.
int EnovasenseModbusClient::getFrequencyAt(int index)
{
    uint16_t value;
    index = EAM::FRQ0 + index;
    int32_t res = modbus_read_registers(m_ctx, index, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Returns the vector containing all the outputs filled during the measurement.
std::vector<uint16_t> EnovasenseModbusClient::getAllOutputs()
{
    int size = getOutputsNumber();
    std::vector<uint16_t> outputs(size);
    int32_t res = modbus_read_registers(m_ctx, EAM::OUT0, size, outputs.data());
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return outputs;
}

// Reads the value contained in the output at the index "index" corresponding to the X value of the addresses OUTX.
int EnovasenseModbusClient::getOutputAt(int index)
{
    uint16_t value;
    index = EAM::OUT0 + index;
    int32_t res = modbus_read_registers(m_ctx, index, 1, &value);
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return res < 0 ? -1 : value;
}

// Returns the vector of amplitudes and phases filled during the measurement.
std::vector<uint16_t> EnovasenseModbusClient::getRawData()
{
    int size = 2 * getFrequenciesNumber();
    std::vector<uint16_t> rawData(size);
    int32_t res = modbus_read_registers(m_ctx, EAM::PHI0, size, rawData.data());
    if (res == -1)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
    }
    return rawData;
}
