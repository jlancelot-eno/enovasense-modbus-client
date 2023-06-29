#include <iostream>

#include "src/enovasensemodbusclient.h"

using namespace std;

int main()
{
    cout << "Hello World!" << endl;

    string IP = "192.168.0.100";
    int port = 502;
    EnovasenseModbusClient myclient(IP,port);
    //uint16_t tab_reg[60];
    int res = myclient.getCalibration();
    int slckVal = myclient.isSoftwareLockEnabled();
    slckVal = myclient.setSoftwareLockEnabled(true);
    int errStart = myclient.startMeasurement();
    int ampVal = myclient.getLaserAmplitude();
    int threshVal = myclient.getLaserThreshold();
    int coolingVal = myclient.getCoolingTime();
    int heatingVal = myclient.getHeatingTime();
    int measuringVal = myclient.getMeasuringTime();
    int frequenciesNum = myclient.getFrequenciesNumber();
    int outputsNum = myclient.getOutputsNumber();
    std::vector<uint16_t> dataVector = myclient.getRawData();

    std::cout << "calibration value is " << res << std::endl;
    std::cout << "measurement start code error is " << errStart << std::endl;
    std::cout << "SLCK value is " << slckVal << std::endl;
    std::cout << "amplitude value is " << ampVal << std::endl;
    std::cout << "threshold value is " << threshVal << std::endl;
    std::cout << "cooling value is " << coolingVal << std::endl;
    std::cout << "heating value is " << heatingVal << std::endl;
    std::cout << "measuring value is " << measuringVal << std::endl;
    std::cout << "frequencies Number value is " << frequenciesNum << std::endl;
    std::cout << "outputs Number value is " << outputsNum << std::endl;
    std::cout << "is ready " << myclient.isReady() << std::endl;
    std::cout << "is calibrated " << myclient.isCalibrationSetSuccessfully() << std::endl;
    std::cout << "is measuring " << myclient.isMeasuring() << std::endl;
    std::cout << "is laser unlocked " << myclient.isLaserUnlocked() << std::endl;
    std::cout << "is laser on " << myclient.isLaserOn() << std::endl;
    std::cout << "is acquisition buffer empty " << myclient.isAcquisitionBufferEmpty() << std::endl;
    std::cout << "is module external locked " << myclient.isModuleExternalLock() << std::endl;
    std::cout << "used frequency " << myclient.getFrequencyAt(0) << std::endl;
    std::cout << "used output " << myclient.getOutputAt(0) << std::endl;

    if (res == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return res;
    }
    for (unsigned int i=0; i < dataVector.size(); i++) {
        printf("dataVector[%d]=%d (0x%X)\n", i, dataVector[i], dataVector[i]);
    }

    return res;
}
