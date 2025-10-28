// ======================================================================
// \title  ImuManager.cpp
// \author pi
// \brief  cpp file for ImuManager component implementation class
// ======================================================================

#include "Components/ImuManager/ImuManager.hpp"
#include "Components/ImuManager/ImuManager_AccelerationRangeEnumAc.hpp"
#include "Components/ImuManager/ImuManager_GyroscopeRangeEnumAc.hpp"

namespace Components {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

ImuManager ::ImuManager(const char *const compName)
    : ImuManagerComponentBase(compName), m_address(MpuImu::DEVICE_DEFAULT_ADDRESS) {}

ImuManager ::~ImuManager() {}

Drv::I2cStatus ImuManager ::bus_write(Fw::Buffer& writeBuffer, Fw::Buffer& readBuffer) {
    Drv::I2cStatus status;
    FW_ASSERT(writeBuffer.isValid());
    if (readBuffer.isValid()) {
        status = this->busWriteRead_out(0, this->m_address, writeBuffer, readBuffer);
    } else {
        status = this->busWrite_out(0, this->m_address, writeBuffer);
    }
    return status;
}

Drv::I2cStatus ImuManager ::reset() {
    // Attempt to write the reset data
    U8 reset_sequence[] = {MpuImu::POWER_MGMT_REGISTER, MpuImu::RESET_VALUE};
    Fw::Buffer writeBuffer(reset_sequence, sizeof(reset_sequence));
    Fw::Buffer readBuffer;
    return this->bus_write(writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManager ::read_reset(U8& value) {
    U8 registerAddress = MpuImu::POWER_MGMT_REGISTER;
    Fw::Buffer writeBuffer(&registerAddress, sizeof(registerAddress));
    Fw::Buffer readBuffer(&value, sizeof(value));
    return this->bus_write(writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManager ::enable() {
    U8 power_on_sequence[] = {MpuImu::POWER_MGMT_REGISTER, MpuImu::POWER_ON_VALUE};
    Fw::Buffer writeBuffer(power_on_sequence, sizeof(power_on_sequence));
    Fw::Buffer readBuffer;
    return this->bus_write(writeBuffer, readBuffer);
}

U8 ImuManager ::accelerometer_range_to_register(ImuManager_AccelerationRange range) {
    U8 registerValue = 0;
    switch (range.e) {
        case ImuManager_AccelerationRange::RANGE_2G:
            registerValue = MpuImu::ACCEL_CONFIG_2G;
            break;
        case ImuManager_AccelerationRange::RANGE_4G:
            registerValue = MpuImu::ACCEL_CONFIG_4G;
            break;
        case ImuManager_AccelerationRange::RANGE_8G:
            registerValue = MpuImu::ACCEL_CONFIG_8G;
            break;
        case ImuManager_AccelerationRange::RANGE_16G:
            registerValue = MpuImu::ACCEL_CONFIG_16G;
            break;
        default:
            FW_ASSERT(0, range.e);
            break;
    }
    return registerValue;
}

U8 ImuManager ::gyroscope_range_to_register(ImuManager_GyroscopeRange range) {
    U8 registerValue = 0;
    switch (range.e) {
        case ImuManager_GyroscopeRange::RANGE_250DEG:
            registerValue = MpuImu::GYRO_CONFIG_250DEG;
            break;
        case ImuManager_GyroscopeRange::RANGE_500DEG:
            registerValue = MpuImu::GYRO_CONFIG_500DEG;
            break;
        case ImuManager_GyroscopeRange::RANGE_1000DEG:
            registerValue = MpuImu::GYRO_CONFIG_1000DEG;
            break;
        case ImuManager_GyroscopeRange::RANGE_2000DEG:
            registerValue = MpuImu::GYRO_CONFIG_2000DEG;
            break;
        default:
            FW_ASSERT(0, range.e);
            break;
    }
    return registerValue;
}

Drv::I2cStatus ImuManager ::configure_device() {
    Fw::ParamValid paramValid;
    Drv::I2cStatus status = Drv::I2cStatus::I2C_OK;
    // Read accelerometer parameter and configure
    {
        const ImuManager_AccelerationRange accelerationRange = static_cast<ImuManager_AccelerationRange::T>(this->paramGet_ACCELEROMETER_RANGE(paramValid));
        FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));

        U8 accel_config_sequence[] = {MpuImu::ACCEL_CONFIG_REGISTER, this->accelerometer_range_to_register(accelerationRange)};
        Fw::Buffer writeBuffer(accel_config_sequence, sizeof(accel_config_sequence));
        Fw::Buffer readBuffer;
        status = this->bus_write(writeBuffer, readBuffer);
        if (status != Drv::I2cStatus::I2C_OK) {
            return status;
        }
    }
    // Read gyroscope parameter and configure
    {
        const ImuManager_GyroscopeRange gyroscopeRange = static_cast<ImuManager_GyroscopeRange::T>(this->paramGet_GYROSCOPE_RANGE(paramValid));
        FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));
        U8 gyro_config_sequence[] = {MpuImu::GYRO_CONFIG_REGISTER, this->gyroscope_range_to_register(gyroscopeRange)};
        Fw::Buffer writeBuffer(gyro_config_sequence, sizeof(gyro_config_sequence));
        Fw::Buffer readBuffer;
        status = this->bus_write(writeBuffer, readBuffer);
        if (status != Drv::I2cStatus::I2C_OK) {
            return status;
        }
    }
    return status;
}


Drv::I2cStatus ImuManager ::read(ImuManager_ImuData& imuData) {
    U8 data[MpuImu::DATA_LENGTH];
    U8 registerAddress = MpuImu::DATA_BASE_REGISTER;

    Fw::Buffer writeBuffer(&registerAddress, 1);
    Fw::Buffer readBuffer(data, MpuImu::DATA_LENGTH);
    // If bus write fails, state machine is reset, so just return
    Drv::I2cStatus status = this->bus_write(writeBuffer, readBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        return status;
    }
    MpuImu::RawImuData raw = this->deserialize_raw_data(readBuffer);

    // This code will read the currently scaled parameters
    Fw::ParamValid paramValid;
    const ImuManager_AccelerationRange accelerationRange = static_cast<ImuManager_AccelerationRange::T>(this->paramGet_ACCELEROMETER_RANGE(paramValid));
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));
    const ImuManager_GyroscopeRange gyroscopeRange = static_cast<ImuManager_GyroscopeRange::T>(this->paramGet_GYROSCOPE_RANGE(paramValid));
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));

    imuData = this->convert_raw_data(raw, accelerationRange, gyroscopeRange);
    return status;
}

MpuImu::RawImuData ImuManager ::deserialize_raw_data(Fw::Buffer& buffer) {
    auto deserializer = buffer.getDeserializer();
    MpuImu::RawImuData raw;
    deserializer.deserialize(raw.acceleration[0]);
    deserializer.deserialize(raw.acceleration[1]);
    deserializer.deserialize(raw.acceleration[2]);
    deserializer.deserialize(raw.temperature);
    deserializer.deserialize(raw.gyroscope[0]);
    deserializer.deserialize(raw.gyroscope[1]);
    deserializer.deserialize(raw.gyroscope[2]);
    return raw;
}

ImuManager_ImuData ImuManager ::convert_raw_data(const MpuImu::RawImuData& raw,
                                      const ImuManager_AccelerationRange& accelerationRange,
                                      const ImuManager_GyroscopeRange& gyroscopeRange) {
    // Set the values of the IMU data by multiplying by conversion factors
    ImuManager_ImuData imuData;
    imuData.get_acceleration().set_x(static_cast<F32>(raw.acceleration[0]) * 1.0f /
                                     static_cast<F32>(accelerationRange));
    imuData.get_acceleration().set_y(static_cast<F32>(raw.acceleration[1]) * 1.0f /
                                     static_cast<F32>(accelerationRange));
    imuData.get_acceleration().set_z(static_cast<F32>(raw.acceleration[2]) * 1.0f /
                                     static_cast<F32>(accelerationRange));
    imuData.set_temperature((static_cast<F32>(raw.temperature) / MpuImu::TEMPERATURE_SCALAR) + MpuImu::TEMPERATURE_OFFSET);
    imuData.get_rotation().set_x(static_cast<F32>(raw.gyroscope[0]) * 10.0f / static_cast<F32>(gyroscopeRange));
    imuData.get_rotation().set_y(static_cast<F32>(raw.gyroscope[1]) * 10.0f / static_cast<F32>(gyroscopeRange));
    imuData.get_rotation().set_z(static_cast<F32>(raw.gyroscope[2]) * 10.0f / static_cast<F32>(gyroscopeRange));
    return imuData;
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void ImuManager ::run_handler(FwIndexType portNum, U32 context) {
  this->imuStateMachine_sendSignal_tick();
  this->dispatchCurrentMessages();
}

// ----------------------------------------------------------------------
// Implementations for internal state machine actions
// ----------------------------------------------------------------------

void ImuManager ::Components_ImuStateMachine_action_doReset(
    SmId smId, Components_ImuStateMachine::Signal signal) {
  this->log_ACTIVITY_LO_StartStateMachine();
  this->reset();
  this->imuStateMachine_sendSignal_success();
}

void ImuManager ::Components_ImuStateMachine_action_doWaitReset(
    SmId smId, Components_ImuStateMachine::Signal signal) {
  U8 value;
  auto status = this->read_reset(value);
  FW_ASSERT(status == Drv::I2cStatus::I2C_OK);
  this->log_ACTIVITY_LO_ResetValue(value);
  if ((value & MpuImu::RESET_VALUE) == 0) {
    this->imuStateMachine_sendSignal_success();
  }
}

void ImuManager ::Components_ImuStateMachine_action_doEnable(
    SmId smId, Components_ImuStateMachine::Signal signal) {
  auto status = this->enable();
  FW_ASSERT(status == Drv::I2cStatus::I2C_OK);
  this->log_ACTIVITY_LO_WakeupEvent();
  this->imuStateMachine_sendSignal_success();
}

void ImuManager ::Components_ImuStateMachine_action_doConfigureDevice(
    SmId smId, Components_ImuStateMachine::Signal signal) {
  auto status = this->configure_device();
  FW_ASSERT(status == Drv::I2cStatus::I2C_OK);
  this->log_ACTIVITY_LO_ConfigureEvent();
  this->imuStateMachine_sendSignal_success();
}

void ImuManager ::Components_ImuStateMachine_action_doRead(
    SmId smId, Components_ImuStateMachine::Signal signal) {
  
    ImuManager_ImuData imu_data;
    auto status = this->read(imu_data);
    FW_ASSERT(Drv::I2cStatus::I2C_OK == status);
    this->log_ACTIVITY_LO_ReadyEvent();
    ImuManager_GeometricVector3 accel_data = imu_data.get_acceleration();
    this->tlmWrite_ACCEL_DATA_X(accel_data.get_x());
    this->tlmWrite_ACCEL_DATA_Y(accel_data.get_y());
    this->tlmWrite_ACCEL_DATA_Z(accel_data.get_z());
    ImuManager_GeometricVector3 gyro_data = imu_data.get_rotation();
    this->tlmWrite_GYRO_DATA_X(gyro_data.get_x());
    this->tlmWrite_GYRO_DATA_Y(gyro_data.get_y());
    this->tlmWrite_GYRO_DATA_Z(gyro_data.get_z());
    this->tlmWrite_TEMP_DATA(imu_data.get_temperature());
  }

} // namespace Components
