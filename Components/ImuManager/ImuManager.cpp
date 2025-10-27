// ======================================================================
// \title  ImuManager.cpp
// \author pi
// \brief  cpp file for ImuManager component implementation class
// ======================================================================

#include "Components/ImuManager/ImuManager.hpp"

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
  // TODO
  printf("inside ENABLE\n");
}

void ImuManager ::Components_ImuStateMachine_action_doConfigureDevice(
    SmId smId, Components_ImuStateMachine::Signal signal) {
  // TODO
}

void ImuManager ::Components_ImuStateMachine_action_doRead(
    SmId smId, Components_ImuStateMachine::Signal signal) {
  // TODO
}

} // namespace Components
