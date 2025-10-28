// ======================================================================
// \title  ImuManager.hpp
// \author pi
// \brief  hpp file for ImuManager component implementation class
// ======================================================================

#ifndef Components_ImuManager_HPP
#define Components_ImuManager_HPP

#include "Components/ImuManager/ImuManagerComponentAc.hpp"
#include "Components/ImuManager/ImuManager_ImuDataSerializableAc.hpp"
#include "Components/ImuManager/ImuTypes.hpp"
#include "Components/ImuManager/ImuManager_GyroscopeRangeEnumAc.hpp"

#include "Components/ImuManager/ImuManager_AccelerationRangeEnumAc.hpp"

namespace Components {

class ImuManager final : public ImuManagerComponentBase {

public:
  // ----------------------------------------------------------------------
  // Component construction and destruction
  // ----------------------------------------------------------------------

  //! Construct ImuManager object
  ImuManager(const char *const compName //!< The component name
  );

  //! Destroy ImuManager object
  ~ImuManager();

private:
  // ----------------------------------------------------------------------
  // Handler implementations for typed input ports
  // ----------------------------------------------------------------------

  //! Handler implementation for run
  //!
  //! Input rate group port
  void run_handler(FwIndexType portNum, //!< The port number
                   U32 context          //!< The call order
                   ) override;

private:

    //! Write to the I2C bus and handle errors
    Drv::I2cStatus bus_write(Fw::Buffer& writeBuffer, Fw::Buffer& readBuffer);

    //! Resets the IMU
    Drv::I2cStatus reset();

    //! Read the reset register value
    Drv::I2cStatus read_reset(U8& value);

    //! Enable on the IMU
    Drv::I2cStatus enable();

    //! Configure the IMU's accelerometer and gyroscope
    Drv::I2cStatus configure_device();

    //! Convert enum to register value
    U8 accelerometer_range_to_register(ImuManager_AccelerationRange range);

    //! Convert enum to register value
    U8 gyroscope_range_to_register(ImuManager_GyroscopeRange range);

    //! Read IMU data
    Drv::I2cStatus read(ImuManager_ImuData& imuData);

    //! Deserialize raw IMU data
    MpuImu::RawImuData deserialize_raw_data(Fw::Buffer& buffer);

    //! Convert raw IMU data
    ImuManager_ImuData convert_raw_data(const MpuImu::RawImuData& raw,
                                      const ImuManager_AccelerationRange& accelerationRange,
                                      const ImuManager_GyroscopeRange& gyroscopeRange);

private:
  // ----------------------------------------------------------------------
  // Implementations for internal state machine actions
  // ----------------------------------------------------------------------

  //! Implementation for action doReset of state machine
  //! Components_ImuStateMachine
  void Components_ImuStateMachine_action_doReset(
      SmId smId,                                //!< The state machine id
      Components_ImuStateMachine::Signal signal //!< The signal
      ) override;

  //! Implementation for action doWaitReset of state machine
  //! Components_ImuStateMachine
  void Components_ImuStateMachine_action_doWaitReset(
      SmId smId,                                //!< The state machine id
      Components_ImuStateMachine::Signal signal //!< The signal
      ) override;

  //! Implementation for action doEnable of state machine
  //! Components_ImuStateMachine
  void Components_ImuStateMachine_action_doEnable(
      SmId smId,                                //!< The state machine id
      Components_ImuStateMachine::Signal signal //!< The signal
      ) override;

  //! Implementation for action doConfigureDevice of state machine
  //! Components_ImuStateMachine
  void Components_ImuStateMachine_action_doConfigureDevice(
      SmId smId,                                //!< The state machine id
      Components_ImuStateMachine::Signal signal //!< The signal
      ) override;

  //! Implementation for action doRead of state machine
  //! Components_ImuStateMachine
  void Components_ImuStateMachine_action_doRead(
      SmId smId,                                //!< The state machine id
      Components_ImuStateMachine::Signal signal //!< The signal
      ) override;

private:
  U8 m_address;
};

} // namespace Components

#endif
