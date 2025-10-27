module Components {
    @ IMU manager component
    queued component ImuManager {

        state machine instance imuStateMachine: ImuStateMachine

        @ Input rate group port
        sync input port run: Svc.Sched

        output port busWrite: Drv.I2c

        output port busRead: Drv.I2c

        output port busWriteRead: Drv.I2cWriteRead

        param ACCELEROMETER_RANGE: U8
        param GYROSCOPE_RANGE: U8

        telemetry ACCEL_DATA_X: I16
        telemetry ACCEL_DATA_Y: I16
        telemetry ACCEL_DATA_Z: I16
        telemetry GYRO_DATA_X: I16
        telemetry GYRO_DATA_Y: I16
        telemetry GYRO_DATA_Z: I16
        telemetry TEMP_DATA: I16

        event StartStateMachine severity activity low format "Starting IMU State Machine"
        event ResetValue(value: U8) severity activity low format "Reset register: 0x{x}"

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Port to return the value of a parameter
        param get port prmGetOut

        @Port to set the value of a parameter
        param set port prmSetOut

    }
}