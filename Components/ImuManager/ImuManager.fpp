module Components {
    @ IMU manager component
    queued component ImuManager {
        @ Range of the accelerometer in G's, integer values represent the conversion factor for the raw values from
        @ the accelerometer registers to Gs
        enum AccelerationRange : U16 {
            RANGE_2G = 16384 
            RANGE_4G = 8192
            RANGE_8G = 4096
            RANGE_16G = 2048 
        }

        @ Range of the gyroscope in degrees per second, integer values represent the conversion factor for the raw values from
        @ the gyroscope registers to 10ths of a degree per second
        enum GyroscopeRange : U16 {
            RANGE_250DEG = 1310
            RANGE_500DEG = 655
            RANGE_1000DEG = 328
            RANGE_2000DEG = 164 
        }

        state machine instance imuStateMachine: ImuStateMachine

        @ Input rate group port
        sync input port run: Svc.Sched

        output port busWrite: Drv.I2c

        output port busRead: Drv.I2c

        output port busWriteRead: Drv.I2cWriteRead

        param ACCELEROMETER_RANGE: AccelerationRange default AccelerationRange.RANGE_2G
        param GYROSCOPE_RANGE: GyroscopeRange default GyroscopeRange.RANGE_250DEG

        telemetry ACCEL_DATA_X: I16
        telemetry ACCEL_DATA_Y: I16
        telemetry ACCEL_DATA_Z: I16
        telemetry GYRO_DATA_X: I16
        telemetry GYRO_DATA_Y: I16
        telemetry GYRO_DATA_Z: I16
        telemetry TEMP_DATA: I16

        event StartStateMachine severity activity low format "Starting IMU State Machine"
        event ResetValue(value: U8) severity activity low format "Reset register: 0x{x}"
        event WakeupEvent severity activity low format "Waking up IMU"
        event ConfigureEvent severity activity low format "Configuring IMU"
        event ReadyEvent severity activity low format "Reading telemetry from IMU"

        @ Struct representing X, Y, Z data
        struct GeometricVector3 {
            x: F32 @< X component of the vector
            y: F32 @< Y component of the vector
            z: F32 @< Z component of the vector
        }
        @ Struct representing ImuData
        struct ImuData {
            @ Accelerations from the accelerometer
            acceleration: GeometricVector3
            
            @ Angular rates from the gyroscope
            rotation: GeometricVector3

            @ Temperature in degrees Celsius
            temperature: F32
        }


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