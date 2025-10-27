module Components {
    state machine ImuStateMachine {
        initial enter RESET

        signal tick
        signal success

        action doReset
        action doWaitReset
        action doEnable
        action doConfigureDevice
        action doRead

        state RESET {
            on tick do { doReset }
            on success enter WAIT_RESET
        }

        state WAIT_RESET {
            on tick do { doWaitReset }
            on success enter WAKE_UP
        }

        state WAKE_UP {
            on tick do { doEnable }
            on success enter CONFIGURE
        }

        state CONFIGURE {
            on tick do { doConfigureDevice }
            on success enter READY
        }

        state READY {
            on tick do { doRead }
            on success enter READY
        }
    }
}