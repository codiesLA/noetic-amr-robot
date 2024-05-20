# Driving No.
MotionNum_Min = 0  # Dec:6144, Hex:0x1800
MotionNum_Default = 0
MotionNum_Max = 255  # Dec:22464, Hex:0x57c0 255*64+6144
MotionNumAdress_Min = 6144
MotionNumAdress_Max = 22464
MotionNumAdress_Pitch = 64

# Method (base address +0,1) default: 2
# based on manual HM-60262-6E page 59
ControlMethod = {
    "AbsolutePosition": 1,  # 1:Absolute positioning
    "RelativePositionBasedOnOrderPosition": 2,  # 2: Incremental positioning (based on command position)
    "RelativePositionBasedOnDetectPosition": 3,  # 3:Incremental positioning (based on feedback position)
    "ContinusOperation": 7,  # 7: Continuous operation (Position control)
    "RoundAbsolutePositionOrder": 8,  # 8: Wrap absolute positioning
    "RoundNearPositionOrder": 9,  # 9: Wrap proximity positioning
    "RoundForwardAbsolutePositionOrder": 10,  # 10: Wrap forward direction absolute positioning
    "RoundForwardReversePositionOrder": 11,  # 11: Wrap reverse direction absolute positioning
    "RoundAbsolutePress": 12,  # 12: Wrap absolute push-motion
    "RoundNearPress": 13,  # 13: Wrap proximity push-motion
    "RoundForwardPress": 14,  # 14: Wrap forward direction push-motion
    "RoundReversePress": 15,  # 15: Wrap reverse direction push-motion
    "ContinusOperationWithSpeed": 16,  # 16: continuous operation (speed control)
    "ContinusOperationWithPress": 17,  # 17: continuous operation (push-motion)
    "ContinusOperationWithTorque": 18,  # 18: continuous operation (torque motion)
    "AbsolutePositionPress": 20,  # 20:absolute positioning push-motion
    "RerativePositionWithOrderPosition": 21,  # 21:Incremental positioning push-motion (based on command position)
    "RerativePositionWithDetectPosition": 22,  # 22:Incremental positioning push-motion (based on feedback position)
}
ControlMethod_Default = ControlMethod["RelativePositionBasedOnOrderPosition"]

# Position (reference address +2,3) default:0
# -2147483648 to 2147483647 step
Position_Max = 2147483647
Position_Default = 0
Position_Min = -2147483648

# Operation speed (reference address +4,5) default: 1000
# -4000000 to 4000000 Hz
Speed_Max = 4000000
Speed_Default = 1000
Speed_Min = -4000000

# Starting/changing rate (reference address +6, 7) default: 1000000
# 1 to 1000000000 (1=0.01Khz/sec or 1= 0.001ms/Khz)
ChangeSpeed_Max = 1000000000
ChangeSpeed_Default = 1000
ChangeSpeed_Min = 1

# Stopping Deceleration (reference address +8, 9) default: 1000000
# 1 to 1000000000 (1=0.01Khz/sec or 1= 0.001ms/Khz)
Stop_Max = 1000000000
Stop_Default = 1000
Stop_Min = 1

# Operating current (reference address +10, 11) default: 1000
# 0 to 1000(1=0.1%)
MotionSupply_Max = 1000
MotionSupply_Default = 1000
MotionSupply_Min = 0

# Drive-complete delay time (reference address +12, 13) default: 0
# 0 to 65535(1=0.001s)
MotionFinishDelay_Max = 65535
MotionFinishDelay_Default = 0
MotionFinishDelay_Min = 0

# Link (reference address +14, 15) default:0
MergeMethod = {
    "NoMerge": 0,  # 0:No Link
    "SelfSend": 1,  # 1:Manual sequential
    "AutoSend": 2,  # 2:Automatic sequential
    "SurfaceSend": 3,  # 3:Continuous form connection
}
Merge_Default = MergeMethod["NoMerge"]

# Next data No. (reference address +16, 17) default:-1
# -256:STOP, -2:(+2), -1:(+1), 0 to 255: Operation data number
MergeTo_Stop = -256
MergeTo_PlusTwo = -2
MergeTo_PlusOne = -1
MergeTo_Max = 255
MergeTo_Default = MergeTo_PlusOne
MergeTo_Min = 0

# Area offset (reference address +18, 19）default : 0
# -2147483648 to 2147483647 step
OffsetArea_Max = 2147483647
OffsetArea_Default = 0
OffsetArea_Min = -2147483648

# Area Width (reference address +20, 21） default: -1
# -1: Disabled
# 0 to　4194303 step
WidthArea_Disable = -1
WidthArea_Max = 4194303
WidthArea_Default = WidthArea_Disable
WidthArea_Min = 0

# Loop count (reference address +22, 23） default: 0
# 0: No loop
# 2 to 255: 2-255 loop
CountLoop_Max = 255
CountLoop_Default = 0
countLoop_Min = 2

# Loop offset (reference address +24, 25） default : 0
# -4194304 to 4194303 step
PositionOffset_Max = 4194303
PositionOffset_Default = 0
PositionOffset_Min = -4194304

# Loop end No. (reference address +26, 27） default:0
FinishLoop = {"Enable": 0, "End": 1}  # 0: no loop end point  # 1: loop end point

FinishLoop_Default = FinishLoop["Enable"]

# (Low) I/O event No. (reference address +28, 29） default:-1
# -1: Disable
# 0 to 31: Operation I/O event number
WeakEvent_Disable = -1
WeakEvent_Max = 31
WeakEvent_Default = WeakEvent_Disable
WeakEvent_Min = 0

# (High) I/O event No. (reference address +30, 31） default:-1
# -1: Disable
# 0 to 31: Operation I/O event No.
StrongEvent_Disable = -1
StrongEvent_Max = 31
StrongEvent_Default = StrongEvent_Disable
StrongEvent_Min = 0
