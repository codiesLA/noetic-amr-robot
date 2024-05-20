import AZDKDParameter

# Driving No.
MotionNum_Min = 0  # Dec:88, Hex:0x0058
MotionNum_Default = 0
MotionNum_Max = 255  # Dec:16408, Hex: 255*64+88
MotionNumAdress_Min = 88
MotionNumAdress_Max = 16408
MotionNumAdress_Pitch = 64

# Control Method (base address +0,1) default: 2
ControlMethod_Default = AZDKDParameter.ControlMethod[
    "RelativePositionBasedOnOrderPosition"
]

# Position (reference address +2,3) default:0
# -2147483648 to 2147483647 step
Position_Max = AZDKDParameter.Position_Max
Position_Default = AZDKDParameter.Position_Default
Position_Min = AZDKDParameter.Position_Min

# Operation speed (reference address +4,5) default: 1000
# -4,000,000 to 4,000,000 Hz
Speed_Max = AZDKDParameter.Speed_Max
Speed_Default = AZDKDParameter.Speed_Default
Speed_Min = AZDKDParameter.Speed_Min

# Starting/changing rate (reference address +6, 7) default: 1000000
# 1 to 1,000,000,000 (1=0.01Khz/sec or 1= 0.001ms/Khz)
ChangeSpeed_Max = AZDKDParameter.ChangeSpeed_Max
ChangeSpeed_Default = AZDKDParameter.ChangeSpeed_Default
ChangeSpeed_Min = AZDKDParameter.ChangeSpeed_Min

# Stopping Deceleration (reference address +8, 9) default: 1000000
# 1 to 1,000,000,000 (1=0.01Khz/sec or 1= 0.001ms/Khz)
Stop_Max = AZDKDParameter.Stop_Max
Stop_Default = AZDKDParameter.Stop_Default
Stop_Min = AZDKDParameter.Stop_Min

# Operating current (reference address +10, 11) default: 1000
# 0 to 1,000(1=0.1%)
MotionSupply_Max = AZDKDParameter.MotionSupply_Max
MotionSupply_Default = AZDKDParameter.MotionSupply_Default
MotionSupply_Min = AZDKDParameter.MotionSupply_Min

# Reflection trigger (reference address +12, 13） default:0
FeedbackTrigger = {
    "MotionNo": -7,  # -7:Operation data number update
    "Method": -6,  # -6:Operation type update
    "Position": -5,  # -5:Position update
    "Speed": -4,  # -4:Speed update
    "ChangeSpeed": -3,  # -3:Acceleration/deceleration rate update
    "Stop": -2,  # -2:Stopping deceleration update
    "MotionSupply": -1,  # -1:Operation current update
    "Disable": 0,  # 0:No use feedback
    "AllDataFeedback": 1,  # 1:Feedback all data
}
FeedbackTrigger_Default = FeedbackTrigger["AllDataFeedback"]

# Transport destination (reference address +14, 15） default:0
TransportDestination = {
    "ExecutionMemory": 0,  # 0：execution memory
    "BufferMemory": 1,  # 1: buffer memory
}
TransportDestination_Default = TransportDestination["ExecutionMemory"]
