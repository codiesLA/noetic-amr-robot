from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian
import AZDKDParameter
import AZDKDDirectParameter

method = AZDKDParameter.ControlMethod_Default # Control Method (base address +0,1) default: 2
position = AZDKDParameter.Position_Default # Position (reference address +2,3) default:0
speed = AZDKDParameter.Speed_Default # Operation speed (reference address +4,5) default: 1000
changeSpeed = AZDKDParameter.ChangeSpeed_Default # Starting/changing rate (reference address +6, 7) default: 1000000
stop = AZDKDParameter.Stop_Default # Stopping Deceleration (reference address +8, 9) default: 1000000
motionSupply = AZDKDParameter.MotionSupply_Default # Operating current (reference address +10, 11) default: 1000
motionFinishDelay = AZDKDParameter.MotionFinishDelay_Default # Drive-complete delay time (reference address +12, 13) default: 0
merge = AZDKDParameter.Merge_Default # Link (reference address +14, 15 default:0
mergeTo = AZDKDParameter.MergeTo_Default # Next data No. (reference address +16, 17) default:-1
offsetArea = AZDKDParameter.OffsetArea_Default # Area offset (reference address +18, 19）default : 0
widthArea = AZDKDParameter.WidthArea_Default # Area Width (reference address +20, 21） default: -1
countLoop = AZDKDParameter.CountLoop_Default # Loop count (reference address +22, 23） default: 0
postionOffset = AZDKDParameter.PositionOffset_Default # Loop offset (reference address +24, 25） default : 0
finishLoop = AZDKDParameter.FinishLoop_Default # Loop end No. (reference address +26, 27） default:0
weakEvent = AZDKDParameter.WeakEvent_Default # (Low) I/O event No. (reference address +28, 29） default:-1
strongEvent =  AZDKDParameter.StrongEvent_Default # (High) I/O event No. (reference address +30, 31） default:-1

def makeMotionParameter():
    builder = BinaryPayloadBuilder(byteorder=Endian.Big)
    builder.add_32bit_int(method) # Method (base address +0,1)
    builder.add_32bit_int(position) # Position (reference address +2,3)
    builder.add_32bit_int(speed) # Operation speed (reference address +4,5)
    builder.add_32bit_int(changeSpeed) # Starting/changing rate (reference address +6, 7)
    builder.add_32bit_int(stop) # Stopping Deceleration (reference address +8, 9)
    builder.add_32bit_int(motionSupply) # Operating current (reference address +10, 11)
    builder.add_32bit_int(motionFinishDelay)  # Drive-complete delay time (reference address +12, 13)
    builder.add_32bit_int(merge) # Link (reference address +14, 15）
    builder.add_32bit_int(mergeTo)# Next data No. (reference address +16, 17)
    builder.add_32bit_int(offsetArea) # Area offset (reference address +18, 19）
    builder.add_32bit_int(widthArea) # Area Width (reference address +20, 21）
    builder.add_32bit_int(countLoop)  # Loop count (reference address +22, 23）
    builder.add_32bit_int(postionOffset) # Loop offset (reference address +24, 25）
    builder.add_32bit_int(finishLoop)  # Loop end No. (reference address +26, 27）
    builder.add_32bit_int(weakEvent) # (Low) I/O event No. (reference address +28, 29）
    builder.add_32bit_int(strongEvent) # (High) I/O event No. (reference address +30, 31）
    return builder.build()

def getAddress(motionNumber):
    return motionNumber * AZDKDParameter.MotionNumAdress_Pitch + AZDKDParameter.MotionNumAdress_Min
