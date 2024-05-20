import AZDKDDirectMessageManager
import AZDKDParameter

class AZDKD:
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
    
    def __init__(self, method):
        self.method = method
    
    def directComand_(self, vel):
        AZDKDDirectMessageManager.method = AZDKDParameter.ControlMethod[self.method]
        AZDKDDirectMessageManager.speed = vel 
        addr = AZDKDDirectMessageManager.getAddress()
        val = AZDKDDirectMessageManager.makeMotionParameter()
        
        return addr, val
